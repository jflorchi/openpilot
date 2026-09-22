#include "selfdrive/ui/ui.h"

#include <array>
#include <cassert>
#include <cmath>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "selfdrive/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00
#define BACKLIGHT_OFFROAD 50

#define UI_DT (1.0 / UI_FREQ)

// The model output is noisy frame to frame and switches between plan hypotheses, which is very
// visible on the drawn path. Controls smooth this out in the MPC, the UI draws the raw output.
#define PATH_TS 0.10
// Snap to the model instead of filtering when it moves this much in one frame, so the path stays
// responsive to real maneuvers and to the model resetting.
#define PATH_MAX_JUMP 1.5
// Radar dRel is only used to shorten the drawn path, but it's noisy enough to make the end of the
// path pump in and out at the frame rate.
#define LEAD_D_TS 0.20

// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, QPointF *out) {
  const float margin = 500.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix, Ep);

  // Project.
  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    *out = point;
    return true;
  }
  return false;
}

static int get_path_length_idx(const cereal::ModelDataV2::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 1; i < TRAJECTORY_SIZE && line_x[i] <= path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

// Low-pass filtered model path in car space. Exposes the same accessors as the capnp reader so it
// can be handed to update_line_data() in its place.
struct FilteredPath {
  std::array<float, TRAJECTORY_SIZE> x = {}, y = {}, z = {};

  const std::array<float, TRAJECTORY_SIZE> &getX() const { return x; }
  const std::array<float, TRAJECTORY_SIZE> &getY() const { return y; }
  const std::array<float, TRAJECTORY_SIZE> &getZ() const { return z; }

  void update(const cereal::ModelDataV2::XYZTData::Reader &line, uint64_t cur_started_frame) {
    const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
    const float k = (UI_DT / PATH_TS) / (1.0 + UI_DT / PATH_TS);

    bool reset = !valid || cur_started_frame != started_frame;
    for (int i = 0; !reset && i < TRAJECTORY_SIZE; i++) {
      reset = std::abs(line_y[i] - y[i]) > PATH_MAX_JUMP;
    }

    for (int i = 0; i < TRAJECTORY_SIZE; i++) {
      x[i] = reset ? line_x[i] : (1. - k) * x[i] + k * line_x[i];
      y[i] = reset ? line_y[i] : (1. - k) * y[i] + k * line_y[i];
      z[i] = reset ? line_z[i] : (1. - k) * z[i] + k * line_z[i];
    }
    started_frame = cur_started_frame;
    valid = true;
  }

private:
  bool valid = false;
  uint64_t started_frame = 0;
};

static FilteredPath path_filter;
static FirstOrderFilter lead_d_filter(0.0, LEAD_D_TS, UI_DT);
static bool lead_d_filter_valid = false;

static void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::ModelDataV2::XYZTData::Reader &line) {
  for (int i = 0; i < 2; ++i) {
    auto lead_data = (i == 0) ? radar_state.getLeadOne() : radar_state.getLeadTwo();
    if (lead_data.getStatus()) {
      float z = line.getZ()[get_path_length_idx(line, lead_data.getDRel())];
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
    }
  }
}

// Linearly interpolates the line at the given distance, returns false if the line doesn't reach it.
template <typename LineT>
static bool interp_line_at(const LineT &line, float distance, float *y_out, float *z_out) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
  for (int i = 1; i < TRAJECTORY_SIZE; i++) {
    if (line_x[i] >= distance) {
      const float dx = line_x[i] - line_x[i - 1];
      const float t = dx > 0 ? std::clamp((distance - line_x[i - 1]) / dx, 0.0f, 1.0f) : 0.0f;
      *y_out = line_y[i - 1] + t * (line_y[i] - line_y[i - 1]);
      *z_out = line_z[i - 1] + t * (line_z[i] - line_z[i - 1]);
      return true;
    }
  }
  return false;
}

template <typename LineT>
static void update_line_data(const UIState *s, const LineT &line,
                             float y_off, float z_off, line_vertices_data *pvd, float max_distance, bool allow_invert=true) {
  const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();

  std::vector<QPointF> left_points, right_points;
  auto add_point = [&](float x, float y, float z) {
    QPointF left, right;
    bool l = calib_frame_to_full_frame(s, x, y - y_off, z + z_off, &left);
    bool r = calib_frame_to_full_frame(s, x, y + y_off, z + z_off, &right);
    if (l && r) {
      // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
      if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
        return;
      }
      left_points.push_back(left);
      right_points.push_back(right);
    }
  };

  for (int i = 0; i < TRAJECTORY_SIZE && line_x[i] < max_distance; i++) {
    add_point(line_x[i], line_y[i], line_z[i]);
  }
  // End exactly at max_distance. Truncating at a trajectory point instead makes the tip of the line
  // snap by a whole segment (~15m at highway speed) as max_distance moves.
  float end_y, end_z;
  if (interp_line_at(line, max_distance, &end_y, &end_z)) {
    add_point(max_distance, end_y, end_z);
  }

  pvd->cnt = 2 * left_points.size();
  assert(left_points.size() == right_points.size());
  assert(pvd->cnt <= std::size(pvd->v));

  for (int left_idx = 0; left_idx < left_points.size(); left_idx++){
    int right_idx = 2 * left_points.size() - left_idx - 1;
    pvd->v[left_idx] = left_points[left_idx];
    pvd->v[right_idx] = right_points[left_idx];
  }
}

static void update_model(UIState *s, const cereal::ModelDataV2::Reader &model) {
  UIScene &scene = s->scene;
  path_filter.update(model.getPosition(), scene.started_frame);
  float max_distance = std::clamp(path_filter.getX()[TRAJECTORY_SIZE - 1],
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.025 * scene.lane_line_probs[i], 0, &scene.lane_line_vertices[i], max_distance);
  }

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.025, 0, &scene.road_edge_vertices[i], max_distance);
  }

  // update path
  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
  if (lead_one.getStatus()) {
    if (!lead_d_filter_valid) {
      lead_d_filter.reset(lead_one.getDRel());
      lead_d_filter_valid = true;
    }
    const float lead_d = lead_d_filter.update(lead_one.getDRel()) * 2.;
    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
  } else {
    lead_d_filter_valid = false;
  }
  update_line_data(s, path_filter, scene.end_to_end ? 0.9 : 0.5, 1.22, &scene.track_vertices, max_distance, false);
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("liveCalibration")) {
    auto rpy_list = sm["liveCalibration"].getLiveCalibration().getRpyCalib();
    Eigen::Vector3d rpy;
    rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0,1,0,
                        0,0,1,
                        1,0,0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i,j);
      }
    }
  }
  if (s->worldObjectsVisible()) {
    if (sm.updated("modelV2")) {
      update_model(s, sm["modelV2"].getModelV2());
    }
    if (sm.updated("radarState") && sm.rcv_frame("modelV2") > s->scene.started_frame) {
      update_leads(s, sm["radarState"].getRadarState(), sm["modelV2"].getModelV2().getPosition());
    }
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    scene.longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (!scene.started && sm.updated("sensorEvents")) {
    for (auto sensor : sm["sensorEvents"].getSensorEvents()) {
      if (sensor.which() == cereal::SensorEventData::ACCELERATION) {
        auto accel = sensor.getAcceleration().getV();
        if (accel.totalSize().wordCount) { // TODO: sometimes empty lists are received. Figure out why
          scene.accel_sensor = accel[2];
        }
      } else if (sensor.which() == cereal::SensorEventData::GYRO_UNCALIBRATED) {
        auto gyro = sensor.getGyroUncalibrated().getV();
        if (gyro.totalSize().wordCount) {
          scene.gyro_sensor = gyro[1];
        }
      }
    }
  }
  if (!Hardware::TICI() && sm.updated("roadCameraState")) {
    auto camera_state = sm["roadCameraState"].getRoadCameraState();

    float max_lines = Hardware::EON() ? 5408 : 1904;
    float max_gain = Hardware::EON() ? 1.0: 10.0;
    float max_ev = max_lines * max_gain;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  } else if (Hardware::TICI() && sm.updated("wideRoadCameraState")) {
    auto camera_state = sm["wideRoadCameraState"].getWideRoadCameraState();

    float max_lines = 1618;
    float max_gain = 10.0;
    float max_ev = max_lines * max_gain / 6;

    float ev = camera_state.getGain() * float(camera_state.getIntegLines());

    scene.light_sensor = std::clamp<float>(1.0 - (ev / max_ev), 0.0, 1.0);
  }
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
}

void ui_update_params(UIState *s) {
  s->scene.is_metric = Params().getBool("IsMetric");
}

void UIState::updateStatus() {
  // reflect the actual laneless decision (manual or automatic) in the path color
  if (scene.started && sm->updated("lateralPlan")) {
    scene.end_to_end = !(*sm)["lateralPlan"].getLateralPlan().getUseLaneLines();
  }

  if (scene.started && sm->updated("controlsState")) {
    auto controls_state = (*sm)["controlsState"].getControlsState();
    auto alert_status = controls_state.getAlertStatus();
    auto state = controls_state.getState();
    if (alert_status == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      status = STATUS_WARNING;
    } else if (alert_status == cereal::ControlsState::AlertStatus::CRITICAL) {
      status = STATUS_ALERT;
    } else if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
      status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
      wide_camera = Hardware::TICI() ? Params().getBool("WideCameraOnly") : false;
    }
    started_prev = scene.started;
    emit offroadTransition(!scene.started);
  }
}

UIState::UIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState", "roadCameraState",
    "pandaStates", "carParams", "driverMonitoringState", "sensorEvents", "carState", "liveLocationKalman",
    "wideRoadCameraState", "managerState", "lateralPlan",
  });

  Params params;
  wide_camera = Hardware::TICI() ? Params().getBool("WideCameraOnly") : false;
  prime_type = std::atoi(params.get("PrimeType").c_str());

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &UIState::update);
  timer->start(1000 / UI_FREQ);
}

void UIState::update() {
  update_sockets(this);
  update_state(this);
  updateStatus();

  if (sm->frame % UI_FREQ == 0) {
    watchdog_kick();
  }
  emit uiUpdate(*this);
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
  setAwake(true);
  resetInteractiveTimout();

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Device::update);
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);

  // TODO: remove from UIState and use signals
  uiState()->awake = awake;
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }
}

void Device::resetInteractiveTimout() {
  interactive_timeout = (ignition_on ? 10 : 30) * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = BACKLIGHT_OFFROAD;
  if (s.scene.started) {
    // Scale to 0% to 100%
    clipped_brightness = 100.0 * s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  }

  if (brightness != last_brightness) {
    if (!brightness_future.isRunning()) {
      brightness_future = QtConcurrent::run(Hardware::set_brightness, brightness);
      last_brightness = brightness;
    }
  }
}

bool Device::motionTriggered(const UIState &s) {
  static float accel_prev = 0;
  static float gyro_prev = 0;

  bool accel_trigger = abs(s.scene.accel_sensor - accel_prev) > 0.2;
  bool gyro_trigger = abs(s.scene.gyro_sensor - gyro_prev) > 0.15;

  gyro_prev = s.scene.gyro_sensor;
  accel_prev = (accel_prev * (accel_samples - 1) + s.scene.accel_sensor) / accel_samples;

  return (!awake && accel_trigger && gyro_trigger);
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off || motionTriggered(s)) {
    resetInteractiveTimout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
    emit interactiveTimout();
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

UIState *uiState() {
  static UIState ui_state;
  return &ui_state;
}
