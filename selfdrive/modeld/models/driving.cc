#include "selfdrive/modeld/models/driving.h"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <vector>
#include <cstdio>
#include <cstring>

#include <eigen3/Eigen/Dense>

#include "common/clutil.h"
#include "common/params.h"
#include "common/timing.h"
#include "common/swaglog.h"

constexpr float FCW_THRESHOLD_5MS2_HIGH = 0.15;
constexpr float FCW_THRESHOLD_5MS2_LOW = 0.05;
constexpr float FCW_THRESHOLD_3MS2 = 0.7;

std::array<float, 5> prev_brake_5ms2_probs = {0,0,0,0,0};
std::array<float, 3> prev_brake_3ms2_probs = {0,0,0};

// #define DUMP_YUV

template<class T, size_t size>
constexpr const kj::ArrayPtr<const T> to_kj_array_ptr(const std::array<T, size> &arr) {
  return kj::ArrayPtr(arr.data(), arr.size());
}

void model_init(ModelState* s, cl_device_id device_id, cl_context context) {
#ifdef C2PORT
  s->frame = new ModelFrame(device_id, context, c2port::INPUT_SCALE, c2port::INPUT_OFFSET);
  s->wide_frame = new ModelFrame(device_id, context, c2port::INPUT_SCALE, c2port::INPUT_OFFSET);

  s->thneed = std::make_unique<Thneed>(true);
  s->thneed->load("models/supercombo_c2port.thneed");
  s->thneed->clexec();
  s->in_imgs = s->thneed->input_index("imgs");
  s->in_desire = s->thneed->input_index("desire_tokens");
  s->in_feat = s->thneed->input_index("feat_hist");
  s->in_traffic = s->thneed->input_index("traffic_convention");
  assert(s->in_imgs >= 0 && s->in_desire >= 0 && s->in_feat >= 0 && s->in_traffic >= 0);
  assert(s->thneed->input_sizes[s->in_imgs] == 4 * c2port::FRAME_SIZE * sizeof(float));
  assert(s->thneed->input_sizes[s->in_desire] == s->desire_tokens.size() * sizeof(float));
  assert(s->thneed->input_sizes[s->in_feat] == s->feat_tokens.size() * sizeof(float));

  // frames before the first 5 are all-zero pixels, like the upstream model's zero-initialized image queue
  for (auto &r : s->ring) {
    r = std::make_unique<float[]>(c2port::RING * c2port::FRAME_SIZE);
    std::fill_n(r.get(), c2port::RING * c2port::FRAME_SIZE, c2port::INPUT_OFFSET);
  }
  s->imgs = std::make_unique<float[]>(4 * c2port::FRAME_SIZE);

  s->traffic_convention[Params().getBool("IsRHD") ? 1 : 0] = 1.0;
  return;
#endif
  s->frame = new ModelFrame(device_id, context);
  s->wide_frame = new ModelFrame(device_id, context);

#ifdef USE_THNEED
  s->m = std::make_unique<ThneedModel>("models/supercombo.thneed",
#elif USE_ONNX_MODEL
  s->m = std::make_unique<ONNXModel>("models/supercombo.onnx",
#else
  s->m = std::make_unique<SNPEModel>("models/supercombo.dlc",
#endif
   &s->output[0], NET_OUTPUT_SIZE, USE_GPU_RUNTIME, true);

#ifdef TEMPORAL
  s->m->addRecurrent(&s->output[OUTPUT_SIZE], TEMPORAL_SIZE);
#endif

#ifdef DESIRE
  s->m->addDesire(s->pulse_desire, DESIRE_LEN);
#endif

#ifdef TRAFFIC_CONVENTION
  const int idx = Params().getBool("IsRHD") ? 1 : 0;
  s->traffic_convention[idx] = 1.0;
  s->m->addTrafficConvention(s->traffic_convention, TRAFFIC_CONVENTION_LEN);
#endif
}


#ifdef C2PORT
// Repack the new model output into the ModelOutput layout the rest of this file (and modeld) reads.
static void c2port_repack(ModelState *s) {
  using namespace c2port;
  const float *r = s->raw.data();
  auto &out = *reinterpret_cast<ModelOutput*>(s->output.data());

  // plan: one hypothesis (means + log-stds, same 15-value element layout); make it the best of the 5
  auto &plans = const_cast<ModelOutputPlans&>(out.plans);
  for (int h = 0; h < PLAN_MHP_N; h++) {
    auto &p = plans.prediction[h];
    std::memcpy(p.mean.data(), r + PLAN, sizeof(p.mean));
    std::memcpy(p.std.data(), r + PLAN + sizeof(p.mean) / sizeof(float), sizeof(p.std));
    p.prob = (h == 0) ? 1e3f : -1e3f;
  }
  static_assert(sizeof(ModelOutputPlanPrediction::mean) / sizeof(float) == 495);

  // lane lines (means, log-stds, 4 x {deprecated, valid} logits) and road edges: identical layouts
  auto &lanes = const_cast<ModelOutputLaneLines&>(out.lane_lines);
  std::memcpy(&lanes.mean, r + LANE_LINES, sizeof(lanes.mean) + sizeof(lanes.std));
  std::memcpy(&lanes.prob, r + LANE_LINES_PROB, sizeof(lanes.prob));
  static_assert(sizeof(ModelOutputLaneLines::mean) * 2 == (LANE_LINES_PROB - LANE_LINES) * sizeof(float));
  auto &edges = const_cast<ModelOutputRoadEdges&>(out.road_edges);
  std::memcpy(&edges, r + ROAD_EDGES, (LEAD - ROAD_EDGES) * sizeof(float));
  static_assert(sizeof(ModelOutputRoadEdges) == (LEAD - ROAD_EDGES) * sizeof(float));

  // stop lines: not produced by this model
  auto &stop = const_cast<ModelOutputStopLines&>(out.stop_lines);
  for (auto &p : stop.prediction) p.prob = -1e3f;
  stop.prob = -1e3f;

  // meta: engaged, 5 x {gas/brake disengage, steer override, hard brake 3/4/5}, 6 x {gas press, brake press, blinker l/r}
  auto &meta = const_cast<ModelOutputMeta&>(out.meta);
  std::memcpy(meta.desire_state_prob.array.data(), r + DESIRE_STATE, DESIRE_LEN * sizeof(float));
  meta.engaged_prob = r[META];
  for (int i = 0; i < DISENGAGE_LEN; i++) {
    auto &d = meta.disengage_prob[i];
    const float *m = r + META + 1 + 6 * i;
    d.gas_disengage = m[0]; d.brake_disengage = m[1]; d.steer_override = m[2];
    d.brake_3ms2 = m[3]; d.brake_4ms2 = m[4]; d.brake_5ms2 = m[5];
    d.gas_pressed = r[META + 31 + 4 * i];
  }
  for (int i = 0; i < BLINKER_LEN; i++) {
    meta.blinker_prob[i].left = r[META + 33 + 4 * i];
    meta.blinker_prob[i].right = r[META + 34 + 4 * i];
  }
  for (int i = 0; i < DESIRE_PRED_LEN; i++) {
    std::memcpy(meta.desire_pred_prob[i].array.data(), r + DESIRE_PRED + i * DESIRE_LEN, DESIRE_LEN * sizeof(float));
  }

  // pose: means then log-stds of (trans, rot)
  std::memcpy(const_cast<ModelOutputPose*>(&out.pose), r + POSE, sizeof(ModelOutputPose));

  // leads: new per-time-offset layout, kept in the unused recurrent tail (see fill_model)
  auto &leads = *reinterpret_cast<Leads*>(s->output.data() + ::OUTPUT_SIZE);
  std::memcpy(leads.mean.data(), r + LEAD, sizeof(leads.mean) + sizeof(leads.std));
  std::memcpy(leads.prob.data(), r + LEAD_PROB, sizeof(leads.prob));
}

ModelOutput* c2port_eval(ModelState *s, VisionBuf *buf, VisionBuf *wbuf, const mat3 &transform, const mat3 &transform_wide,
                         bool prepare_only) {
  using namespace c2port;
  // desire history (pulses), pooled to 25 tokens (max over groups of 4)
  std::memmove(s->desire_hist.data(), s->desire_hist.data() + DESIRE_LEN, (DESIRE_HIST - 1) * DESIRE_LEN * sizeof(float));
  std::memcpy(s->desire_hist.data() + (DESIRE_HIST - 1) * DESIRE_LEN, s->pulse_desire, DESIRE_LEN * sizeof(float));
  for (int t = 0; t < 25; t++) {
    for (int d = 0; d < DESIRE_LEN; d++) {
      float m = s->desire_hist[(4 * t) * DESIRE_LEN + d];
      for (int k = 1; k < 4; k++) m = std::max(m, s->desire_hist[(4 * t + k) * DESIRE_LEN + d]);
      s->desire_tokens[t * DESIRE_LEN + d] = m;
    }
  }

  // frames: narrow and wide (the narrow camera again on the comma two), each into its ring at ring_head
  VisionBuf *wide = (wbuf != nullptr) ? wbuf : buf;
  std::memcpy(s->ring[0].get() + s->ring_head * FRAME_SIZE, s->frame->prepare_single(buf->buf_cl, buf->width, buf->height, transform),
              FRAME_SIZE * sizeof(float));
  std::memcpy(s->ring[1].get() + s->ring_head * FRAME_SIZE, s->wide_frame->prepare_single(wide->buf_cl, wide->width, wide->height, transform_wide),
              FRAME_SIZE * sizeof(float));
  const int slot_t = s->ring_head, slot_t4 = (s->ring_head + 1) % RING;  // newest and 4 frames ago
  s->ring_head = (s->ring_head + 1) % RING;

  auto push_feature = [&](const float *f) {
    std::memmove(s->feat_hist.data(), s->feat_hist.data() + FEAT_LEN, (FEAT_HIST - 1) * FEAT_LEN * sizeof(float));
    std::memcpy(s->feat_hist.data() + (FEAT_HIST - 1) * FEAT_LEN, f, FEAT_LEN * sizeof(float));
  };
  if (prepare_only) {
    // dropped frame: keep the 20 Hz spacing of the feature history by repeating the latest feature
    std::array<float, FEAT_LEN> last;
    std::memcpy(last.data(), s->feat_hist.data() + (FEAT_HIST - 1) * FEAT_LEN, sizeof(last));
    push_feature(last.data());
    return nullptr;
  }

  // model input 'imgs': narrow t-4, narrow t, wide t-4, wide t
  float *imgs = s->imgs.get();
  std::memcpy(imgs + 0 * FRAME_SIZE, s->ring[0].get() + slot_t4 * FRAME_SIZE, FRAME_SIZE * sizeof(float));
  std::memcpy(imgs + 1 * FRAME_SIZE, s->ring[0].get() + slot_t * FRAME_SIZE, FRAME_SIZE * sizeof(float));
  std::memcpy(imgs + 2 * FRAME_SIZE, s->ring[1].get() + slot_t4 * FRAME_SIZE, FRAME_SIZE * sizeof(float));
  std::memcpy(imgs + 3 * FRAME_SIZE, s->ring[1].get() + slot_t * FRAME_SIZE, FRAME_SIZE * sizeof(float));

  // feature history tokens: every 4th of the 96-step history (rows 0, 4, .., 92), the last 8 = rows 64 .. 92,
  // i.e. the features of frames t-32 .. t-4 (taken before this frame's feature is pushed)
  constexpr int FEAT_TOKEN_ROW0 = FEAT_HIST - 4 * FEAT_TOKENS;
  static_assert(FEAT_TOKEN_ROW0 == 64);
  for (int k = 0; k < FEAT_TOKENS; k++) {
    std::memcpy(s->feat_tokens.data() + k * FEAT_LEN, s->feat_hist.data() + (FEAT_TOKEN_ROW0 + 4 * k) * FEAT_LEN, FEAT_LEN * sizeof(float));
  }

  std::vector<float*> finputs(s->thneed->inputs.size(), nullptr);
  finputs[s->in_imgs] = imgs;
  finputs[s->in_desire] = s->desire_tokens.data();
  finputs[s->in_feat] = s->feat_tokens.data();
  finputs[s->in_traffic] = s->traffic_convention;
  s->thneed->write_inputs(finputs.data());
  std::vector<float*> no_copy(finputs.size(), nullptr);  // inputs are already written; skip execute()'s memcpy
  if (!s->recorded) {
    // first run: record the kernels, then replay the recording on later frames (like ThneedModel)
    s->thneed->record = true;
    s->thneed->clexec();
    s->thneed->copy_output(s->raw.data());
    s->thneed->stop();
    s->recorded = true;
  } else {
    s->thneed->execute(no_copy.data(), s->raw.data());
  }

  // debug: C2PORT_DUMP=N writes the inputs/outputs of the first N executed frames to C2PORT_DUMP_DIR (for c2port/check_dump.py)
  static const int dump_frames = getenv("C2PORT_DUMP") ? atoi(getenv("C2PORT_DUMP")) : 0;
  static int dumped = 0;
  if (dumped < dump_frames) {
    const char *dir = getenv("C2PORT_DUMP_DIR") ? getenv("C2PORT_DUMP_DIR") : "/data/c2port/dump";
    auto dump = [&](const char *what, const void *data, size_t size) {
      char fn[256];
      snprintf(fn, sizeof(fn), "%s/%03d_%s.bin", dir, dumped, what);
      FILE *f = fopen(fn, "wb");
      if (f) { fwrite(data, 1, size, f); fclose(f); }
    };
    dump("imgs", imgs, 4 * FRAME_SIZE * sizeof(float));
    dump("desire", s->desire_tokens.data(), sizeof(s->desire_tokens));
    dump("feat", s->feat_tokens.data(), sizeof(s->feat_tokens));
    dump("traffic", s->traffic_convention, sizeof(s->traffic_convention));
    dump("out", s->raw.data(), sizeof(s->raw));
    dumped++;
  }

  push_feature(s->raw.data() + HIDDEN);
  c2port_repack(s);
  return (ModelOutput*)&s->output;
}
#endif

ModelOutput* model_eval_frame(ModelState* s, VisionBuf* buf, VisionBuf* wbuf,
                              const mat3 &transform, const mat3 &transform_wide, float *desire_in, bool prepare_only) {
#ifdef DESIRE
  if (desire_in != NULL) {
    for (int i = 1; i < DESIRE_LEN; i++) {
      // Model decides when action is completed
      // so desire input is just a pulse triggered on rising edge
      if (desire_in[i] - s->prev_desire[i] > .99) {
        s->pulse_desire[i] = desire_in[i];
      } else {
        s->pulse_desire[i] = 0.0;
      }
      s->prev_desire[i] = desire_in[i];
    }
  }
#endif

#ifdef C2PORT
  return c2port_eval(s, buf, wbuf, transform, transform_wide, prepare_only);
#endif

  // if getInputBuf is not NULL, net_input_buf will be
  auto net_input_buf = s->frame->prepare(buf->buf_cl, buf->width, buf->height, transform, static_cast<cl_mem*>(s->m->getInputBuf()));
  s->m->addImage(net_input_buf, s->frame->buf_size);
  LOGT("Image added");

  if (wbuf != nullptr) {
    auto net_extra_buf = s->wide_frame->prepare(wbuf->buf_cl, wbuf->width, wbuf->height, transform_wide, static_cast<cl_mem*>(s->m->getExtraBuf()));
    s->m->addExtra(net_extra_buf, s->wide_frame->buf_size);
    LOGT("Extra image added");
  }

  if (prepare_only) {
    return nullptr;
  }

  s->m->execute();
  LOGT("Execution finished");

  return (ModelOutput*)&s->output;
}

void model_free(ModelState* s) {
  delete s->frame;
}

void fill_lead(cereal::ModelDataV2::LeadDataV3::Builder lead, const ModelOutputLeads &leads, int t_idx, float prob_t) {
  std::array<float, LEAD_TRAJ_LEN> lead_t = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  const auto &best_prediction = leads.get_best_prediction(t_idx);
  lead.setProb(sigmoid(leads.prob[t_idx]));
  lead.setProbTime(prob_t);
  std::array<float, LEAD_TRAJ_LEN> lead_x, lead_y, lead_v, lead_a;
  std::array<float, LEAD_TRAJ_LEN> lead_x_std, lead_y_std, lead_v_std, lead_a_std;
  for (int i=0; i<LEAD_TRAJ_LEN; i++) {
    lead_x[i] = best_prediction.mean[i].x;
    lead_y[i] = best_prediction.mean[i].y;
    lead_v[i] = best_prediction.mean[i].velocity;
    lead_a[i] = best_prediction.mean[i].acceleration;
    lead_x_std[i] = exp(best_prediction.std[i].x);
    lead_y_std[i] = exp(best_prediction.std[i].y);
    lead_v_std[i] = exp(best_prediction.std[i].velocity);
    lead_a_std[i] = exp(best_prediction.std[i].acceleration);
  }
  lead.setT(to_kj_array_ptr(lead_t));
  lead.setX(to_kj_array_ptr(lead_x));
  lead.setY(to_kj_array_ptr(lead_y));
  lead.setV(to_kj_array_ptr(lead_v));
  lead.setA(to_kj_array_ptr(lead_a));
  lead.setXStd(to_kj_array_ptr(lead_x_std));
  lead.setYStd(to_kj_array_ptr(lead_y_std));
  lead.setVStd(to_kj_array_ptr(lead_v_std));
  lead.setAStd(to_kj_array_ptr(lead_a_std));
}

#ifdef C2PORT
void c2port_fill_lead(cereal::ModelDataV2::LeadDataV3::Builder lead, const c2port::Leads &leads, int t_idx, float prob_t) {
  std::array<float, LEAD_TRAJ_LEN> lead_t = {0.0, 2.0, 4.0, 6.0, 8.0, 10.0};
  std::array<float, LEAD_TRAJ_LEN> lead_x, lead_y, lead_v, lead_a;
  std::array<float, LEAD_TRAJ_LEN> lead_x_std, lead_y_std, lead_v_std, lead_a_std;
  for (int i=0; i<LEAD_TRAJ_LEN; i++) {
    const int o = (t_idx * LEAD_TRAJ_LEN + i) * LEAD_PRED_DIM;
    lead_x[i] = leads.mean[o + 0];
    lead_y[i] = leads.mean[o + 1];
    lead_v[i] = leads.mean[o + 2];
    lead_a[i] = leads.mean[o + 3];
    lead_x_std[i] = exp(leads.std[o + 0]);
    lead_y_std[i] = exp(leads.std[o + 1]);
    lead_v_std[i] = exp(leads.std[o + 2]);
    lead_a_std[i] = exp(leads.std[o + 3]);
  }
  lead.setProb(sigmoid(leads.prob[t_idx]));
  lead.setProbTime(prob_t);
  lead.setT(to_kj_array_ptr(lead_t));
  lead.setX(to_kj_array_ptr(lead_x));
  lead.setY(to_kj_array_ptr(lead_y));
  lead.setV(to_kj_array_ptr(lead_v));
  lead.setA(to_kj_array_ptr(lead_a));
  lead.setXStd(to_kj_array_ptr(lead_x_std));
  lead.setYStd(to_kj_array_ptr(lead_y_std));
  lead.setVStd(to_kj_array_ptr(lead_v_std));
  lead.setAStd(to_kj_array_ptr(lead_a_std));
}
#endif

void fill_meta(cereal::ModelDataV2::MetaData::Builder meta, const ModelOutputMeta &meta_data) {
  std::array<float, DESIRE_LEN> desire_state_softmax;
  softmax(meta_data.desire_state_prob.array.data(), desire_state_softmax.data(), DESIRE_LEN);

  std::array<float, DESIRE_PRED_LEN * DESIRE_LEN> desire_pred_softmax;
  for (int i=0; i<DESIRE_PRED_LEN; i++) {
    softmax(meta_data.desire_pred_prob[i].array.data(), desire_pred_softmax.data() + (i * DESIRE_LEN), DESIRE_LEN);
  }

  std::array<float, DISENGAGE_LEN> lat_long_t = {2,4,6,8,10};
  std::array<float, DISENGAGE_LEN> gas_disengage_sigmoid, brake_disengage_sigmoid, steer_override_sigmoid,
                                   brake_3ms2_sigmoid, brake_4ms2_sigmoid, brake_5ms2_sigmoid;
  for (int i=0; i<DISENGAGE_LEN; i++) {
    gas_disengage_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].gas_disengage);
    brake_disengage_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].brake_disengage);
    steer_override_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].steer_override);
    brake_3ms2_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].brake_3ms2);
    brake_4ms2_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].brake_4ms2);
    brake_5ms2_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].brake_5ms2);
    //gas_pressed_sigmoid[i] = sigmoid(meta_data.disengage_prob[i].gas_pressed);
  }

  std::memmove(prev_brake_5ms2_probs.data(), &prev_brake_5ms2_probs[1], 4*sizeof(float));
  std::memmove(prev_brake_3ms2_probs.data(), &prev_brake_3ms2_probs[1], 2*sizeof(float));
  prev_brake_5ms2_probs[4] = brake_5ms2_sigmoid[0];
  prev_brake_3ms2_probs[2] = brake_3ms2_sigmoid[0];

  bool above_fcw_threshold = true;
  for (int i=0; i<prev_brake_5ms2_probs.size(); i++) {
    float threshold = i < 2 ? FCW_THRESHOLD_5MS2_LOW : FCW_THRESHOLD_5MS2_HIGH;
    above_fcw_threshold = above_fcw_threshold && prev_brake_5ms2_probs[i] > threshold;
  }
  for (int i=0; i<prev_brake_3ms2_probs.size(); i++) {
    above_fcw_threshold = above_fcw_threshold && prev_brake_3ms2_probs[i] > FCW_THRESHOLD_3MS2;
  }

  auto disengage = meta.initDisengagePredictions();
  disengage.setT(to_kj_array_ptr(lat_long_t));
  disengage.setGasDisengageProbs(to_kj_array_ptr(gas_disengage_sigmoid));
  disengage.setBrakeDisengageProbs(to_kj_array_ptr(brake_disengage_sigmoid));
  disengage.setSteerOverrideProbs(to_kj_array_ptr(steer_override_sigmoid));
  disengage.setBrake3MetersPerSecondSquaredProbs(to_kj_array_ptr(brake_3ms2_sigmoid));
  disengage.setBrake4MetersPerSecondSquaredProbs(to_kj_array_ptr(brake_4ms2_sigmoid));
  disengage.setBrake5MetersPerSecondSquaredProbs(to_kj_array_ptr(brake_5ms2_sigmoid));

  meta.setEngagedProb(sigmoid(meta_data.engaged_prob));
  meta.setDesirePrediction(to_kj_array_ptr(desire_pred_softmax));
  meta.setDesireState(to_kj_array_ptr(desire_state_softmax));
  meta.setHardBrakePredicted(above_fcw_threshold);
}

template<size_t size>
void fill_xyzt(cereal::ModelDataV2::XYZTData::Builder xyzt, const std::array<float, size> &t,
               const std::array<float, size> &x, const std::array<float, size> &y, const std::array<float, size> &z) {
  xyzt.setT(to_kj_array_ptr(t));
  xyzt.setX(to_kj_array_ptr(x));
  xyzt.setY(to_kj_array_ptr(y));
  xyzt.setZ(to_kj_array_ptr(z));
}

template<size_t size>
void fill_xyzt(cereal::ModelDataV2::XYZTData::Builder xyzt, const std::array<float, size> &t,
               const std::array<float, size> &x, const std::array<float, size> &y, const std::array<float, size> &z,
               const std::array<float, size> &x_std, const std::array<float, size> &y_std, const std::array<float, size> &z_std) {
  fill_xyzt(xyzt, t, x, y, z);
  xyzt.setXStd(to_kj_array_ptr(x_std));
  xyzt.setYStd(to_kj_array_ptr(y_std));
  xyzt.setZStd(to_kj_array_ptr(z_std));
}

void fill_plan(cereal::ModelDataV2::Builder &framed, const ModelOutputPlanPrediction &plan) {
  std::array<float, TRAJECTORY_SIZE> pos_x, pos_y, pos_z;
  std::array<float, TRAJECTORY_SIZE> pos_x_std, pos_y_std, pos_z_std;
  std::array<float, TRAJECTORY_SIZE> vel_x, vel_y, vel_z;
  std::array<float, TRAJECTORY_SIZE> rot_x, rot_y, rot_z;
  std::array<float, TRAJECTORY_SIZE> rot_rate_x, rot_rate_y, rot_rate_z;

  for(int i=0; i<TRAJECTORY_SIZE; i++) {
    pos_x[i] = plan.mean[i].position.x;
    pos_y[i] = plan.mean[i].position.y;
    pos_z[i] = plan.mean[i].position.z;
    pos_x_std[i] = exp(plan.std[i].position.x);
    pos_y_std[i] = exp(plan.std[i].position.y);
    pos_z_std[i] = exp(plan.std[i].position.z);
    vel_x[i] = plan.mean[i].velocity.x;
    vel_y[i] = plan.mean[i].velocity.y;
    vel_z[i] = plan.mean[i].velocity.z;
    rot_x[i] = plan.mean[i].rotation.x;
    rot_y[i] = plan.mean[i].rotation.y;
    rot_z[i] = plan.mean[i].rotation.z;
    rot_rate_x[i] = plan.mean[i].rotation_rate.x;
    rot_rate_y[i] = plan.mean[i].rotation_rate.y;
    rot_rate_z[i] = plan.mean[i].rotation_rate.z;
  }

  fill_xyzt(framed.initPosition(), T_IDXS_FLOAT, pos_x, pos_y, pos_z, pos_x_std, pos_y_std, pos_z_std);
  fill_xyzt(framed.initVelocity(), T_IDXS_FLOAT, vel_x, vel_y, vel_z);
  fill_xyzt(framed.initOrientation(), T_IDXS_FLOAT, rot_x, rot_y, rot_z);
  fill_xyzt(framed.initOrientationRate(), T_IDXS_FLOAT, rot_rate_x, rot_rate_y, rot_rate_z);
}

void fill_lane_lines(cereal::ModelDataV2::Builder &framed, const std::array<float, TRAJECTORY_SIZE> &plan_t,
                     const ModelOutputLaneLines &lanes) {
  std::array<float, TRAJECTORY_SIZE> left_far_y, left_far_z;
  std::array<float, TRAJECTORY_SIZE> left_near_y, left_near_z;
  std::array<float, TRAJECTORY_SIZE> right_near_y, right_near_z;
  std::array<float, TRAJECTORY_SIZE> right_far_y, right_far_z;
  for (int j=0; j<TRAJECTORY_SIZE; j++) {
    left_far_y[j] = lanes.mean.left_far[j].y;
    left_far_z[j] = lanes.mean.left_far[j].z;
    left_near_y[j] = lanes.mean.left_near[j].y;
    left_near_z[j] = lanes.mean.left_near[j].z;
    right_near_y[j] = lanes.mean.right_near[j].y;
    right_near_z[j] = lanes.mean.right_near[j].z;
    right_far_y[j] = lanes.mean.right_far[j].y;
    right_far_z[j] = lanes.mean.right_far[j].z;
  }

  auto lane_lines = framed.initLaneLines(4);
  fill_xyzt(lane_lines[0], plan_t, X_IDXS_FLOAT, left_far_y, left_far_z);
  fill_xyzt(lane_lines[1], plan_t, X_IDXS_FLOAT, left_near_y, left_near_z);
  fill_xyzt(lane_lines[2], plan_t, X_IDXS_FLOAT, right_near_y, right_near_z);
  fill_xyzt(lane_lines[3], plan_t, X_IDXS_FLOAT, right_far_y, right_far_z);

  framed.setLaneLineStds({
    exp(lanes.std.left_far[0].y),
    exp(lanes.std.left_near[0].y),
    exp(lanes.std.right_near[0].y),
    exp(lanes.std.right_far[0].y),
  });

  framed.setLaneLineProbs({
    sigmoid(lanes.prob.left_far.val),
    sigmoid(lanes.prob.left_near.val),
    sigmoid(lanes.prob.right_near.val),
    sigmoid(lanes.prob.right_far.val),
  });
}

void fill_road_edges(cereal::ModelDataV2::Builder &framed, const std::array<float, TRAJECTORY_SIZE> &plan_t,
                     const ModelOutputRoadEdges &edges) {
  std::array<float, TRAJECTORY_SIZE> left_y, left_z;
  std::array<float, TRAJECTORY_SIZE> right_y, right_z;
  for (int j=0; j<TRAJECTORY_SIZE; j++) {
    left_y[j] = edges.mean.left[j].y;
    left_z[j] = edges.mean.left[j].z;
    right_y[j] = edges.mean.right[j].y;
    right_z[j] = edges.mean.right[j].z;
  }

  auto road_edges = framed.initRoadEdges(2);
  fill_xyzt(road_edges[0], plan_t, X_IDXS_FLOAT, left_y, left_z);
  fill_xyzt(road_edges[1], plan_t, X_IDXS_FLOAT, right_y, right_z);

  framed.setRoadEdgeStds({
    exp(edges.std.left[0].y),
    exp(edges.std.right[0].y),
  });
}

void fill_model(cereal::ModelDataV2::Builder &framed, const ModelOutput &net_outputs) {
  const auto &best_plan = net_outputs.plans.get_best_prediction();
  std::array<float, TRAJECTORY_SIZE> plan_t;
  std::fill_n(plan_t.data(), plan_t.size(), NAN);
  plan_t[0] = 0.0;
  for (int xidx=1, tidx=0; xidx<TRAJECTORY_SIZE; xidx++) {
    // increment tidx until we find an element that's further away than the current xidx
    for (int next_tid = tidx + 1; next_tid < TRAJECTORY_SIZE && best_plan.mean[next_tid].position.x < X_IDXS[xidx]; next_tid++) {
      tidx++;
    }
    if (tidx == TRAJECTORY_SIZE - 1) {
      // if the Plan doesn't extend far enough, set plan_t to the max value (10s), then break
      plan_t[xidx] = T_IDXS[TRAJECTORY_SIZE - 1];
      break;
    }

    // interpolate to find `t` for the current xidx
    float current_x_val = best_plan.mean[tidx].position.x;
    float next_x_val = best_plan.mean[tidx+1].position.x;
    float p = (X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val);
    plan_t[xidx] = p * T_IDXS[tidx+1] + (1 - p) * T_IDXS[tidx];
  }

  fill_plan(framed, best_plan);
  fill_lane_lines(framed, plan_t, net_outputs.lane_lines);
  fill_road_edges(framed, plan_t, net_outputs.road_edges);

  // meta
  fill_meta(framed.initMeta(), net_outputs.meta);

  // leads
  auto leads = framed.initLeadsV3(LEAD_MHP_SELECTION);
  std::array<float, LEAD_MHP_SELECTION> t_offsets = {0.0, 2.0, 4.0};
  for (int i=0; i<LEAD_MHP_SELECTION; i++) {
#ifdef C2PORT
    c2port_fill_lead(leads[i], *reinterpret_cast<const c2port::Leads*>(reinterpret_cast<const float*>(&net_outputs) + OUTPUT_SIZE), i, t_offsets[i]);
#else
    fill_lead(leads[i], net_outputs.leads, i, t_offsets[i]);
#endif
  }
}

void model_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_frame_id_extra, uint32_t frame_id, float frame_drop,
                   const ModelOutput &net_outputs, uint64_t timestamp_eof,
                   float model_execution_time, kj::ArrayPtr<const float> raw_pred, const bool valid) {
  const uint32_t frame_age = (frame_id > vipc_frame_id) ? (frame_id - vipc_frame_id) : 0;
  MessageBuilder msg;
  auto framed = msg.initEvent(valid).initModelV2();
  framed.setFrameId(vipc_frame_id);
  framed.setFrameIdExtra(vipc_frame_id_extra);
  framed.setFrameAge(frame_age);
  framed.setFrameDropPerc(frame_drop * 100);
  framed.setTimestampEof(timestamp_eof);
  framed.setModelExecutionTime(model_execution_time);
  if (send_raw_pred) {
    framed.setRawPredictions(raw_pred.asBytes());
  }
  fill_model(framed, net_outputs);
  pm.send("modelV2", msg);
}

void posenet_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,
                     const ModelOutput &net_outputs, uint64_t timestamp_eof, const bool valid) {
  MessageBuilder msg;
  const auto &v_mean = net_outputs.pose.velocity_mean;
  const auto &r_mean = net_outputs.pose.rotation_mean;
  const auto &v_std = net_outputs.pose.velocity_std;
  const auto &r_std = net_outputs.pose.rotation_std;

  auto posenetd = msg.initEvent(valid && (vipc_dropped_frames < 1)).initCameraOdometry();
  posenetd.setTrans({v_mean.x, v_mean.y, v_mean.z});
  posenetd.setRot({r_mean.x, r_mean.y, r_mean.z});
  posenetd.setTransStd({exp(v_std.x), exp(v_std.y), exp(v_std.z)});
  posenetd.setRotStd({exp(r_std.x), exp(r_std.y), exp(r_std.z)});

  posenetd.setTimestampEof(timestamp_eof);
  posenetd.setFrameId(vipc_frame_id);

  pm.send("cameraOdometry", msg);
}
