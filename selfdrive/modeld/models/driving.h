#pragma once

// gate this here
#define TEMPORAL
#define DESIRE
#define TRAFFIC_CONVENTION

#include <array>
#include <memory>

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/visionipc_client.h"
#include "common/mat.h"
#include "common/modeldata.h"
#include "common/util.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"
#ifdef C2PORT
#include "selfdrive/modeld/thneed/thneed.h"
#endif

constexpr int DESIRE_LEN = 8;
constexpr int DESIRE_PRED_LEN = 4;
constexpr int TRAFFIC_CONVENTION_LEN = 2;
constexpr int MODEL_FREQ = 20;

constexpr int DISENGAGE_LEN = 5;
constexpr int BLINKER_LEN = 6;
constexpr int META_STRIDE = 7;

constexpr int PLAN_MHP_N = 5;
constexpr int STOP_LINE_MHP_N = 3;

constexpr int LEAD_MHP_N = 2;
constexpr int LEAD_TRAJ_LEN = 6;
constexpr int LEAD_PRED_DIM = 4;
constexpr int LEAD_MHP_SELECTION = 3;

struct ModelOutputXYZ {
  float x;
  float y;
  float z;
};
static_assert(sizeof(ModelOutputXYZ) == sizeof(float)*3);

struct ModelOutputYZ {
  float y;
  float z;
};
static_assert(sizeof(ModelOutputYZ) == sizeof(float)*2);

struct ModelOutputPlanElement {
  ModelOutputXYZ position;
  ModelOutputXYZ velocity;
  ModelOutputXYZ acceleration;
  ModelOutputXYZ rotation;
  ModelOutputXYZ rotation_rate;
};
static_assert(sizeof(ModelOutputPlanElement) == sizeof(ModelOutputXYZ)*5);

struct ModelOutputPlanPrediction {
  std::array<ModelOutputPlanElement, TRAJECTORY_SIZE> mean;
  std::array<ModelOutputPlanElement, TRAJECTORY_SIZE> std;
  float prob;
};
static_assert(sizeof(ModelOutputPlanPrediction) == (sizeof(ModelOutputPlanElement)*TRAJECTORY_SIZE*2) + sizeof(float));

struct ModelOutputPlans {
  std::array<ModelOutputPlanPrediction, PLAN_MHP_N> prediction;

  constexpr const ModelOutputPlanPrediction &get_best_prediction() const {
    int max_idx = 0;
    for (int i = 1; i < prediction.size(); i++) {
      if (prediction[i].prob > prediction[max_idx].prob) {
        max_idx = i;
      }
    }
    return prediction[max_idx];
  }
};
static_assert(sizeof(ModelOutputPlans) == sizeof(ModelOutputPlanPrediction)*PLAN_MHP_N);

struct ModelOutputLinesXY {
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> left_far;
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> left_near;
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> right_near;
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> right_far;
};
static_assert(sizeof(ModelOutputLinesXY) == sizeof(ModelOutputYZ)*TRAJECTORY_SIZE*4);

struct ModelOutputLineProbVal {
  float val_deprecated;
  float val;
};
static_assert(sizeof(ModelOutputLineProbVal) == sizeof(float)*2);

struct ModelOutputLinesProb {
  ModelOutputLineProbVal left_far;
  ModelOutputLineProbVal left_near;
  ModelOutputLineProbVal right_near;
  ModelOutputLineProbVal right_far;
};
static_assert(sizeof(ModelOutputLinesProb) == sizeof(ModelOutputLineProbVal)*4);

struct ModelOutputLaneLines {
  ModelOutputLinesXY mean;
  ModelOutputLinesXY std;
  ModelOutputLinesProb prob;
};
static_assert(sizeof(ModelOutputLaneLines) == (sizeof(ModelOutputLinesXY)*2) + sizeof(ModelOutputLinesProb));

struct ModelOutputEdgessXY {
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> left;
  std::array<ModelOutputYZ, TRAJECTORY_SIZE> right;
};
static_assert(sizeof(ModelOutputEdgessXY) == sizeof(ModelOutputYZ)*TRAJECTORY_SIZE*2);

struct ModelOutputRoadEdges {
  ModelOutputEdgessXY mean;
  ModelOutputEdgessXY std;
};
static_assert(sizeof(ModelOutputRoadEdges) == (sizeof(ModelOutputEdgessXY)*2));

struct ModelOutputLeadElement {
  float x;
  float y;
  float velocity;
  float acceleration;
};
static_assert(sizeof(ModelOutputLeadElement) == sizeof(float)*4);

struct ModelOutputLeadPrediction {
  std::array<ModelOutputLeadElement, LEAD_TRAJ_LEN> mean;
  std::array<ModelOutputLeadElement, LEAD_TRAJ_LEN> std;
  std::array<float, LEAD_MHP_SELECTION> prob;
};
static_assert(sizeof(ModelOutputLeadPrediction) == (sizeof(ModelOutputLeadElement)*LEAD_TRAJ_LEN*2) + (sizeof(float)*LEAD_MHP_SELECTION));

struct ModelOutputLeads {
  std::array<ModelOutputLeadPrediction, LEAD_MHP_N> prediction;
  std::array<float, LEAD_MHP_SELECTION> prob;

  constexpr const ModelOutputLeadPrediction &get_best_prediction(int t_idx) const {
    int max_idx = 0;
    for (int i = 1; i < prediction.size(); i++) {
      if (prediction[i].prob[t_idx] > prediction[max_idx].prob[t_idx]) {
        max_idx = i;
      }
    }
    return prediction[max_idx];
  }
};
static_assert(sizeof(ModelOutputLeads) == (sizeof(ModelOutputLeadPrediction)*LEAD_MHP_N) + (sizeof(float)*LEAD_MHP_SELECTION));

struct ModelOutputStopLineElement {
  ModelOutputXYZ position;
  ModelOutputXYZ rotation;
  float speed;
  float time;
};
static_assert(sizeof(ModelOutputStopLineElement) == (sizeof(ModelOutputXYZ)*2 + sizeof(float)*2));

struct ModelOutputStopLinePrediction {
  ModelOutputStopLineElement mean;
  ModelOutputStopLineElement std;
  float prob;
};
static_assert(sizeof(ModelOutputStopLinePrediction) == (sizeof(ModelOutputStopLineElement)*2 + sizeof(float)));

struct ModelOutputStopLines {
  std::array<ModelOutputStopLinePrediction, STOP_LINE_MHP_N> prediction;
  float prob;

  constexpr const ModelOutputStopLinePrediction &get_best_prediction(int t_idx) const {
    int max_idx = 0;
    for (int i = 1; i < prediction.size(); i++) {
      if (prediction[i].prob > prediction[max_idx].prob) {
        max_idx = i;
      }
    }
    return prediction[max_idx];
  }
};
static_assert(sizeof(ModelOutputStopLines) == (sizeof(ModelOutputStopLinePrediction)*STOP_LINE_MHP_N) + sizeof(float));

struct ModelOutputPose {
  ModelOutputXYZ velocity_mean;
  ModelOutputXYZ rotation_mean;
  ModelOutputXYZ velocity_std;
  ModelOutputXYZ rotation_std;
};
static_assert(sizeof(ModelOutputPose) == sizeof(ModelOutputXYZ)*4);

struct ModelOutputDisengageProb {
  float gas_disengage;
  float brake_disengage;
  float steer_override;
  float brake_3ms2;
  float brake_4ms2;
  float brake_5ms2;
  float gas_pressed;
};
static_assert(sizeof(ModelOutputDisengageProb) == sizeof(float)*7);

struct ModelOutputBlinkerProb {
  float left;
  float right;
};
static_assert(sizeof(ModelOutputBlinkerProb) == sizeof(float)*2);

struct ModelOutputDesireProb {
  union {
    struct {
      float none;
      float turn_left;
      float turn_right;
      float lane_change_left;
      float lane_change_right;
      float keep_left;
      float keep_right;
      float null;
    };
    struct {
      std::array<float, DESIRE_LEN> array;
    };
  };
};
static_assert(sizeof(ModelOutputDesireProb) == sizeof(float)*DESIRE_LEN);

struct ModelOutputMeta {
  ModelOutputDesireProb desire_state_prob;
  float engaged_prob;
  std::array<ModelOutputDisengageProb, DISENGAGE_LEN> disengage_prob;
  std::array<ModelOutputBlinkerProb, BLINKER_LEN> blinker_prob;
  std::array<ModelOutputDesireProb, DESIRE_PRED_LEN> desire_pred_prob;
};
static_assert(sizeof(ModelOutputMeta) == sizeof(ModelOutputDesireProb) + sizeof(float) + (sizeof(ModelOutputDisengageProb)*DISENGAGE_LEN) + (sizeof(ModelOutputBlinkerProb)*BLINKER_LEN) + (sizeof(ModelOutputDesireProb)*DESIRE_PRED_LEN));

struct ModelOutput {
  const ModelOutputPlans plans;
  const ModelOutputLaneLines lane_lines;
  const ModelOutputRoadEdges road_edges;
  const ModelOutputLeads leads;
  const ModelOutputStopLines stop_lines;
  const ModelOutputMeta meta;
  const ModelOutputPose pose;
};

constexpr int OUTPUT_SIZE = sizeof(ModelOutput) / sizeof(float);
#ifdef TEMPORAL
  constexpr int TEMPORAL_SIZE = 512;
#else
  constexpr int TEMPORAL_SIZE = 0;
#endif
constexpr int NET_OUTPUT_SIZE = OUTPUT_SIZE + TEMPORAL_SIZE;

#ifdef C2PORT
// Current upstream supercombo ported to the comma two (see c2port/README.md). The model has no recurrent state;
// modeld keeps its history buffers. Output layout (floats), from the model's output_slices metadata:
namespace c2port {
  constexpr int OUTPUT_SIZE = 2576;
  constexpr int META = 0, DESIRE_PRED = 55, POSE = 87, LANE_LINES = 117, LANE_LINES_PROB = 645, ROAD_EDGES = 653,
                LEAD = 917, LEAD_PROB = 1061, HIDDEN = 1064, PLAN = 1576, DESIRE_STATE = 2566;
  constexpr int PIXELS = 128 * 256;
  constexpr int FRAME_SIZE = PIXELS * 8;         // one camera frame, channel-last: 6 YUV channels + 2 unused per pixel
  // model input 'imgs' [1,128,256,32]: per pixel narrow t-4, narrow t, wide t-4, wide t (8 floats each)
  constexpr int RING = 5;                        // frames t-4 .. t
  constexpr int DESIRE_HIST = 100, FEAT_HIST = 96, FEAT_LEN = 512, FEAT_TOKENS = 8;
  // input normalization, applied by loadyuv: (pixel - 127.5) / 63.75
  constexpr float INPUT_SCALE = 1.0f / 63.75f, INPUT_OFFSET = -127.5f / 63.75f;

  // new lead layout: 3 time offsets x 6 steps x (x, y, v, a), means then log-stds; stored after the old ModelOutput
  struct Leads {
    std::array<float, LEAD_MHP_SELECTION * LEAD_TRAJ_LEN * LEAD_PRED_DIM> mean;
    std::array<float, LEAD_MHP_SELECTION * LEAD_TRAJ_LEN * LEAD_PRED_DIM> std;
    std::array<float, LEAD_MHP_SELECTION> prob;
  };
}
#endif

// TODO: convert remaining arrays to std::array and update model runners
struct ModelState {
  ModelFrame *frame;
  ModelFrame *wide_frame;
  std::array<float, NET_OUTPUT_SIZE> output = {};
  std::unique_ptr<RunModel> m;
#ifdef DESIRE
  float prev_desire[DESIRE_LEN] = {};
  float pulse_desire[DESIRE_LEN] = {};
#endif
#ifdef TRAFFIC_CONVENTION
  float traffic_convention[TRAFFIC_CONVENTION_LEN] = {};
#endif
#ifdef C2PORT
  std::unique_ptr<Thneed> thneed;
  bool recorded = false;
  int in_imgs, in_desire, in_feat, in_traffic;              // thneed input indices
  std::array<float, c2port::OUTPUT_SIZE> raw = {};         // model output
  cl_mem ring[2] = {};                                     // narrow / wide raw pixels on the GPU: RING frames each
  std::unique_ptr<float[]> imgs;                           // host copy of the model input 'imgs', only for C2PORT_DUMP
  int ring_head = 0;
  std::array<float, c2port::DESIRE_HIST * DESIRE_LEN> desire_hist = {};
  std::array<float, c2port::FEAT_HIST * c2port::FEAT_LEN> feat_hist = {};
  std::array<float, 25 * DESIRE_LEN> desire_tokens = {};
  std::array<float, c2port::FEAT_TOKENS * c2port::FEAT_LEN> feat_tokens = {};
#endif
};
#ifdef C2PORT
static_assert(sizeof(c2port::Leads) <= TEMPORAL_SIZE * sizeof(float), "c2port leads live in the unused recurrent tail");
#endif

void model_init(ModelState* s, cl_device_id device_id, cl_context context);
ModelOutput *model_eval_frame(ModelState* s, VisionBuf* buf, VisionBuf* buf_wide,
                              const mat3 &transform, const mat3 &transform_wide, float *desire_in, bool prepare_only);
void model_free(ModelState* s);
void model_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_frame_id_extra, uint32_t frame_id, float frame_drop,
                   const ModelOutput &net_outputs, uint64_t timestamp_eof,
                   float model_execution_time, kj::ArrayPtr<const float> raw_pred, const bool valid);
void posenet_publish(PubMaster &pm, uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,
                     const ModelOutput &net_outputs, uint64_t timestamp_eof, const bool valid);
