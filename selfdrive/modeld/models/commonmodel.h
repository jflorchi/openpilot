#pragma once

#include <cfloat>
#include <cstdlib>

#include <memory>

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "common/mat.h"
#include "selfdrive/modeld/transforms/loadyuv.h"
#include "selfdrive/modeld/transforms/transform.h"

const bool send_raw_pred = getenv("SEND_RAW_PRED") != NULL;

void softmax(const float* input, float* output, size_t len);
float sigmoid(float input);

class ModelFrame {
public:
  ModelFrame(cl_device_id device_id, cl_context context, float input_scale = 1.0f, float input_offset = 0.0f);
  ~ModelFrame();
  float* prepare(cl_mem yuv_cl, int width, int height, const mat3& transform, cl_mem *output);
  // warp + load a single frame into frame slot `slot` of ring_cl, channel-last: 128x256 pixels x 8 floats
  // (6 channels + 2 zeros). Stays on the GPU.
  void prepare_hwc8(cl_mem yuv_cl, int width, int height, const mat3& transform, cl_mem ring_cl, int slot);
  // interleave 4 channel-last frames (slots of rings a and b) into out: per pixel a[a0], a[a1], b[b0], b[b1]
  void gather_hwc8(cl_mem a, cl_mem b, int a0, int a1, int b0, int b1, cl_mem out);

  const int MODEL_WIDTH = 512;
  const int MODEL_HEIGHT = 256;
  const int MODEL_FRAME_SIZE = MODEL_WIDTH * MODEL_HEIGHT * 3 / 2;
  const int buf_size = MODEL_FRAME_SIZE * 2;
  const int HWC8_FRAME_SIZE = (MODEL_WIDTH / 2) * (MODEL_HEIGHT / 2) * 8;  // after MODEL_WIDTH/HEIGHT: members init in order

private:
  Transform transform;
  LoadYUVState loadyuv;
  cl_command_queue q;
  cl_mem y_cl, u_cl, v_cl, net_input_cl;
  std::unique_ptr<float[]> input_frames;
};
