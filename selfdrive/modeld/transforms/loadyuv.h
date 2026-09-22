#pragma once

#include "common/clutil.h"

typedef struct {
  int width, height;
  cl_kernel loadys_krnl, loaduv_krnl, copy_krnl, to_hwc8_krnl;
} LoadYUVState;

void loadyuv_init(LoadYUVState* s, cl_context ctx, cl_device_id device_id, int width, int height,
                  float scale = 1.0f, float offset = 0.0f);

void loadyuv_destroy(LoadYUVState* s);

void loadyuv_queue(LoadYUVState* s, cl_command_queue q,
                   cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                   cl_mem out_cl, bool do_shift = false);

// planar 6-channel frame (as written by loadyuv_queue) -> width/2 * height/2 pixels x 8 floats
void loadyuv_to_hwc8(LoadYUVState* s, cl_command_queue q, cl_mem planar_cl, cl_mem hwc8_cl);
