#pragma once

#include "common/clutil.h"

typedef struct {
  int width, height;
  cl_kernel loadys_krnl, loaduv_krnl, copy_krnl, to_hwc8_krnl, gather_hwc8_krnl;
} LoadYUVState;

void loadyuv_init(LoadYUVState* s, cl_context ctx, cl_device_id device_id, int width, int height,
                  float scale = 1.0f, float offset = 0.0f);

void loadyuv_destroy(LoadYUVState* s);

void loadyuv_queue(LoadYUVState* s, cl_command_queue q,
                   cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                   cl_mem out_cl, bool do_shift = false);

// planar 6-channel frame (as written by loadyuv_queue) -> width/2 * height/2 pixels x 8 floats, written at frame slot
// out_slot of hwc8_cl
void loadyuv_to_hwc8(LoadYUVState* s, cl_command_queue q, cl_mem planar_cl, cl_mem hwc8_cl, int out_slot = 0);

// 4 channel-last frames (slots of rings a and b) -> per pixel 32 floats: a[a0], a[a1], b[b0], b[b1]
void loadyuv_gather_hwc8(LoadYUVState* s, cl_command_queue q, cl_mem a, cl_mem b, int a0, int a1, int b0, int b1,
                         cl_mem out_cl);
