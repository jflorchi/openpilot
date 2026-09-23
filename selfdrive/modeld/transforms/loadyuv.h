#pragma once

#include "common/clutil.h"

typedef struct {
  int width, height;
  cl_kernel loadys_krnl, loaduv_krnl, copy_krnl, load_hwc8_krnl;
} LoadYUVState;

void loadyuv_init(LoadYUVState* s, cl_context ctx, cl_device_id device_id, int width, int height,
                  float scale = 1.0f, float offset = 0.0f);

void loadyuv_destroy(LoadYUVState* s);

void loadyuv_queue(LoadYUVState* s, cl_command_queue q,
                   cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                   cl_mem out_cl, bool do_shift = false);

// warped Y/U/V planes of one camera -> channel-last model input positions 2*cam (frame from ring slot slot_t4) and
// 2*cam + 1 (this frame), keeping this frame in ring slot slot_t (width/2 * height/2 uchar8 per slot). See loadyuv.cl.
void loadyuv_hwc8(LoadYUVState* s, cl_command_queue q, cl_mem y_cl, cl_mem u_cl, cl_mem v_cl,
                  cl_mem ring_cl, int slot_t, int slot_t4, cl_mem out_cl, int cam);
