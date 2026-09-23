#define UV_SIZE ((TRANSFORMED_WIDTH/2)*(TRANSFORMED_HEIGHT/2))

// optional affine normalization of the loaded pixel values (out = pixel * INPUT_SCALE + INPUT_OFFSET)
#ifndef INPUT_SCALE
#define INPUT_SCALE 1.0f
#endif
#ifndef INPUT_OFFSET
#define INPUT_OFFSET 0.0f
#endif

__kernel void loadys(__global uchar8 const * const Y,
                     __global float * out,
                     int out_offset)
{
    const int gid = get_global_id(0);
    const int ois = gid * 8;
    const int oy = ois / TRANSFORMED_WIDTH;
    const int ox = ois % TRANSFORMED_WIDTH;

    const uchar8 ys = Y[gid];
    const float8 ysf = convert_float8(ys) * INPUT_SCALE + INPUT_OFFSET;

    // 02
    // 13

    __global float* outy0;
    __global float* outy1;
    if ((oy & 1) == 0) {
      outy0 = out + out_offset; //y0
      outy1 = out + out_offset + UV_SIZE*2; //y2
    } else {
      outy0 = out + out_offset + UV_SIZE; //y1
      outy1 = out + out_offset + UV_SIZE*3; //y3
    }

    vstore4(ysf.s0246, 0, outy0 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
    vstore4(ysf.s1357, 0, outy1 + (oy/2) * (TRANSFORMED_WIDTH/2) + ox/2);
}

__kernel void loaduv(__global uchar8 const * const in,
                     __global float8 * out,
                     int out_offset)
{
  const int gid = get_global_id(0);
  const uchar8 inv = in[gid];
  const float8 outv  = convert_float8(inv) * INPUT_SCALE + INPUT_OFFSET;
  out[gid + out_offset / 8] = outv;
}

__kernel void copy(__global float8 * inout,
                   int in_offset)
{
  const int gid = get_global_id(0);
  inout[gid] = inout[gid + in_offset / 8];
}

// Channel-last model input straight from the warped Y/U/V planes, for one camera. Per pixel (UV_SIZE of them, one per 2x2
// Y block) 8 values: y0 y1 y2 y3 (the 2x2 block: top-left, bottom-left, top-right, bottom-right, as loadys) u v 0 0.
// The frame is kept as raw pixels in slot slot_t of a ring (for when it is t-4), and written normalized to input
// position 2*cam + 1; the frame from slot slot_t4 of the ring goes to position 2*cam. The input has 4 positions per
// pixel: narrow t-4, narrow t, wide t-4, wide t.
__kernel void load_hwc8(__global const uchar * Y,
                        __global const uchar * U,
                        __global const uchar * V,
                        __global uchar8 * ring,
                        int slot_t, int slot_t4,
                        __global float8 * out,
                        int cam)
{
  const int p = get_global_id(0);
  const int y = (p / (TRANSFORMED_WIDTH/2)) * 2, x = (p % (TRANSFORMED_WIDTH/2)) * 2;
  const uchar8 cur = (uchar8)(Y[y*TRANSFORMED_WIDTH + x], Y[(y+1)*TRANSFORMED_WIDTH + x],
                              Y[y*TRANSFORMED_WIDTH + x+1], Y[(y+1)*TRANSFORMED_WIDTH + x+1], U[p], V[p], 0, 0);
  const uchar8 prev = ring[slot_t4*UV_SIZE + p];
  ring[slot_t*UV_SIZE + p] = cur;

  float8 prevf = convert_float8(prev) * INPUT_SCALE + INPUT_OFFSET, curf = convert_float8(cur) * INPUT_SCALE + INPUT_OFFSET;
  prevf.s67 = curf.s67 = (float2)(0.0f, 0.0f);  // unused channels: exactly 0 (the model's weights for them are 0)
  out[4*p + 2*cam] = prevf;
  out[4*p + 2*cam + 1] = curf;
}
