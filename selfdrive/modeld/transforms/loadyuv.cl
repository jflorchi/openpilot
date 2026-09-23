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

// planar (y0 y1 y2 y3 u v, UV_SIZE each) -> per pixel 8 floats (the 6 channels + 2 zeros), for channel-last models.
// out_offset in pixels (float8s), e.g. a slot of a frame ring
__kernel void to_hwc8(__global const float * in,
                      __global float8 * out,
                      int out_offset)
{
  const int p = get_global_id(0);
  out[out_offset + p] = (float8)(in[p], in[UV_SIZE + p], in[2*UV_SIZE + p], in[3*UV_SIZE + p], in[4*UV_SIZE + p], in[5*UV_SIZE + p], 0.0f, 0.0f);
}

// interleave 4 channel-last frames (slots of two frame rings, UV_SIZE pixels each) into one [UV_SIZE, 4 x 8] input:
// per pixel a[slot a0], a[slot a1], b[slot b0], b[slot b1]
__kernel void gather_hwc8(__global const float8 * a,
                          __global const float8 * b,
                          int a0, int a1, int b0, int b1,
                          __global float8 * out)
{
  const int p = get_global_id(0);
  out[4*p + 0] = a[a0*UV_SIZE + p];
  out[4*p + 1] = a[a1*UV_SIZE + p];
  out[4*p + 2] = b[b0*UV_SIZE + p];
  out[4*p + 3] = b[b1*UV_SIZE + p];
}
