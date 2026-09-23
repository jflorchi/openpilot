# Current upstream supercombo on the comma two

Runs openpilot's current `driving_supercombo.onnx` (upstream master, Sep 2026; 30M params, 0.85 GMAC per frame) on the
comma two's Adreno 530. The model is rewritten into a graph the 2022-era tinygrad can compile, compiled to a thneed
on the device, and loaded by the thneed runtime in modeld (`C2PORT` in `models/driving.cc`).

SNPE isn't used: the device runtime is SNPE 1.x and Qualcomm only distributes 2.x converters, whose DLCs 1.x can't
load. Upstream made the same switch in `29cd51b` (tinygrad `2e9b7637` compiles ONNX straight to a thneed).

## Pipeline

On a computer with `onnx`/`onnxruntime` (source: upstream `openpilot/selfdrive/modeld/models/driving_supercombo.onnx`):

| step | script | what it does |
|---|---|---|
| 1 | `make_c2_onnx.py` | history buffers moved to the host (new inputs `imgs`, `desire_tokens`, `feat_hist`); fp16 -> fp32; opset 11; GELU / LayerNorm / L2-norm decomposed fp16-safely; masked `Where` -> add; grouped convs with a channel multiplier -> 1x1 duplicate + depthwise |
| 2 | `make_policy_2d.py` | the policy transformer (one block, only the last token used) restructured into 2D ops: attention for the last query only, no activation x activation matmuls |
| 3 | `strip_input_norm.py` | `(x - 127.5) / 63.75` removed from the graph; modeld's `loadyuv` applies it |
| 4 | `split_input_frames.py` | input fed channel-last, `imgs [1,128,256,32]` (per pixel 4 frames x 8 channels): the image backend's input conversion becomes a copy instead of a ~7 ms gather |
| 5 | `make_recipe.py` | graph + references to the original weights by name (~27 kB gz), so the device can rebuild the model from the public ONNX without `onnx`/`protobuf` |

Every step was checked against the original model in fp32: the rewrite matches to ~3e-7 relative.

On the device (`/data/c2port`, see `device/`):

| script | what it does |
|---|---|
| `fetch.sh` | downloads the original ONNX (sha256-checked) and tinygrad `2e9b7637`, applies `tinygrad_c2port.patch`, rebuilds the model from the recipe (`build_model.py`, with a stand-in `onnx` package in `onnx/`) |
| `install_pyopencl.sh` | builds pyopencl 2022.2.4 into `/data/c2port/pylib` (termux's `usr` and `home` are read-only) |
| `build_final.sh` | compiles `supercombo_c2port.thneed` on the GPU (`SAVE_SOURCE=1 OPT=99 OPENCL=1 FLOAT16=1 UNSAFE_FLOAT4=1 NATIVE_EXPLOG=1 OPTWG=1`) and checks it with `thneed_check.py` (a Python mirror of `thneed/serialize.cc`) on real frames |
| `full_cycle.sh` | all of the above plus installing into a build copy (`/data/c2port/op`), rebuilding modeld, and `live_test.sh` |
| `live_test.sh` | parked bench: camerad + calibrationd + the ported modeld on the live camera, `live_monitor.py` stats, `C2PORT_DUMP` dump checked by `check_dump4.py` |
| `kprof.py` | per-kernel GPU time of a thneed (OpenCL profiling events), written to a JSON with each kernel's source |
| `splitk.py` | rewrites the weight-bound 1x1 convs of a built thneed as split-K kernels (tuned per kernel on real frames, each checked against the original kernel's output); run it on `supercombo_c2port.thneed` after `build_final.sh` |

The ssh proxy only carries ~5 kB/s and drops long transfers, so the device downloads the large public inputs itself.

## Adreno 530 findings (tinygrad 2e9b7637)

All fixed in `tinygrad_c2port.patch` or in the graph rewrite:

- `write_imagef` into a `CL_HALF_FLOAT` image **rounds toward zero** (100% of values). It compounds to ~13% output error in fp16. Every image write now goes through `write_imageh(convert_half4_rte())`.
- Grouped convs with a channel multiplier are computed wrong by the image backend (rewritten in step 1).
- `OPT>=2` fused an elementwise op onto a conv through a pure reshape, giving an NCHW-shaped image output (crash); `get_getters` fell through to a float* getter for image inputs in another layout (compile error); `UNSAFE_FLOAT4` baked constants with `%f` (6 decimals).
- `MATMUL=1` gives wrong results; `GPU=1` (buffer kernels) is ~20x too slow; `clGetProgramInfo(BINARIES)` fails, so the thneed ships kernel source (`SAVE_SOURCE=1`) with `-D` options as `#define`s.
- Images backed by linear buffers are ~20% slower than native images; the loader creates native images for tinygrad-format files.
- Inputs written by `memcpy` into mapped buffers are replayed stale by the recorded kgsl commands; `Thneed::write_inputs` uses `clEnqueueWriteBuffer`.

## Accuracy and speed (device, fp16)

- Real camera frames: 1.2-2.0e-3 relative to fp32 (pure fp16 storage emulated on the same frames: ~1.7e-3).
- Model on the GPU: 33.3 ms per frame (was 40.2 before the channel-last input).
- Live bench (parked, own camera): 19.9 Hz, `modelExecutionTime` median 39.5 ms / p90 40.1 ms including frame
  preparation, frame drops 0.4%, C++ outputs within 2-5e-3 of an fp32 reference on the dumped inputs, history buffers
  consistent.

## Split-K (`splitk.py`)

Per-kernel profile of the built thneed: 32.2 ms of GPU time over 189 kernels, spread flat (top 10 = 26%), with no gaps
between kernels. The 1x1 convs with one output pixel (the policy / temporal matmuls, 92 kernels) were 37% of it and
streamed their 47 MB of weights at ~4 GB/s: tinygrad gives each thread one packed output channel and loops over every
input channel, so they ran as 1-16 workgroups. `splitk.py` lets S threads share an output channel and adds the partial
sums in local memory, keeping the kernel signature and fused epilogue, so only the program source and work sizes in the
thneed header change. 22 of 28 target programs replaced (the 9-row convs are faster as they were).

- GPU time 32.2 -> 25.3 ms; real frames vs fp32 2.03e-3 / 1.20e-3 -> 2.04e-3 / 1.21e-3.
- Live bench, back to back: `modelExecutionTime` median 34.5 -> 27.6 ms, 20.1 Hz, no frame drops, history check OK.

## Install / roll back

Install: copy `/data/c2port/supercombo_c2port.thneed` to `selfdrive/modeld/models/` and rebuild; `SConscript` defines
`C2PORT` when the file exists. Roll back: delete it and rebuild; modeld is then exactly the previous (Nuclear Grade) path.
