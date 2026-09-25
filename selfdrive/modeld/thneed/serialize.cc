#include <cassert>
#include <cstdio>
#include <set>

#include "json11.hpp"
#include "common/util.h"
#include "common/clutil.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "selfdrive/modeld/thneed/thneed.h"
using namespace json11;

extern map<cl_program, string> g_program_source;

// Source-only thneeds (tinygrad) would otherwise compile every program on each start (~30 s for the C2PORT model).
// Compiled programs are cached here, keyed by source and driver version. A missing or bad entry falls back to source.
#define PROGRAM_CACHE_DIR "/data/c2port/clcache"

static uint64_t fnv1a(const string &s, uint64_t h = 1469598103934665603ULL) {
  for (unsigned char c : s) { h ^= c; h *= 1099511628211ULL; }
  return h;
}

static cl_program program_from_cache_or_source(cl_context context, cl_device_id device_id, const string &source, int *cache_hits) {
  char driver[256] = {0};
  clGetDeviceInfo(device_id, CL_DRIVER_VERSION, sizeof(driver) - 1, driver, NULL);
  char path[512];
  snprintf(path, sizeof(path), PROGRAM_CACHE_DIR "/%016llx.bin", (unsigned long long)fnv1a(source, fnv1a(driver)));

  string binary = util::file_exists(path) ? util::read_file(path) : "";
  if (!binary.empty()) {
    int err;
    size_t length = binary.size();
    const uint8_t *bin = (const uint8_t *)binary.data();
    cl_program prg = clCreateProgramWithBinary(context, 1, &device_id, &length, &bin, NULL, &err);
    if (err == CL_SUCCESS && clBuildProgram(prg, 1, &device_id, NULL, NULL, NULL) == CL_SUCCESS) {
      (*cache_hits)++;
      return prg;
    }
    LOGW("thneed: cached program %s unusable (err %d), building from source", path, err);
    if (err == CL_SUCCESS) clReleaseProgram(prg);
  }

  cl_program prg = cl_program_from_source(context, device_id, source);
  size_t size = 0;
  if (clGetProgramInfo(prg, CL_PROGRAM_BINARY_SIZES, sizeof(size), &size, NULL) == CL_SUCCESS && size > 0) {
    string out(size, '\0');
    uint8_t *bufs[1] = {(uint8_t *)out.data()};
    if (clGetProgramInfo(prg, CL_PROGRAM_BINARIES, sizeof(bufs), bufs, NULL) == CL_SUCCESS) {
      util::create_directories(PROGRAM_CACHE_DIR, 0775);
      string tmp = string(path) + ".tmp";
      if (util::write_file(tmp.c_str(), out.data(), out.size(), O_WRONLY | O_CREAT | O_TRUNC) == 0) {
        rename(tmp.c_str(), path);
      }
    } else {
      LOGW("thneed: clGetProgramInfo(CL_PROGRAM_BINARIES) failed, not caching");
    }
  }
  return prg;
}

void Thneed::load(const char *filename) {
  printf("Thneed::load: loading from %s\n", filename);
  double t_start = millis_since_boot();

  string buf = util::read_file(filename);
  int jsz = *(int *)buf.data();
  string err;
  string jj(buf.data() + sizeof(int), jsz);
  Json jdat = Json::parse(jj, err);

  map<cl_mem, cl_mem> real_mem;
  real_mem[NULL] = NULL;

  // tinygrad-compiled thneeds list their inputs/outputs by name and use standalone images. SNPE-recorded thneeds don't,
  // and may alias images with buffers, so they keep the buffer-backed path below.
  const bool named_io = !jdat["inputs"].array_items().empty();

  int ptr = sizeof(int)+jsz;
  for (auto &obj : jdat["objects"].array_items()) {
    auto mobj = obj.object_items();
    int sz = mobj["size"].int_value();
    cl_mem clbuf = NULL;

    if (named_io && mobj["arg_type"] == "image2d_t" && mobj["buffer_id"].string_value().empty()) {
      // native (tiled) image: ~20% faster on the Adreno 530 than an image over a linear buffer
      cl_image_format format = {CL_RGBA, CL_HALF_FLOAT};
      cl_image_desc desc = {0};
      desc.image_type = CL_MEM_OBJECT_IMAGE2D;
      desc.image_width = mobj["width"].int_value();
      desc.image_height = mobj["height"].int_value();
      clbuf = clCreateImage(context, CL_MEM_READ_WRITE, &format, &desc, NULL, NULL);
      assert(clbuf != NULL);
      if (mobj["needs_load"].bool_value()) {
        size_t origin[3] = {0, 0, 0};
        size_t region[3] = {desc.image_width, desc.image_height, 1};
        size_t row_pitch = mobj["row_pitch"].int_value();
        CL_CHECK(clEnqueueWriteImage(command_queue, clbuf, CL_TRUE, origin, region, row_pitch, 0, &buf[ptr], 0, NULL, NULL));
        ptr += sz;
      }
      real_mem[*(cl_mem*)(mobj["id"].string_value().data())] = clbuf;
      continue;
    }

    if (mobj["buffer_id"].string_value().size() > 0) {
      // image buffer must already be allocated
      clbuf = real_mem[*(cl_mem*)(mobj["buffer_id"].string_value().data())];
      assert(mobj["needs_load"].bool_value() == false);
    } else {
      if (mobj["needs_load"].bool_value()) {
        //printf("loading %p %d @ 0x%X\n", clbuf, sz, ptr);
        clbuf = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE, sz, &buf[ptr], NULL);
        ptr += sz;
      } else if (named_io) {
        // zero-initialized, as tinygrad assumes
        vector<char> zeros(sz, 0);
        clbuf = clCreateBuffer(context, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE, sz, zeros.data(), NULL);
      } else {
        clbuf = clCreateBuffer(context, CL_MEM_READ_WRITE, sz, NULL, NULL);
      }
    }
    assert(clbuf != NULL);

    if (mobj["arg_type"] == "image2d_t" || mobj["arg_type"] == "image1d_t") {
      cl_image_desc desc = {0};
      desc.image_type = (mobj["arg_type"] == "image2d_t") ? CL_MEM_OBJECT_IMAGE2D : CL_MEM_OBJECT_IMAGE1D_BUFFER;
      desc.image_width = mobj["width"].int_value();
      desc.image_height = mobj["height"].int_value();
      desc.image_row_pitch = mobj["row_pitch"].int_value();
      desc.buffer = clbuf;

      cl_image_format format;
      format.image_channel_order = CL_RGBA;
      format.image_channel_data_type = CL_HALF_FLOAT;

      clbuf = clCreateImage(context, CL_MEM_READ_WRITE, &format, &desc, NULL, NULL);
      assert(clbuf != NULL);
    }

    real_mem[*(cl_mem*)(mobj["id"].string_value().data())] = clbuf;
  }

  double t_objects = millis_since_boot();
  map<string, cl_program> g_programs;
  int cache_hits = 0;
  for (const auto &[name, source] : jdat["programs"].object_items()) {
    if (debug >= 1) printf("building %s with size %zu\n", name.c_str(), source.string_value().size());
    g_programs[name] = named_io ? program_from_cache_or_source(context, device_id, source.string_value(), &cache_hits)
                                : cl_program_from_source(context, device_id, source.string_value());
  }
  double t_programs = millis_since_boot();
  LOGW("thneed load: objects %.0f ms, %zu programs %.0f ms (%d cached)", t_objects - t_start,
       jdat["programs"].object_items().size(), t_programs - t_objects, cache_hits);

  for (auto &obj : jdat["binaries"].array_items()) {
    string name = obj["name"].string_value();
    size_t length = obj["length"].int_value();
    if (debug >= 1) printf("binary %s with size %zu\n", name.c_str(), length);
    g_programs[name] = cl_program_from_binary(context, device_id, (const uint8_t*)&buf[ptr], length);
    ptr += length;
  }

  for (auto &obj : jdat["kernels"].array_items()) {
    auto gws = obj["global_work_size"];
    auto lws = obj["local_work_size"];
    auto kk = shared_ptr<CLQueuedKernel>(new CLQueuedKernel(this));

    kk->name = obj["name"].string_value();
    kk->program = g_programs[kk->name];
    kk->work_dim = obj["work_dim"].int_value();
    for (int i = 0; i < kk->work_dim; i++) {
      kk->global_work_size[i] = gws[i].int_value();
      kk->local_work_size[i] = lws[i].int_value();
    }
    kk->num_args = obj["num_args"].int_value();
    for (int i = 0; i < kk->num_args; i++) {
      string arg = obj["args"].array_items()[i].string_value();
      int arg_size = obj["args_size"].array_items()[i].int_value();
      kk->args_size.push_back(arg_size);
      if (arg_size == 8) {
        cl_mem val = *(cl_mem*)(arg.data());
        val = real_mem[val];
        kk->args.push_back(string((char*)&val, sizeof(val)));
      } else {
        kk->args.push_back(arg);
      }
    }
    kq.push_back(kk);
  }

  if (named_io) {
    for (auto &obj : jdat["inputs"].array_items()) {
      cl_mem mem = real_mem[*(cl_mem*)(obj["buffer_id"].string_value().data())];
      size_t sz = obj["size"].int_value();
      // not mapped: named inputs are written with write_inputs (clEnqueueWriteBuffer), see thneed.cc
      input_clmem.push_back(mem);
      input_sizes.push_back(sz);
      inputs.push_back(nullptr);
      input_names.push_back(obj["name"].string_value());
      if (debug >= 1) printf("input %s: %zu bytes\n", input_names.back().c_str(), sz);
    }
    output = real_mem[*(cl_mem*)(jdat["outputs"].array_items()[0]["buffer_id"].string_value().data())];
  }

  clFinish(command_queue);
}

int Thneed::input_index(const string &name) const {
  for (int i = 0; i < input_names.size(); i++) {
    if (input_names[i] == name) return i;
  }
  return -1;
}

void Thneed::save(const char *filename, bool save_binaries) {
  printf("Thneed::save: saving to %s\n", filename);

  // get kernels
  std::vector<Json> kernels;
  std::set<string> saved_objects;
  std::vector<Json> objects;
  std::map<string, string> programs;
  std::map<string, string> binaries;

  for (auto &k : kq) {
    kernels.push_back(k->to_json());

    // check args for objects
    int i = 0;
    for (auto &a : k->args) {
      if (a.size() == 8) {
        if (saved_objects.find(a) == saved_objects.end()) {
          saved_objects.insert(a);
          cl_mem val = *(cl_mem*)(a.data());
          if (val != NULL) {
            bool needs_load = k->arg_names[i] == "weights" || k->arg_names[i] == "biases";

            auto jj = Json::object({
              {"id", a},
              {"arg_type", k->arg_types[i]},
            });

            if (k->arg_types[i] == "image2d_t" || k->arg_types[i] == "image1d_t") {
              cl_mem buf;
              clGetImageInfo(val, CL_IMAGE_BUFFER, sizeof(buf), &buf, NULL);
              string aa = string((char *)&buf, sizeof(buf));
              jj["buffer_id"] = aa;

              size_t width, height, row_pitch;
              clGetImageInfo(val, CL_IMAGE_WIDTH, sizeof(width), &width, NULL);
              clGetImageInfo(val, CL_IMAGE_HEIGHT, sizeof(height), &height, NULL);
              clGetImageInfo(val, CL_IMAGE_ROW_PITCH, sizeof(row_pitch), &row_pitch, NULL);
              jj["width"] = (int)width;
              jj["height"] = (int)height;
              jj["row_pitch"] = (int)row_pitch;
              jj["size"] = (int)(height * row_pitch);
              jj["needs_load"] = false;

              if (saved_objects.find(aa) == saved_objects.end()) {
                saved_objects.insert(aa);
                size_t sz;
                clGetMemObjectInfo(buf, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
                // save the buffer
                objects.push_back(Json::object({
                  {"id", aa},
                  {"arg_type", "<image buffer>"},
                  {"needs_load", needs_load},
                  {"size", (int)sz}
                }));
                if (needs_load) assert(sz == height * row_pitch);
              }
            } else {
              size_t sz = 0;
              clGetMemObjectInfo(val, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
              jj["size"] = (int)sz;
              jj["needs_load"] = needs_load;
            }

            objects.push_back(jj);
          }
        }
      }
      i++;
    }

    if (save_binaries) {
      int err;
      size_t binary_size = 0;
      err = clGetProgramInfo(k->program, CL_PROGRAM_BINARY_SIZES, sizeof(binary_size), &binary_size, NULL);
      assert(err == 0);
      assert(binary_size > 0);
      string sv(binary_size, '\x00');

      uint8_t* bufs[1] = { (uint8_t*)sv.data(), };
      err = clGetProgramInfo(k->program, CL_PROGRAM_BINARIES, sizeof(bufs), &bufs, NULL);
      assert(err == 0);

      binaries[k->name] = sv;
    } else {
      programs[k->name] = g_program_source[k->program];
    }
  }

  vector<string> saved_buffers;
  for (auto &obj : objects) {
    auto mobj = obj.object_items();
    cl_mem val = *(cl_mem*)(mobj["id"].string_value().data());
    int sz = mobj["size"].int_value();
    if (mobj["needs_load"].bool_value()) {
      char *buf = (char *)malloc(sz);
      if (mobj["arg_type"] == "image2d_t" || mobj["arg_type"] == "image1d_t") {
        assert(false);
      } else {
        // buffers allocated with CL_MEM_HOST_WRITE_ONLY, hence this hack
        //hexdump((uint32_t*)val, 0x100);

        // the worst hack in thneed, the flags are at 0x14
        ((uint32_t*)val)[0x14] &= ~CL_MEM_HOST_WRITE_ONLY;
        cl_int ret = clEnqueueReadBuffer(command_queue, val, CL_TRUE, 0, sz, buf, 0, NULL, NULL);
        assert(ret == CL_SUCCESS);
      }
      //printf("saving buffer: %d %p %s\n", sz, buf, mobj["arg_type"].string_value().c_str());
      saved_buffers.push_back(string(buf, sz));
      free(buf);
    }
  }

  std::vector<Json> jbinaries;
  for (auto &obj : binaries) {
    jbinaries.push_back(Json::object({{"name", obj.first}, {"length", (int)obj.second.size()}}));
    saved_buffers.push_back(obj.second);
  }

  Json jdat = Json::object({
    {"kernels", kernels},
    {"objects", objects},
    {"programs", programs},
    {"binaries", jbinaries},
  });

  string str = jdat.dump();
  int jsz = str.length();

  FILE *f = fopen(filename, "wb");
  fwrite(&jsz, 1, sizeof(jsz), f);
  fwrite(str.data(), 1, jsz, f);
  for (auto &s : saved_buffers) {
    fwrite(s.data(), 1, s.length(), f);
  }
  fclose(f);
}

Json CLQueuedKernel::to_json() const {
  return Json::object {
    { "name", name },
    { "work_dim", (int)work_dim },
    { "global_work_size", Json::array { (int)global_work_size[0], (int)global_work_size[1], (int)global_work_size[2] } },
    { "local_work_size", Json::array { (int)local_work_size[0], (int)local_work_size[1], (int)local_work_size[2] } },
    { "num_args", (int)num_args },
    { "args", args },
    { "args_size", args_size },
  };
}

