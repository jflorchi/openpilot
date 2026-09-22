#!/data/data/com.termux/files/usr/bin/bash
# Build pyopencl for termux python3.8 against the Adreno vendor OpenCL, into /data/c2port/pylib
# (system site-packages are read-only). Undo: rm -rf /data/c2port/pylib
set -e
cd /data/c2port
export PYTHONPATH=/data/c2port/pylib
mkdir -p pylib
mkdir -p cl/lib
[ -d cl/OpenCL-Headers ] || git clone -q --depth 1 --branch v2022.09.30 https://github.com/KhronosGroup/OpenCL-Headers.git cl/OpenCL-Headers
ln -sf /system/vendor/lib64/libOpenCL.so cl/lib/libOpenCL.so
[ -d pylib/pybind11 ] || python3 -m pip install -q --target /data/c2port/pylib "pybind11<2.11" "pytools<2023" "typing_extensions<4.8"
[ -d pylib/wheel ] || python3 -m pip install -q --target /data/c2port/pylib "wheel<0.42"
mkdir -p pyopencl-src && cd pyopencl-src
[ -f pyopencl-2022.2.4.tar.gz ] || curl -fsSL -o pyopencl-2022.2.4.tar.gz https://files.pythonhosted.org/packages/source/p/pyopencl/pyopencl-2022.2.4.tar.gz
[ -d pyopencl-2022.2.4 ] || tar xzf pyopencl-2022.2.4.tar.gz
cd pyopencl-2022.2.4
python3 configure.py --cl-inc-dir=/data/c2port/cl/OpenCL-Headers --cl-lib-dir=/data/c2port/cl/lib --cl-libname=OpenCL --cl-pretend-version=2.0
export CFLAGS="-O2" LDFLAGS="-L/data/c2port/cl/lib -Wl,-rpath,/system/vendor/lib64"
MAX_JOBS=4 python3 -m pip install -v --no-build-isolation --no-deps --target /data/c2port/pylib . 2>&1 | tail -20
cd /data/c2port
echo BUILD_FINISHED
python3 -c "import pyopencl as cl; p=cl.get_platforms(); d=p[0].get_devices()[0]; print('PYOPENCL_OK', cl.VERSION_TEXT, p[0].name, '|', d.name, '|', d.version, '| max wg', d.max_work_group_size, '| image support', d.image_support)"
