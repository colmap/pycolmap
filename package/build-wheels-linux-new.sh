#!/bin/bash

# Installation based off of https://colmap.github.io/install.html (COLMAP)
# and https://github.com/mihaidusmanu/pycolmap#getting-started (pycolmap)
# and http://ceres-solver.org/installation.html (Ceres)
# However, the OS is centOS 7, instead of Ubuntu.

# Author: John Lambert (johnwlambert)

uname -a
echo "Current Debian Version:"
cat /etc/debian-release

apt-get -y install wget

ls -ltrh /io/

# we cannot simply use `pip` or `python`, since points to old 2.7 version
PYBIN="/opt/python/$PYTHON_VERSION/bin"
PYVER_NUM=$($PYBIN/python -c "import sys;print(sys.version.split(\" \")[0])")
PYTHONVER="$(basename $(dirname $PYBIN))"

echo "Python bin path: $PYBIN"
echo "Python version number: $PYVER_NUM"
echo "Python version: $PYTHONVER"

export PATH=$PYBIN:$PATH

${PYBIN}/pip install auditwheel

PYTHON_EXECUTABLE=${PYBIN}/python
# We use distutils to get the include directory and the library path directly from the selected interpreter
# We provide these variables to CMake to hint what Python development files we wish to use in the build.
PYTHON_INCLUDE_DIR=$(${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")
PYTHON_LIBRARY=$(${PYTHON_EXECUTABLE} -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))")

CURRDIR=$(pwd)

echo "Num. processes to use for building: ${nproc}"

# ------ Install boost (build it staticly) ------
cd $CURRDIR
apt-get install -y libicu libicu-devel centos-release-scl-rh devtoolset-7-gcc-c++

# Download and install Boost-1.65.1
# colmap needs only program_options filesystem graph system unit_test_framework
mkdir -p boost && \
    cd boost && \
    wget -nv https://boostorg.jfrog.io/artifactory/main/release/1.65.1/source/boost_1_65_1.tar.gz && \
    tar xzf boost_1_65_1.tar.gz && \
    cd boost_1_65_1 && \
    ./bootstrap.sh --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex,graph,test && \
    ./b2 -j$(nproc) cxxflags="-fPIC" runtime-link=static variant=release link=static install

# Boost should now be visible under /usr/local
ls -ltrh /usr/local

# ------ Install dependencies from the default repositories ------
cd $CURRDIR
sudo apt-get update && sudo apt-get install -y \
        build-essential \
        ninja-build \
        libboost-program-options-dev \
        libboost-filesystem-dev \
        libboost-graph-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libceres-dev \
        libflann-dev \
        libfreeimage-dev \
        libmetis-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libsqlite3-dev \
        libglew-dev \
        qtbase5-dev \
        libqt5opengl5-dev \
        libcgal-dev \
        libcgal-qt5-dev \
        libgl1-mesa-dri \
        libunwind-dev \
        xvfb


# Disable CGAL since it pulls many dependencies and increases the wheel size
#yum install -y yum-utils
#yum-config-manager --add-repo=http://springdale.princeton.edu/data/springdale/7/x86_64/os/Computational/
#yum install -y --nogpgcheck CGAL-devel

cd $CURRDIR

echo "PYTHON_EXECUTABLE:${PYTHON_EXECUTABLE}"
echo "PYTHON_INCLUDE_DIR:${PYTHON_INCLUDE_DIR}"
echo "PYTHON_LIBRARY:${PYTHON_LIBRARY}"

# ------ Build COLMAP ------
cd $CURRDIR
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build/
cd build/
CXXFLAGS="-fPIC" CFLAGS="-fPIC" cmake .. \
        -GNinja \
        -DTESTS_ENABLED=ON \

if [ $ec -ne 0 ]; then
    echo "Error:"
    cat ./CMakeCache.txt
    exit $ec
fi
set -e -x
make -j$(nproc) install

# ------ Build pycolmap wheel ------
cd /io/
cat setup.py
PLAT=manylinux_2_24_x86_64
EIGEN3_INCLUDE_DIRS="$EIGEN_DIR" "${PYBIN}/python" setup.py bdist_wheel --plat-name=$PLAT

# Bundle external shared libraries into the wheels
mkdir -p /io/wheelhouse
for whl in ./dist/*.whl; do
    auditwheel repair "$whl" -w /io/wheelhouse/ --no-update-tags
done
ls -ltrh /io/wheelhouse/