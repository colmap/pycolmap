#!/bin/bash

# Installation based off of https://colmap.github.io/install.html (COLMAP)
# and https://github.com/mihaidusmanu/pycolmap#getting-started (pycolmap)
# and http://ceres-solver.org/installation.html (Ceres)
# However, the OS is centOS 7, instead of Ubuntu.

# Author: John Lambert (johnwlambert)

uname -a
echo "Current CentOS Version:"
cat /etc/centos-release

yum -y install wget

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
yum install -y libicu libicu-devel centos-release-scl-rh devtoolset-7-gcc-c++

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
yum install -y \
    git \
    cmake \
    gcc gcc-c++ make \
    freeimage-devel \
    metis-devel \
    glog-devel \
    gflags-devel \
    glew-devel

yum install -y suitesparse-devel atlas-devel lapack-devel blas-devel flann flann-devel lz4 lz4-devel

# Disable CGAL since it pulls many dependencies and increases the wheel size
#yum install -y yum-utils
#yum-config-manager --add-repo=http://springdale.princeton.edu/data/springdale/7/x86_64/os/Computational/
#yum install -y --nogpgcheck CGAL-devel

cd $CURRDIR
# Using Eigen 3.3, not Eigen 3.4
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xvzf eigen-3.3.9.tar.gz
export EIGEN_DIR="$CURRDIR/eigen-3.3.9"

# While Eigen is a header-only library, it still has to be built!
# Creates Eigen3Config.cmake from Eigen3Config.cmake.in
cd $EIGEN_DIR
mkdir build
cd build
cmake ..

ls -ltrh "$EIGEN_DIR/cmake/"

# ------ Install CERES solver ------
cd $CURRDIR
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF \
         -DBUILD_EXAMPLES=OFF \
         -DEigen3_DIR="$EIGEN_DIR/cmake/"
make -j$(nproc)
make install

echo "PYTHON_EXECUTABLE:${PYTHON_EXECUTABLE}"
echo "PYTHON_INCLUDE_DIR:${PYTHON_INCLUDE_DIR}"
echo "PYTHON_LIBRARY:${PYTHON_LIBRARY}"

# ------ Build FreeImage from source and install ------
#cd $CURRDIR
#wget http://downloads.sourceforge.net/freeimage/FreeImage3180.zip
#unzip FreeImage3180.zip
#cd FreeImage
#make
#make install

# ------ Build COLMAP ------
cd $CURRDIR
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev
mkdir build/
cd build/
CXXFLAGS="-fPIC" CFLAGS="-fPIC" cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBoost_USE_STATIC_LIBS=ON \
         -DBOOST_ROOT=/usr/local \
         -DGUI_ENABLED=OFF \
         -DEIGEN3_INCLUDE_DIRS=$EIGEN_DIR

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

PLAT=manylinux2014_x86_64
EIGEN3_INCLUDE_DIRS="$EIGEN_DIR" "${PYBIN}/python" setup.py bdist_wheel --plat-name=$PLAT

# Bundle external shared libraries into the wheels
mkdir -p /io/wheelhouse
for whl in ./dist/*.whl; do
    auditwheel repair "$whl" -w /io/wheelhouse/ --no-update-tags
done
ls -ltrh /io/wheelhouse/
