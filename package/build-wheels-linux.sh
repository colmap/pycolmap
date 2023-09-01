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

# ------ Install dependencies from the default repositories ------
cd $CURRDIR
yum install -y \
    git \
    gcc gcc-c++ make \
    freeimage-devel \
    metis-devel \
    glew-devel
cmake --version

yum install -y suitesparse-devel atlas-devel lapack-devel blas-devel flann flann-devel lz4 lz4-devel

# ------ Install boost ------
cd $CURRDIR
#yum install -y centos-release-scl-rh devtoolset-7-gcc-c++
mkdir -p boost && cd boost
export BOOST_FILENAME=boost_1_71_0
wget -nv https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/${BOOST_FILENAME}.tar.gz
tar xzf ${BOOST_FILENAME}.tar.gz
cd ${BOOST_FILENAME}
./bootstrap.sh --with-libraries=filesystem,system,program_options,graph,test --without-icu
./b2 -j$(nproc) cxxflags="-fPIC" variant=release link=shared --disable-icu install

# ------ Install gflags ------
cd $CURRDIR
git clone --branch v2.2.2 --depth 1 https://github.com/gflags/gflags.git
cd glflags
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

# ------ Install glog ------
cd $CURRDIR
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake ..
make -j$(nproc)
make install

# Disable CGAL since it pulls many dependencies and increases the wheel size
#yum install -y yum-utils
#yum-config-manager --add-repo=http://springdale.princeton.edu/data/springdale/7/x86_64/os/Computational/
#yum install -y --nogpgcheck CGAL-devel

# ------ Install Eigen ------
cd $CURRDIR
EIGEN_VERSION="3.3.9"
export EIGEN_DIR="$CURRDIR/eigen-${EIGEN_VERSION}"
wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
tar -xvzf eigen-${EIGEN_VERSION}.tar.gz
cd $EIGEN_DIR
mkdir build && cd build
cmake ..

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
git checkout 8af292cb096b9478703821aa2e84730145b203a1
rm -f cmake/FindGlog.cmake
wget https://raw.githubusercontent.com/colmap/colmap/main/cmake/FindGlog.cmake -P cmake/
mkdir build/
cd build/
CXXFLAGS="-fPIC" CFLAGS="-fPIC" cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBoost_USE_STATIC_LIBS=OFF \
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
WHEEL_DIR="wheels/"
EIGEN3_INCLUDE_DIRS="$EIGEN_DIR" "${PYBIN}/pip" wheel --no-deps -w ${WHEEL_DIR} .

# Bundle external shared libraries into the wheels
OUT_DIR="/io/wheelhouse"
mkdir -p ${OUT_DIR}
for whl in ${WHEEL_DIR}/*.whl; do
    auditwheel repair "$whl" -w ${OUT_DIR} --plat manylinux2014_x86_64
done
ls -ltrh ${OUT_DIR}
