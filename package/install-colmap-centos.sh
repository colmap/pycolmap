#!/bin/bash
set -e -x
uname -a

CURRDIR=$(pwd)
echo "${CURRDIR}"

# ------ Install dependencies from the default repositories ------
cd ${CURRDIR}
yum install -y \
    wget \
    git \
    gcc gcc-c++ make \
    freeimage-devel \
    metis-devel \
    glew-devel \
    suitesparse-devel \
    atlas-devel \
    lapack-devel \
    blas-devel \
    flann \
    flann-devel \
    lz4 \
    lz4-devel

# ------ Install boost ------
cd ${CURRDIR}
export BOOST_FILENAME=boost_1_71_0
wget -nv https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/${BOOST_FILENAME}.tar.gz
tar xzf ${BOOST_FILENAME}.tar.gz
cd ${BOOST_FILENAME}
./bootstrap.sh --with-libraries=filesystem,system,program_options,graph,test --without-icu
./b2 -j$(nproc) cxxflags="-fPIC" variant=release link=shared --disable-icu install

# ------ Install gflags ------
cd ${CURRDIR}
git clone --branch v2.2.2 --depth 1 https://github.com/gflags/gflags.git
cd gflags
mkdir build && cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j$(nproc) install

# ------ Install glog ------
cd ${CURRDIR}
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog.git
cd glog
mkdir build && cd build
cmake ..
make -j$(nproc) install

# ------ Install Eigen ------
cd ${CURRDIR}
EIGEN_VERSION="3.3.9"
export EIGEN_DIR="${CURRDIR}/eigen"
wget https://gitlab.com/libeigen/eigen/-/archive/${EIGEN_VERSION}/eigen-${EIGEN_VERSION}.tar.gz
tar -xvzf eigen-${EIGEN_VERSION}.tar.gz
mv eigen-${EIGEN_VERSION} ${EIGEN_DIR}
cd ${EIGEN_DIR}
mkdir build && cd build
cmake ..

# ------ Install CERES solver ------
cd ${CURRDIR}
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout $(git describe --tags) # Checkout the latest release
mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF \
         -DBUILD_EXAMPLES=OFF \
         -DEigen3_DIR="${EIGEN_DIR}/cmake/"
make -j$(nproc) install

# ------ Build COLMAP ------
cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c0355417328f3706a30a9265fd52bc7a5aa4cb8c
mkdir build && cd build
CXXFLAGS="-fPIC" CFLAGS="-fPIC" cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBoost_USE_STATIC_LIBS=OFF \
         -DBOOST_ROOT=/usr/local \
         -DCUDA_ENABLED=OFF \
         -DCGAL_ENABLED=OFF \
         -DGUI_ENABLED=OFF \
         -DEIGEN3_INCLUDE_DIRS=$EIGEN_DIR
make -j$(nproc) install
