#!/bin/bash
set -e -x
uname -a
CURRDIR=$(pwd)

yum install -y gcc gcc-c++ ninja-build curl zip unzip tar

git clone --branch sarlinpe/libraw-jaspter-nodefaults https://github.com/sarlinpe/vcpkg ${VCPKG_INSTALLATION_ROOT}
cd ${VCPKG_INSTALLATION_ROOT}
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build --triplet=${VCPKG_TARGET_TRIPLET} \
    boost-algorithm \
    boost-filesystem \
    boost-graph \
    boost-heap \
    boost-program-options \
    boost-property-map \
    boost-property-tree \
    boost-regex \
    boost-system \
    ceres[lapack,suitesparse] \
    eigen3 \
    flann \
    jasper[core] \
    freeimage \
    metis \
    gflags \
    glog \
    gtest \
    sqlite3
# We force the core option of jasper to disable the unwanted opengl option.
./vcpkg integrate install

cd ${CURRDIR}
git clone  --branch sarlinpe/link-dl https://github.com/sarlinpe/colmap.git
#git clone https://github.com/colmap/colmap.git
cd colmap
#git checkout ${COLMAP_COMMIT_ID}
mkdir build && cd build
CXXFLAGS="-fPIC" CFLAGS="-fPIC" cmake .. -GNinja \
    -DCUDA_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DGUI_ENABLED=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" \
    -DVCPKG_TARGET_TRIPLET=${VCPKG_TARGET_TRIPLET}
ninja install
