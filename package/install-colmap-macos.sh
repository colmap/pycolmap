#!/bin/bash
set -x -e

CURRDIR=$(pwd)
NUM_LOGICAL_CPUS=$(sysctl -n hw.logicalcpu)

# See https://github.com/actions/setup-python/issues/577
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete
# See https://github.com/actions/setup-python/issues/577#issuecomment-1500828576
rm /usr/local/bin/go || true
rm /usr/local/bin/gofmt || true

# Updating requires Xcode 14.0, which cannot be installed on macOS 11.
brew remove swiftlint
brew remove node@18

brew update
brew install \
    git \
    wget \
    cmake \
    llvm \
    gfortran
export CMAKE_FORTRAN_COMPILER=$(which gfortran-13)

cd ${CURRDIR}
git clone https://github.com/microsoft/vcpkg
cd vcpkg
VCPKG_DIR=$(pwd)
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build --triplet=x64-osx boost-algorithm boost-filesystem boost-graph boost-heap boost-program-options boost-property-map boost-property-tree boost-regex boost-system ceres[lapack,suitesparse] eigen3 flann freeimage metis gflags glog gtest sqlite3
./vcpkg integrate install

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c0355417328f3706a30a9265fd52bc7a5aa4cb8c
mkdir build && cd build
cmake .. -DGUI_ENABLED=OFF \
    -DCUDA_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DCMAKE_TOOLCHAIN_FILE=${VCPKG_DIR}/scripts/buildsystems/vcpkg.cmake \
    -DVCPKG_TARGET_TRIPLET=x64-osx
make -j ${NUM_LOGICAL_CPUS} install
