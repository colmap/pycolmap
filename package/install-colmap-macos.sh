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
    llvm

# for lapack-reference
export CMAKE_Fortran_COMPILER=$(which gfortran-13)
export CMAKE_Fortran_COMPILER_INIT="gfortran-13"
ln -s $(which gfortran-13) "$(dirname $(which gfortran-13))/gfortran"

cd ${CURRDIR}
#git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
git clone --branch sarlinpe/lapack-osx https://github.com/sarlinpe/vcpkg ${VCPKG_INSTALLATION_ROOT}

cd ${VCPKG_INSTALLATION_ROOT}
./bootstrap-vcpkg.sh
./vcpkg install --recurse --clean-after-build --triplet=${VCPKG_TARGET_TRIPLET} boost-algorithm boost-filesystem boost-graph boost-heap boost-program-options boost-property-map boost-property-tree boost-regex boost-system eigen3 flann freeimage metis gflags glog gtest sqlite3 ceres[lapack,suitesparse,tools]
./vcpkg integrate install

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c0355417328f3706a30a9265fd52bc7a5aa4cb8c
mkdir build && cd build
export ARCHFLAGS="-arch ${CIBW_ARCHS_MACOS}"
cmake .. -DGUI_ENABLED=OFF \
    -DCUDA_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DCMAKE_TOOLCHAIN_FILE="${VCPKG_INSTALLATION_ROOT}/scripts/buildsystems/vcpkg.cmake" \
    -DVCPKG_TARGET_TRIPLET=${VCPKG_TARGET_TRIPLET} \
    -DCMAKE_OSX_ARCHITECTURES=${CIBW_ARCHS_MACOS} \
    `if [[ ${CIBW_ARCHS_MACOS} == "arm64" ]]; then echo "-DSIMD_ENABLED=OFF"; fi`
make -j ${NUM_LOGICAL_CPUS} install
