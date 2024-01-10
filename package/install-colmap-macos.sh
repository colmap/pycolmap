#!/bin/bash
set -x -e
CURRDIR=$(pwd)

# See https://github.com/actions/setup-python/issues/577
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete
# See https://github.com/actions/setup-python/issues/577#issuecomment-1500828576
rm /usr/local/bin/go || true
rm /usr/local/bin/gofmt || true

# Updating requires Xcode 14.0, which cannot be installed on macOS 11.
brew remove swiftlint
brew remove node@18

brew update
brew install git cmake ninja llvm ccache

git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
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
    freeimage \
    metis \
    gflags \
    glog \
    gtest \
    sqlite3
./vcpkg integrate install

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout ${COLMAP_COMMIT_ID}
mkdir build && cd build
export ARCHFLAGS="-arch ${CIBW_ARCHS_MACOS}"
cmake .. -GNinja -DGUI_ENABLED=OFF \
    -DCUDA_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCCACHE_ENABLED=ON \
    -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" \
    -DVCPKG_TARGET_TRIPLET=${VCPKG_TARGET_TRIPLET} \
    -DCMAKE_OSX_ARCHITECTURES=${CMAKE_OSX_ARCHITECTURES} \
    `if [[ ${CIBW_ARCHS_MACOS} == "arm64" ]]; then echo "-DSIMD_ENABLED=OFF"; fi`
ninja install

ccache --show-stats --verbose
ccache --evict-older-than 1d
ccache --show-stats --verbose
