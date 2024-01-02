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
    eigen \
    freeimage \
    flann \
    glog \
    gflags \
    metis \
    suite-sparse \
    ceres-solver \
    glew \
    sqlite3 \
    libomp \
    llvm \
    lz4

# Install Boost
mkdir boost && cd boost
BOOST_FILENAME="boost_1_83_0"
wget https://boostorg.jfrog.io/artifactory/main/release/1.83.0/source/${BOOST_FILENAME}.tar.gz
tar xzf ${BOOST_FILENAME}.tar.gz
cd ${BOOST_FILENAME}
BOOST_DIR=${CURRDIR}/boost_install
./bootstrap.sh --prefix=${BOOST_DIR} \
    --with-libraries=filesystem,system,program_options,graph,test \
    --without-icu clang-darwin
./b2 -j ${NUM_LOGICAL_CPUS} \
    cxxflags="-fPIC" \
    link=static \
    runtime-link=static \
    variant=release \
    --disable-icu \
    --prefix=${BOOST_DIR} \
    install

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c206c7a333c8b2ef818135bdfe9d31a28bd2eb2b
mkdir build && cd build
cmake .. -DGUI_ENABLED=OFF \
    -DCUDA_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DBoost_USE_STATIC_LIBS=OFF \
    -DBOOSTROOT=${BOOST_DIR} \
    -DBoost_NO_SYSTEM_PATHS=ON
make -j ${NUM_LOGICAL_CPUS} install
