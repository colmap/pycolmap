#!/bin/bash
set -x -e

# See https://github.com/actions/setup-python/issues/577
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete
# See https://github.com/actions/setup-python/issues/577#issuecomment-1500828576
rm /usr/local/bin/go || true
rm /usr/local/bin/gofmt || true

CURRDIR=$(pwd)
NUM_LOGICAL_CPUS=$(sysctl -n hw.logicalcpu)
echo "Number of logical CPUs is: ${NUM_LOGICAL_CPUS}"

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
    boost \
    lz4

# Install Boost
#mkdir -p boost
#cd boost
#wget https://boostorg.jfrog.io/artifactory/main/release/1.81.0/source/boost_1_81_0.tar.gz
#tar xzf boost_1_81_0.tar.gz
#cd boost_1_81_0
#BOOST_DIR=$CURRDIR/boost_install
#./bootstrap.sh --prefix=${BOOST_DIR} --with-libraries=filesystem,system,program_options,graph,test --without-icu clang-darwin
#./b2 -j$(sysctl -n hw.logicalcpu) cxxflags="-fPIC" link=static runtime-link=static variant=release --disable-icu --prefix=${BOOST_DIR} install

cd $CURRDIR
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c0355417328f3706a30a9265fd52bc7a5aa4cb8c
mkdir build
cd build
cmake .. -DGUI_ENABLED=OFF -DCUDA_ENABLED=OFF -DCGAL_ENABLED=OFF #-DBoost_USE_STATIC_LIBS=ON -DBOOSTROOT=${BOOST_DIR} -DBoost_NO_SYSTEM_PATHS=ON
make -j ${NUM_LOGICAL_CPUS} install
