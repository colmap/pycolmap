#!/bin/bash
set -x -e

PYTHON_VERSIONS=("3.8" "3.9" "3.10")

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

brew update
brew upgrade
brew install wget cmake

for PYTHON_VERSION in ${PYTHON_VERSIONS[@]}; do
    brew install --force "python@${PYTHON_VERSION}"
    python${PYTHON_VERSION} -m pip install -U pip setuptools wheel cffi
done

brew install \
    git \
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
    cgal \
    sqlite3 \
    libomp \
    llvm \
    boost \
    lz4

brew info gcc
brew upgrade gcc
brew info gcc

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
git clone https://github.com/Skydes/colmap.git
cd colmap
git checkout paul/add-exe-lib
# patch src/colmap/util/CMakeLists.txt
sed -n -i '.txt' '/testing.h testing.cc/!p' src/colmap/util/CMakeLists.txt
# build
mkdir build
cd build
cmake .. -DGUI_ENABLED=OFF #-DBoost_USE_STATIC_LIBS=ON -DBOOSTROOT=${BOOST_DIR} -DBoost_NO_SYSTEM_PATHS=ON
make -j ${NUM_LOGICAL_CPUS} install
sudo make install

# Install `delocate` -- OSX equivalent of `auditwheel`
# see https://pypi.org/project/delocate/ for more details

cd $CURRDIR
# flags must be passed, to avoid the issue: `Unsupported compiler -- pybind11 requires C++11 support!`
# see https://github.com/quantumlib/qsim/issues/242 for more details
WHEEL_DIR="${CURRDIR}/wheelhouse_unrepaired/"
for PYTHON_VERSION in ${PYTHON_VERSIONS[@]}; do
    CC=/usr/local/opt/llvm/bin/clang \
        CXX=/usr/local/opt/llvm/bin/clang++ \
        LDFLAGS=-L/usr/local/opt/libomp/lib \
        python${PYTHON_VERSION} -m pip wheel --no-deps -w ${WHEEL_DIR} .
done

python${PYTHON_VERSIONS[0]} -m pip install -U delocate

# Bundle external shared libraries into the wheels
OUT_DIR="${CURRDIR}/wheelhouse"
mkdir -p ${OUT_DIR}
for whl in ${WHEEL_DIR}/*.whl; do
    delocate-listdeps --all "$whl"
    delocate-wheel -w "${OUT_DIR}" -v "$whl"
    rm $whl
done
ls -ltrh ${OUT_DIR}
