#!/bin/bash

# Author: John Lambert (johnwlambert)

set -x -e

function retry {
  local retries=$1
  shift

  local count=0
  until "$@"; do
    exit=$?
    wait=$((2 ** $count))
    count=$(($count + 1))
    if [ $count -lt $retries ]; then
      echo "Retry $count/$retries exited $exit, retrying in $wait seconds..."
      sleep $wait
    else
      echo "Retry $count/$retries exited $exit, no more retries left."
      return $exit
    fi
  done
  return 0
}

declare -a PYTHON_VERSION=( $1 )
# See https://github.com/actions/setup-python/issues/577
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete
rm /usr/local/bin/go || true
rm /usr/local/bin/gofmt || true

brew update
brew upgrade
brew install wget cmake
brew install --force $PYTHON_VERSION

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
    lz4

brew info gcc
brew upgrade gcc
brew info gcc

# Get the python version numbers only by splitting the string
PYBIN="/usr/local/opt/$PYTHON_VERSION/libexec/bin"
INTERPRETER="$PYBIN/python"
ls -ltr /usr/local/opt/python*
ls -ltr $PYBIN
ls -ltr $INTERPRETER
export PATH=$PYBIN:/usr/local/bin:$PATH
echo "Python bin path: $PYBIN"

CURRDIR=$(pwd)
ls -ltrh $CURRDIR

# Build Boost staticly
mkdir -p boost_build
cd boost_build
retry 3 wget https://boostorg.jfrog.io/artifactory/main/release/1.81.0/source/boost_1_81_0.tar.gz
tar xzf boost_1_81_0.tar.gz
cd boost_1_81_0
./bootstrap.sh --prefix=$CURRDIR/boost_install --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex clang-darwin
./b2 -j$(sysctl -n hw.logicalcpu) cxxflags="-fPIC" runtime-link=static variant=release link=static install

echo "CURRDIR is: ${CURRDIR}"

cd $CURRDIR
mkdir -p $CURRDIR/wheelhouse_unrepaired
mkdir -p $CURRDIR/wheelhouse

PYTHON_LIBRARY=$(cd $(dirname $0); pwd)/libpython-not-needed-symbols-exported-by-interpreter
touch ${PYTHON_LIBRARY}

git clone https://github.com/colmap/colmap.git

for compiler in cc c++ gcc g++ clang clang++
do
    which $compiler
    $compiler --version
done

# Install `delocate` -- OSX equivalent of `auditwheel`
# see https://pypi.org/project/delocate/ for more details
cd $CURRDIR
$INTERPRETER -m pip install -U delocate
$INTERPRETER -m pip install -U pip setuptools wheel cffi

ls -ltrh /usr/local
ls -ltrh /usr/local/opt

cd $CURRDIR
cd colmap
git checkout dev
mkdir build
cd build
cmake .. -DGUI_ENABLED=OFF

# examine exit code of last command
ec=$?
if [ $ec -ne 0 ]; then
    echo "Error:"
    cat ./CMakeCache.txt
    exit $ec
fi
set -e -x

NUM_LOGICAL_CPUS=$(sysctl -n hw.logicalcpu)
echo "Number of logical CPUs is: ${NUM_LOGICAL_CPUS}"
make -j $NUM_LOGICAL_CPUS install
sudo make install

cd $CURRDIR
cat setup.py
# flags must be passed, to avoid the issue: `Unsupported compiler -- pybind11 requires C++11 support!`
# see https://github.com/quantumlib/qsim/issues/242 for more details
CC=/usr/local/opt/llvm/bin/clang CXX=/usr/local/opt/llvm/bin/clang++ LDFLAGS=-L/usr/local/opt/libomp/lib $INTERPRETER setup.py bdist_wheel
cp ./dist/*.whl $CURRDIR/wheelhouse_unrepaired

# Bundle external shared libraries into the wheels
ls -ltrh $CURRDIR/wheelhouse_unrepaired/
for whl in $CURRDIR/wheelhouse_unrepaired/*.whl; do
    delocate-listdeps --all "$whl"
    delocate-wheel -w "$CURRDIR/wheelhouse" -v "$whl"
    rm $whl
done
