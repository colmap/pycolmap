#!/bin/bash
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

brew update
brew upgrade
brew install wget python cmake || true
# TODO: try without brew install of boost, but use version below
brew install git cmake boost eigen freeimage glog gflags suite-sparse ceres-solver glew cgal qt5

brew install llvm libomp

brew info gcc
brew upgrade gcc
brew info gcc

echo 'export PATH="/usr/local/opt/qt@5/bin:$PATH"' >> /Users/runner/.bash_profile
export Qt5_CMAKE_DIR="/usr/local/opt/qt@5/lib/cmake/Qt5"

CURRDIR=$(pwd)
ls -ltrh $CURRDIR

# Build Boost staticly
mkdir -p boost_build
cd boost_build
retry 3 wget https://boostorg.jfrog.io/artifactory/main/release/1.73.0/source/boost_1_73_0.tar.gz
tar xzf boost_1_73_0.tar.gz
cd boost_1_73_0
./bootstrap.sh --prefix=$CURRDIR/boost_install --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex clang-darwin
./b2 -j$(sysctl -n hw.logicalcpu) cxxflags="-fPIC" runtime-link=static variant=release link=static install

echo "CURRDIR is: ${CURRDIR}"

cd $CURRDIR
mkdir -p $CURRDIR/wheelhouse_unrepaired
mkdir -p $CURRDIR/wheelhouse

PYTHON_LIBRARY=$(cd $(dirname $0); pwd)/libpython-not-needed-symbols-exported-by-interpreter
touch ${PYTHON_LIBRARY}

declare -a PYTHON_VERS=( $1 )

git clone https://github.com/colmap/colmap.git

sed -i -e 's/Qt5 5.4/Qt5 5.15.2/g' colmap/CMakeLists.txt

for compiler in cc c++ gcc g++ clang clang++
do
    which $compiler
    $compiler --version
done

# Get the python version numbers only by splitting the string
PYBIN="/usr/local/opt/$PYTHON_VERS/bin"
PYTHONVER="$(basename $(dirname $PYBIN))"
export PATH=$PYBIN:/usr/local/bin:$PATH
echo "Python bin path: $PYBIN"
echo "Python version: $PYTHONVER"

# Install `delocate` -- OSX equivalent of `auditwheel`
# see https://pypi.org/project/delocate/ for more details
cd $CURRDIR
"${PYBIN}/pip3" install delocate==0.10.0

ls -ltrh /usr/local
ls -ltrh /usr/local/opt

cd $CURRDIR
cd colmap
git checkout dev
mkdir build
cd build
cmake .. -DQt5_DIR=$Qt5_CMAKE_DIR

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
Qt5_DIR="$Qt5_CMAKE_DIR" CC=/usr/local/opt/llvm/bin/clang CXX=/usr/local/opt/llvm/bin/clang++ LDFLAGS=-L/usr/local/opt/libomp/lib "${PYBIN}/python3" setup.py bdist_wheel
cp ./dist/*.whl $CURRDIR/wheelhouse_unrepaired

# Bundle external shared libraries into the wheels
ls -ltrh $CURRDIR/wheelhouse_unrepaired/
for whl in $CURRDIR/wheelhouse_unrepaired/*.whl; do
    delocate-listdeps --all "$whl"
    delocate-wheel -w "$CURRDIR/wheelhouse" -v "$whl"
    rm $whl
done
