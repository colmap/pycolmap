#!/bin/bash

# Installation based off of https://colmap.github.io/install.html (COLMAP)
# and https://github.com/mihaidusmanu/pycolmap#getting-started (pycolmap)
# and http://ceres-solver.org/installation.html (Ceres)
# However, the OS is centOS 7, instead of Ubuntu.


uname -a
echo "Current CentOS Version:"
cat /etc/centos-release

yum -y install wget

ls -ltrh /io/

yum -y install qt5-qtbase-devel

# we cannot simply use `pip` or `python`, since points to old 2.7 version
PYBIN="/opt/python/$PYTHON_VERSION/bin"
PYVER_NUM=$($PYBIN/python -c "import sys;print(sys.version.split(\" \")[0])")
PYTHONVER="$(basename $(dirname $PYBIN))"

echo "Python bin path: $PYBIN"
echo "Python version number: $PYVER_NUM"
echo "Python version: $PYTHONVER"

export PATH=$PYBIN:$PATH

${PYBIN}/pip install auditwheel

PYTHON_EXECUTABLE=${PYBIN}/python
# We use distutils to get the include directory and the library path directly from the selected interpreter
# We provide these variables to CMake to hint what Python development files we wish to use in the build.
PYTHON_INCLUDE_DIR=$(${PYTHON_EXECUTABLE} -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")
PYTHON_LIBRARY=$(${PYTHON_EXECUTABLE} -c "import distutils.sysconfig as sysconfig; print(sysconfig.get_config_var('LIBDIR'))")


CURRDIR=$(pwd)
COLMAP_BRANCH="dev"

echo "Num. processes to use for building: ${nproc}"


# ----- Install boost (build it staticly) ---------------------------------------
# colmap needs only program_options filesystem graph system unit_test_framework)

cd $CURRDIR
yum install -y wget libicu libicu-devel centos-release-scl-rh devtoolset-7-gcc-c++

# Download and install Boost-1.65.1
mkdir -p boost && \
    cd boost && \
    wget -nv https://boostorg.jfrog.io/artifactory/main/release/1.65.1/source/boost_1_65_1.tar.gz && \
    tar xzf boost_1_65_1.tar.gz && \
    cd boost_1_65_1 && \
    ./bootstrap.sh --with-libraries=serialization,filesystem,thread,system,atomic,date_time,timer,chrono,program_options,regex,graph,test && \
    ./b2 -j$(nproc) cxxflags="-fPIC" runtime-link=static variant=release link=static install

# Boost should now be visible under /usr/local
ls -ltrh /usr/local

# ----------- Install dependencies from the default Ubuntu repositories -----------------
yum install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev
yum install libcgal-qt5-dev




cd $CURRDIR

# Note: `yum install gflags` will not work, since the version is too old (2.1)
# Note: `yum install glog` will also not work, since the version is too old
# Cloning and building https://github.com/google/glog.git will also not work, due to linker issues.
yum -y install gflags-devel glog-devel


cd $CURRDIR


# wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
# tar -xvzf eigen-3.4.0.tar.gz

# Using Eigen 3.3, not Eigen 3.4
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xvzf eigen-3.3.9.tar.gz

echo "CMAKE_PREFIX_PATH -> $CMAKE_PREFIX_PATH"
# export CMAKE_PREFIX_PATH="/eigen-3.4.0/cmake/"
export CMAKE_PREFIX_PATH="/eigen-3.3.9/cmake/"
echo "CMAKE_PREFIX_PATH -> $CMAKE_PREFIX_PATH"

ls -ltrh CMAKE_PREFIX_PATH/

# While Eigen is a header-only library, it still has to be built!
# Creates Eigen3Config.cmake from Eigen3Config.cmake.in
cd /eigen-3.3.9
mkdir build
cd build
cmake ..

ls -ltrh $CMAKE_PREFIX_PATH

cd $CURRDIR
# ----------- Install CERES solver -------------------------------------------------------
yum install libeigen3-dev # was not in COLMAP instructions
yum install libatlas-base-dev libsuitesparse-dev
yum install libgoogle-glog-dev libgflags-dev # was not in COLMAP instructions
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
#git checkout $(git describe --tags) # Checkout the latest release
# the tip of master is currently bugged
git checkout c2fab6502e5a341ff644c2bb2c5171ebd882b2d6
mkdir build
cd build
cmake .. -DBUILD_TESTING=OFF \
         -DBUILD_EXAMPLES=OFF \
         -DEigen3_DIR=$CMAKE_PREFIX_PATH
make -j$(nproc)
make install


cd $CURRDIR

echo ""
echo "PYTHON_EXECUTABLE:${PYTHON_EXECUTABLE}"
echo "PYTHON_INCLUDE_DIR:${PYTHON_INCLUDE_DIR}"
echo "PYTHON_LIBRARY:${PYTHON_LIBRARY}"
echo ""

# ---------- Fix broken dependencies -----

# try new boost install
yum install libboost-all-dev
yum install git
yum install cmake
yum install build-essential
yum install libboost-program-options-dev
yum install libboost-filesystem-dev
yum install libboost-graph-dev
yum install libboost-system-dev
yum install libboost-test-dev
yum install libeigen3-dev
yum install libsuitesparse-dev
# yum install libfreeimage-dev
yum install libgoogle-glog-dev
yum install libgflags-dev
yum install libglew-dev
yum install qtbase5-dev
yum install libqt5opengl5-dev
yum install libcgal-dev
yum install libcgal-qt5-dev

yum -y install freeimage

### ------ Build FreeImage from source and install --------------------
cd $CURRDIR
wget http://downloads.sourceforge.net/freeimage/FreeImage3180.zip
unzip FreeImage3180.zip
cd FreeImage
make
make install

# Install GLEW
yum -y install glew-devel

# ---------------- Clone COLMAP ----------------------------------------------------------
cd $CURRDIR
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout dev

# ----------- Build COLMAP ------------------------------------------------------------
mkdir build/
cd build/
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBoost_USE_STATIC_LIBS=ON \
         -DBOOST_ROOT=/usr/local \
         -DEIGEN3_INCLUDE_DIRS=/eigen-3.3.9

if [ $ec -ne 0 ]; then
    echo "Error:"
    cat ./CMakeCache.txt
    exit $ec
fi
set -e -x

make -j$(nproc) install

mkdir -p /io/wheelhouse

# ----------- Build pycolmap wheel -----------------------------------------------------
cd /io/
#cp package/setup_centos.py setup.py
cat setup.py

PLAT=manylinux2014_x86_64
#"${PYBIN}/python" setup.py bdist_wheel --python-tag=$PYTHONVER --plat-name=$PLAT
EIGEN3_INCLUDE_DIRS="/eigen-3.3.9" "${PYBIN}/python" setup.py bdist_wheel --plat-name=$PLAT #--python-tag=$PYTHONVER

# Bundle external shared libraries into the wheels
for whl in ./dist/*.whl; do
    auditwheel repair "$whl" -w /io/wheelhouse/ --no-update-tags
done

ls -ltrh /io/wheelhouse/
# for whl in /io/wheelhouse/*.whl; do
#     new_filename=$(echo $whl | sed "s#\.none-manylinux2014_x86_64\.#.#g")
#     new_filename=$(echo $new_filename | sed "s#\.manylinux2014_x86_64\.#.#g") # For 37 and 38
#     new_filename=$(echo $new_filename | sed "s#-none-#-#g")
#     mv $whl $new_filename
# done
