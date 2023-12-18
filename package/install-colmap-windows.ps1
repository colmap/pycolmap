CURRDIR = $PWD

${VCPKG_INSTALLATION_ROOT}/vcpkg.exe integrate install

curl -L -o ninja.zip https://github.com/ninja-build/ninja/releases/download/v1.10.2/ninja-win.zip
Expand-Archive -LiteralPath ${CURRDIR}/ninja.zip -DestinationPath ${CURRDIR}
NINJA_PATH = "${CURRDIR}/ninja.exe"

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout c0355417328f3706a30a9265fd52bc7a5aa4cb8c

${VCPKG_INSTALLATION_ROOT}/vcpkg.exe install --recurse @./.azure-pipelines/build-windows-vcpkg.txt --clean-after-build

mkdir build
cd build
cmake .. `
  -GNinja `
  -DCMAKE_MAKE_PROGRAM=${NINJA_PATH} `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_TOOLCHAIN_FILE=${VCPKG_INSTALLATION_ROOT}/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows
${NINJA_PATH} install
