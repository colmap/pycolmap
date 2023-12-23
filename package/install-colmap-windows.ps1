$CURRDIR = $PWD

curl.exe -L -o "ninja.zip" "https://github.com/ninja-build/ninja/releases/download/v1.10.2/ninja-win.zip"
Expand-Archive -LiteralPath "${CURRDIR}/ninja.zip" -DestinationPath ${CURRDIR}
$NINJA_PATH = "${CURRDIR}/ninja.exe"

cd ${CURRDIR}
git clone https://github.com/colmap/colmap.git
cd colmap
git checkout "${env:COLMAP_COMMIT_ID}"

& "./scripts/shell/enter_vs_dev_shell.ps1"

[System.Collections.ArrayList]$DEPS = Get-Content -Path ".azure-pipelines/build-windows-vcpkg.txt"
$DEPS.Remove("cgal")
$DEPS.Remove("qt5-base")
$DEPS.Remove("glew")
& "${env:VCPKG_INSTALLATION_ROOT}/vcpkg.exe" install --recurse --clean-after-build @DEPS
& "${env:VCPKG_INSTALLATION_ROOT}/vcpkg.exe" integrate install

mkdir build
cd build
cmake .. `
  -GNinja `
  -DCMAKE_MAKE_PROGRAM="${NINJA_PATH}" `
  -DCMAKE_BUILD_TYPE="Release" `
  -DCUDA_ENABLED="OFF" `
  -DCGAL_ENABLED="OFF" `
  -DGUI_ENABLED="OFF"
& ${NINJA_PATH} install
