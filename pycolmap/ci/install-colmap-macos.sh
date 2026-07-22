#!/bin/bash
set -x -e
CURRDIR=$(pwd)

# Fix `brew link` error.
find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete

brew update
brew install git cmake ninja gfortran ccache

sudo xcode-select --reset

# When building lapack-reference, vcpkg/cmake looks for an unversioned gfortran.
if ! command -v gfortran >/dev/null; then
    GFORTRAN_PATH="$(find "$(brew --prefix)/bin" -maxdepth 1 -name 'gfortran-*' -print | sort | tail -n 1)"
    test -n "$GFORTRAN_PATH"
    sudo ln -sf "$GFORTRAN_PATH" "$(brew --prefix)/bin/gfortran"
fi

# Setup vcpkg
git clone https://github.com/microsoft/vcpkg ${VCPKG_INSTALLATION_ROOT}
cd ${VCPKG_INSTALLATION_ROOT}
git checkout ${VCPKG_COMMIT_ID}
./bootstrap-vcpkg.sh
./vcpkg integrate install

# Build COLMAP
cd ${CURRDIR}
mkdir build && cd build
"$(brew --prefix cmake)/bin/cmake" .. -GNinja \
    -DCUDA_ENABLED=OFF \
    -DGUI_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DLSD_ENABLED=OFF \
    -DCCACHE_ENABLED=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_MAKE_PROGRAM="$(brew --prefix ninja)/bin/ninja" \
    -DCMAKE_TOOLCHAIN_FILE="${CMAKE_TOOLCHAIN_FILE}" \
    -DVCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET}" \
    -DCMAKE_OSX_ARCHITECTURES="${CMAKE_OSX_ARCHITECTURES}" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET}" \
    `if [[ ${CIBW_ARCHS_MACOS} == "arm64" ]]; then echo "-DSIMD_ENABLED=OFF"; fi`
ninja
sudo ninja install

ccache --show-stats --verbose
ccache --evict-older-than 1d
ccache --show-stats --verbose
