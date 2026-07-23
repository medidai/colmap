#!/bin/bash
set -euo pipefail
set -x

PREFIX="${1:?native prefix path is required}"
NATIVE_CACHE_DIR="${NATIVE_CACHE_DIR:?native cache path is required}"
VCPKG_COMMIT_ID="${VCPKG_COMMIT_ID:?vcpkg commit is required}"
VCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET:-arm64-osx-release}"
MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-15.0}"
SOURCE_DIR="$(pwd)"
VCPKG_ROOT="${NATIVE_CACHE_DIR}/vcpkg"
BUILD_DIR="${NATIVE_CACHE_DIR}/colmap-build"

find /usr/local/bin -lname '*/Library/Frameworks/Python.framework/*' -delete

export HOMEBREW_NO_AUTO_UPDATE=1
sudo xcode-select --reset

if ! command -v gfortran >/dev/null; then
    GFORTRAN_PATH="$(find "$(brew --prefix)/bin" -maxdepth 1 -name 'gfortran-*' -print | sort | tail -n 1)"
    test -n "${GFORTRAN_PATH}"
    sudo ln -sf "${GFORTRAN_PATH}" "$(brew --prefix)/bin/gfortran"
fi

if [ ! -d "${VCPKG_ROOT}/.git" ]; then
    git clone --filter=blob:none https://github.com/microsoft/vcpkg "${VCPKG_ROOT}"
fi
if ! git -C "${VCPKG_ROOT}" cat-file -e "${VCPKG_COMMIT_ID}^{commit}"; then
    git -C "${VCPKG_ROOT}" fetch --depth=1 origin "${VCPKG_COMMIT_ID}"
fi
git -C "${VCPKG_ROOT}" checkout --detach "${VCPKG_COMMIT_ID}"
"${VCPKG_ROOT}/bootstrap-vcpkg.sh" -disableMetrics

mkdir -p \
    "${PREFIX}/colmap" \
    "${PREFIX}/vcpkg_installed" \
    "${NATIVE_CACHE_DIR}/ccache" \
    "${NATIVE_CACHE_DIR}/vcpkg-binaries"

export CCACHE_BASEDIR="${SOURCE_DIR}"
export CCACHE_COMPILERCHECK=content
export CCACHE_DIR="${NATIVE_CACHE_DIR}/ccache"
export CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-500M}"
export VCPKG_BINARY_SOURCES="clear;files,${NATIVE_CACHE_DIR}/vcpkg-binaries,readwrite"
export VCPKG_DISABLE_METRICS=1

rm -rf "${BUILD_DIR}"
"$(brew --prefix cmake)/bin/cmake" -S "${SOURCE_DIR}" -B "${BUILD_DIR}" -GNinja \
    -DCUDA_ENABLED=OFF \
    -DGUI_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DLSD_ENABLED=OFF \
    -DCCACHE_ENABLED=ON \
    -DSIMD_ENABLED=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}/colmap" \
    -DCMAKE_MAKE_PROGRAM="$(brew --prefix ninja)/bin/ninja" \
    -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
    -DVCPKG_INSTALLED_DIR="${PREFIX}/vcpkg_installed" \
    -DVCPKG_OVERLAY_TRIPLETS="${SOURCE_DIR}/pycolmap/ci/triplets" \
    -DVCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET}" \
    -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET}"
"$(brew --prefix cmake)/bin/cmake" --build "${BUILD_DIR}" --target install

ccache --show-stats --verbose
ccache --evict-older-than 7d
rm -rf \
    "${BUILD_DIR}" \
    "${VCPKG_ROOT}/buildtrees" \
    "${VCPKG_ROOT}/downloads" \
    "${VCPKG_ROOT}/packages"
touch "${PREFIX}/.complete"
