#!/bin/bash
set -euo pipefail
set -x

PREFIX="${1:?native prefix path is required}"
NATIVE_CACHE_DIR="${NATIVE_CACHE_DIR:?native cache path is required}"
VCPKG_COMMIT_ID="${VCPKG_COMMIT_ID:?vcpkg commit is required}"
VCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET:-x64-linux-release}"
NINJA_VERSION="${NINJA_VERSION:?Ninja version is required}"
NINJA_LINUX_SHA256="${NINJA_LINUX_SHA256:?Ninja checksum is required}"
CCACHE_LINUX_SHA256="${CCACHE_LINUX_SHA256:?ccache checksum is required}"
BUILD_CUDA_ENABLED="${BUILD_CUDA_ENABLED:-false}"
SOURCE_DIR="$(pwd)"
TOOLS_DIR="${NATIVE_CACHE_DIR}/bin"
VCPKG_ROOT="${NATIVE_CACHE_DIR}/vcpkg"
BUILD_DIR="${NATIVE_CACHE_DIR}/colmap-build"

yum install -y \
    gcc-toolset-12-gcc \
    gcc-toolset-12-gcc-c++ \
    gcc-toolset-12-gcc-gfortran \
    scl-utils \
    git \
    cmake3 \
    curl \
    zip \
    unzip \
    tar \
    xz

set +u
source scl_source enable gcc-toolset-12
set -u

CUDA_HOME="/usr/local/cuda"
if [ ! -d "${CUDA_HOME}" ] && [ -d "${CUDA_HOME}-12.9" ]; then
    ln -s "${CUDA_HOME}-12.9" "${CUDA_HOME}"
fi
if [ -d "${CUDA_HOME}" ]; then
    export PATH="${CUDA_HOME}/bin:${PATH}"
    echo "${CUDA_HOME}/lib64" > /etc/ld.so.conf.d/cuda.conf
fi

mkdir -p "${TOOLS_DIR}"
if [ ! -x "${TOOLS_DIR}/ninja" ]; then
    NINJA_DOWNLOAD_DIR="$(mktemp -d)"
    curl -sSLo "${NINJA_DOWNLOAD_DIR}/ninja-linux.zip" \
        "https://github.com/ninja-build/ninja/releases/download/v${NINJA_VERSION}/ninja-linux.zip"
    echo "${NINJA_LINUX_SHA256}  ${NINJA_DOWNLOAD_DIR}/ninja-linux.zip" | sha256sum --check -
    unzip -q "${NINJA_DOWNLOAD_DIR}/ninja-linux.zip" -d "${TOOLS_DIR}"
    chmod 0755 "${TOOLS_DIR}/ninja"
fi

if [ ! -x "${TOOLS_DIR}/ccache" ]; then
    CCACHE_VERSION="4.10.1"
    CCACHE_ARCHIVE="ccache-${CCACHE_VERSION}-linux-x86_64"
    CCACHE_DOWNLOAD_DIR="$(mktemp -d)"
    curl -sSLo "${CCACHE_DOWNLOAD_DIR}/${CCACHE_ARCHIVE}.tar.xz" \
        "https://github.com/ccache/ccache/releases/download/v${CCACHE_VERSION}/${CCACHE_ARCHIVE}.tar.xz"
    echo "${CCACHE_LINUX_SHA256}  ${CCACHE_DOWNLOAD_DIR}/${CCACHE_ARCHIVE}.tar.xz" | sha256sum --check -
    tar -C "${CCACHE_DOWNLOAD_DIR}" -xf "${CCACHE_DOWNLOAD_DIR}/${CCACHE_ARCHIVE}.tar.xz"
    cp "${CCACHE_DOWNLOAD_DIR}/${CCACHE_ARCHIVE}/ccache" "${TOOLS_DIR}/ccache"
    chmod 0755 "${TOOLS_DIR}/ccache"
fi
export PATH="${TOOLS_DIR}:${PATH}"

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
    "${PREFIX}/tools/bin" \
    "${PREFIX}/vcpkg_installed" \
    "${NATIVE_CACHE_DIR}/ccache" \
    "${NATIVE_CACHE_DIR}/vcpkg-binaries"
install -m 0755 "${TOOLS_DIR}/ninja" "${PREFIX}/tools/bin/ninja"
ln -sf /usr/bin/cmake3 "${PREFIX}/tools/bin/cmake"

export CCACHE_BASEDIR="${SOURCE_DIR}"
export CCACHE_COMPILERCHECK=content
export CCACHE_DIR="${NATIVE_CACHE_DIR}/ccache"
export CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-500M}"
export VCPKG_BINARY_SOURCES="clear;files,${NATIVE_CACHE_DIR}/vcpkg-binaries,readwrite"
export VCPKG_DISABLE_METRICS=1

rm -rf "${BUILD_DIR}"
cmake3 -S "${SOURCE_DIR}" -B "${BUILD_DIR}" -GNinja \
    -DCUDA_ENABLED="${BUILD_CUDA_ENABLED}" \
    -DCMAKE_CUDA_ARCHITECTURES=89 \
    -DGUI_ENABLED=OFF \
    -DCGAL_ENABLED=OFF \
    -DLSD_ENABLED=OFF \
    -DCCACHE_ENABLED=ON \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${PREFIX}/colmap" \
    -DCMAKE_MAKE_PROGRAM="${TOOLS_DIR}/ninja" \
    -DCMAKE_TOOLCHAIN_FILE="${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" \
    -DVCPKG_INSTALLED_DIR="${PREFIX}/vcpkg_installed" \
    -DVCPKG_OVERLAY_TRIPLETS="${SOURCE_DIR}/pycolmap/ci/triplets" \
    -DVCPKG_TARGET_TRIPLET="${VCPKG_TARGET_TRIPLET}" \
    -DCMAKE_EXE_LINKER_FLAGS_INIT=-ldl
cmake3 --build "${BUILD_DIR}" --target install

ccache --show-stats --verbose
ccache --evict-older-than 7d
rm -rf \
    "${BUILD_DIR}" \
    "${VCPKG_ROOT}/buildtrees" \
    "${VCPKG_ROOT}/downloads" \
    "${VCPKG_ROOT}/packages"
touch "${PREFIX}/.complete"
