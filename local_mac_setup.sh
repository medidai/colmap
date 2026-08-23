#!/usr/bin/env bash
# Build this COLMAP tree on macOS and install pycolmap into a chosen Python.
#
# Homebrew Boost must win over a leftover Boost in /usr/local (headers are
# picked from /usr/local/include before -isystem /opt/homebrew/include).
# Pass Boost's own include dir via -I so Clang does not drop it as a duplicate
# of /opt/homebrew/include.
#
# Usage:
#   ./local_mac_setup.sh
#   PYTHON=/path/to/venv/bin/python ./local_mac_setup.sh
#   INSTALL_PREFIX=/usr/local ./local_mac_setup.sh   # needs write/sudo for install

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOMEBREW_PREFIX="${HOMEBREW_PREFIX:-$(brew --prefix)}"
BOOST_ROOT="${BOOST_ROOT:-$(brew --prefix boost)}"
BOOST_INCLUDE="${BOOST_ROOT}/include"
BOOST_DIR="$(ls -d "${HOMEBREW_PREFIX}/lib/cmake"/Boost-* 2>/dev/null | sort -V | tail -1)"

BUILD_DIR="${BUILD_DIR:-${ROOT}/build}"
INSTALL_PREFIX="${INSTALL_PREFIX:-${ROOT}/install}"
PYTHON="${PYTHON:-/Users/xallt/progs/medida/medida-3d/.venv/bin/python}"

if [[ ! -d "${BOOST_INCLUDE}" ]]; then
  echo "Boost headers not found at ${BOOST_INCLUDE}. Install with: brew install boost" >&2
  exit 1
fi
if [[ -z "${BOOST_DIR}" || ! -d "${BOOST_DIR}" ]]; then
  echo "Boost CMake package not found under ${HOMEBREW_PREFIX}/lib/cmake/Boost-*" >&2
  exit 1
fi
if [[ ! -x "${PYTHON}" ]]; then
  echo "Python not found: ${PYTHON}" >&2
  echo "Set PYTHON=/path/to/venv/bin/python" >&2
  exit 1
fi

BOOST_FLAGS="-I${BOOST_INCLUDE}"

echo "Homebrew:        ${HOMEBREW_PREFIX}"
echo "Boost include:   ${BOOST_INCLUDE}"
echo "Boost CMake:     ${BOOST_DIR}"
echo "COLMAP build:    ${BUILD_DIR}"
echo "COLMAP prefix:   ${INSTALL_PREFIX}"
echo "Python:          ${PYTHON}"

cmake -S "${ROOT}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_PREFIX_PATH="${HOMEBREW_PREFIX}" \
  -DCMAKE_C_FLAGS="${BOOST_FLAGS}" \
  -DCMAKE_CXX_FLAGS="${BOOST_FLAGS}"

cmake --build "${BUILD_DIR}"
cmake --install "${BUILD_DIR}"

export CMAKE_PREFIX_PATH="${HOMEBREW_PREFIX}"
export colmap_DIR="${INSTALL_PREFIX}/share/colmap"
export Boost_DIR="${BOOST_DIR}"
export CMAKE_C_FLAGS="${BOOST_FLAGS}"
export CMAKE_CXX_FLAGS="${BOOST_FLAGS}"

if command -v uv >/dev/null 2>&1; then
  uv pip install --python "${PYTHON}" "${ROOT}"
else
  "${PYTHON}" -m pip install "${ROOT}"
fi

"${PYTHON}" -c 'import pycolmap; print(pycolmap.__file__); print(pycolmap.Point2D())'
