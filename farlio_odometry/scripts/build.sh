#!/usr/bin/env bash
# Build the FAR-LIO ROS 1 port in a dedicated catkin space, without touching the
# build/devel trees of the surrounding lio_ws (which hosts ~400 other packages).
#
# Usage:  ./scripts/build.sh [workspace_dir]
#
# Requirements: see README.md (g++ >= 10 for C++20, CMake >= 3.25).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"          # .../src/farlio_ros1/farlio_odometry
SRC_ROOT="$(cd "${PKG_DIR}/../.." && pwd)"         # .../lio_ws/src
WS_DIR="${1:-$(cd "${SRC_ROOT}/.." && pwd)/farlio_ws}"

# Toolchain: prefer an explicitly exported CXX, then g++-10, then the default g++.
CXX_BIN="${CXX:-}"
CC_BIN="${CC:-}"
if [[ -z "${CXX_BIN}" ]]; then
  # Prefer the locally extracted gcc-10 used on this machine (no root needed).
  if [[ -x "${HOME}/opt/gcc-10/usr/bin/g++-10" ]]; then
    CXX_BIN="${HOME}/opt/gcc-10/usr/bin/g++-10"; CC_BIN="${HOME}/opt/gcc-10/usr/bin/gcc-10"
  elif command -v g++-10 >/dev/null 2>&1; then
    CXX_BIN="$(command -v g++-10)"; CC_BIN="$(command -v gcc-10)"
  elif command -v g++-11 >/dev/null 2>&1; then
    CXX_BIN="$(command -v g++-11)"; CC_BIN="$(command -v gcc-11)"
  else
    CXX_BIN="$(command -v g++)"; CC_BIN="$(command -v gcc)"
  fi
fi

CMAKE_BIN="${CMAKE_BIN:-$(command -v cmake)}"
version_ge() {  # version_ge A B -> true if A >= B
  [[ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -1)" == "$2" ]]
}
CMAKE_VER="$("${CMAKE_BIN}" --version | head -1 | awk '{print $3}')"
if ! version_ge "${CMAKE_VER}" "3.25"; then
  if [[ -x "${HOME}/.local/bin/cmake" ]]; then
    CMAKE_BIN="${HOME}/.local/bin/cmake"
    CMAKE_VER="$("${CMAKE_BIN}" --version | head -1 | awk '{print $3}')"
  fi
  if ! version_ge "${CMAKE_VER}" "3.25"; then
    echo "ERROR: CMake >= 3.25 is required (found ${CMAKE_VER})."
    echo "       pip3 install --user 'cmake==3.28.3'"
    exit 1
  fi
fi

# catkin_make invokes "cmake" from PATH, so make sure the new one wins.
export PATH="$(dirname "${CMAKE_BIN}"):${PATH}"

mkdir -p "${WS_DIR}"
cd "${WS_DIR}"
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash

CXX_MAJOR="$("${CXX_BIN}" -dumpversion | cut -d. -f1)"
EXTRA_CXX_FLAGS=""
EXTRA_C_FLAGS=""
if [[ "${CXX_MAJOR}" == "10" ]]; then
  # A locally extracted gcc-10 (see README) needs its support directories spelled out.
  EXTRA_C_FLAGS="-B/usr/lib/gcc/x86_64-linux-gnu/10"
  EXTRA_CXX_FLAGS="${EXTRA_C_FLAGS} -isystem /usr/include/c++/10 -isystem /usr/include/x86_64-linux-gnu/c++/10"
fi

"${CMAKE_BIN}" --version | head -1
"${CXX_BIN}" --version | head -1

# The EKF template instantiations are memory hungry; cap the default parallel
# build to avoid OOM kills on machines with many cores but limited RAM.
JOBS="${JOBS:-$(nproc)}"
if (( JOBS > 8 )); then JOBS=8; fi
echo "Building with -j${JOBS} (override with JOBS=...)"

catkin_make --source "${SRC_ROOT}" -j"${JOBS}" --only-pkg-with-deps farlio_odometry \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="${CC_BIN}" \
    -DCMAKE_CXX_COMPILER="${CXX_BIN}" \
    -DCMAKE_C_FLAGS="${EXTRA_C_FLAGS}" \
    -DCMAKE_CXX_FLAGS="${EXTRA_CXX_FLAGS}" \
    -DCMAKE_MAKE_PROGRAM="$(command -v make)"

echo
echo "Build finished. Source the overlay with:"
echo "  source ${WS_DIR}/devel/setup.bash"
