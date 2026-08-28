#!/usr/bin/env bash
# Onboard PC dependency installation script: requires RBDL (+urdfreader addon)
# Skips each step if already installed (idempotent).
set -euo pipefail

RBDL_SRC_DIR="${RBDL_SRC_DIR:-$HOME/rbdl}"
RBDL_INSTALL_DIR="${RBDL_INSTALL_DIR:-$HOME/rbdl-install}"

echo "== Installing apt dependencies =="
sudo apt update
sudo apt install -y cmake libeigen3-dev ros-humble-dynamixel-sdk

if [ -f "${RBDL_INSTALL_DIR}/include/rbdl/rbdl.h" ] && [ -f "${RBDL_INSTALL_DIR}/include/rbdl/addons/urdfreader/urdfreader.h" ]; then
  echo "== RBDL is already installed at ${RBDL_INSTALL_DIR}, skipping build =="
else
  echo "== Preparing RBDL source =="
  if [ -d "${RBDL_SRC_DIR}/.git" ]; then
    echo "Using existing clone: ${RBDL_SRC_DIR}"
  else
    git clone https://github.com/rbdl/rbdl.git "${RBDL_SRC_DIR}"
  fi
  cd "${RBDL_SRC_DIR}"
  git submodule update --init --recursive

  echo "== Building and installing RBDL =="
  mkdir -p build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${RBDL_INSTALL_DIR}" \
    -DRBDL_BUILD_ADDON_URDFREADER=ON \
    -DRBDL_BUILD_STATIC=OFF
  make -j"$(nproc)"
  make install
fi

echo "== Done: RBDL_DIR=${RBDL_INSTALL_DIR} =="
echo "Build with: colcon build --cmake-args -DRBDL_DIR=${RBDL_INSTALL_DIR}"
