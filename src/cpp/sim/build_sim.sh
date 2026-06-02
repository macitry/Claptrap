#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

COMMON_FLAGS=(
  -std=c++17
  -O2
  -Wall
  -Wextra
  -I"$SCRIPT_DIR"
  -I"$ROOT_DIR/mujoco/include"
  -L"$ROOT_DIR/mujoco/build/lib"
  -Wl,-rpath,"$ROOT_DIR/mujoco/build/lib"
)

g++ "$SCRIPT_DIR/robot_shared_memory.cc" \
  "$SCRIPT_DIR/launch_robot_scene.cc" \
  "${COMMON_FLAGS[@]}" \
  -lmujoco \
  -lcrypto \
  -pthread \
  -lrt \
  -o "$SCRIPT_DIR/launch_robot_scene_cpp"

g++ "$SCRIPT_DIR/robot_shared_memory.cc" \
  "$SCRIPT_DIR/shared_memory_client.cc" \
  "${COMMON_FLAGS[@]}" \
  -lcrypto \
  -pthread \
  -lrt \
  -o "$SCRIPT_DIR/shared_memory_client_cpp"

echo "Built $SCRIPT_DIR/launch_robot_scene_cpp"
echo "Built $SCRIPT_DIR/shared_memory_client_cpp"
