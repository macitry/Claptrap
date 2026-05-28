#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

g++ "$SCRIPT_DIR/urdf2mjcf.cc" \
  -std=c++17 \
  -O2 \
  -I"$ROOT_DIR/mujoco/include" \
  -L"$ROOT_DIR/mujoco/build/lib" \
  -Wl,-rpath,"$ROOT_DIR/mujoco/build/lib" \
  -lmujoco \
  -o "$SCRIPT_DIR/urdf2mjcf"

echo "Built $SCRIPT_DIR/urdf2mjcf"
