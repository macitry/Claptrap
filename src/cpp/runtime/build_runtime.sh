#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

g++ "$SCRIPT_DIR/apps/robot_runtime.cc" \
  "$SCRIPT_DIR/core/plugin_manager.cc" \
  "$SCRIPT_DIR/core/plugin_registry.cc" \
  "$SCRIPT_DIR/core/runtime_config.cc" \
  "$SCRIPT_DIR/plugins/controller/controller_plugin.cc" \
  "$SCRIPT_DIR/plugins/estimator/estimator_plugin.cc" \
  "$SCRIPT_DIR/plugins/sim/sim_plugin.cc" \
  -std=c++17 \
  -O2 \
  -Wall \
  -Wextra \
  -I"$SCRIPT_DIR" \
  -pthread \
  -o "$SCRIPT_DIR/robot_runtime"

echo "Built $SCRIPT_DIR/robot_runtime"
