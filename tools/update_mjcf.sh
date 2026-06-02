#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

URDF="${1:-$ROOT_DIR/model/urdf/claptrap.urdf}"
GENERATED="${2:-$ROOT_DIR/model/xml/claptrap.generated.xml}"
FLOATING_GENERATED="${3:-$ROOT_DIR/model/xml/claptrap.floating.generated.xml}"
FINAL="${4:-$ROOT_DIR/model/xml/claptrap.xml}"
FLOATING_FINAL="$ROOT_DIR/model/xml/claptrap_floating.xml"
SCENE="$ROOT_DIR/model/xml/claptrap_scene.xml"
CHECK_MJB="${TMPDIR:-/tmp}/claptrap_mjcf_check_$$.mjb"

cleanup() {
  rm -f "$CHECK_MJB"
}
trap cleanup EXIT

validate_mjcf() {
  local xml="$1"
  local label="$2"

  rm -f "$CHECK_MJB"
  "$ROOT_DIR/mujoco/build/bin/compile" "$xml" "$CHECK_MJB" >/dev/null
  echo "Validated $label: $xml"
}

"$SCRIPT_DIR/urdf2mjcf" "$URDF" "$GENERATED"
python3 "$SCRIPT_DIR/add_imu_sites.py" "$GENERATED"
"$SCRIPT_DIR/make_floating_mjcf.py" "$GENERATED" "$FLOATING_GENERATED"

echo "Updated generated MJCF: $GENERATED"
echo "Updated floating MJCF: $FLOATING_GENERATED"
validate_mjcf "$FINAL" "final MJCF"
validate_mjcf "$FLOATING_FINAL" "floating MJCF"
validate_mjcf "$SCENE" "scene MJCF"
