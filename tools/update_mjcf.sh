#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

URDF="${1:-$ROOT_DIR/model/urdf/claptrap.urdf}"
GENERATED="${2:-$ROOT_DIR/model/xml/claptrap.generated.xml}"
FINAL="${3:-$ROOT_DIR/model/xml/claptrap.xml}"
CHECK_MJB="${TMPDIR:-/tmp}/claptrap_mjcf_check.mjb"

"$SCRIPT_DIR/urdf2mjcf" "$URDF" "$GENERATED"

rm -f "$CHECK_MJB"
"$ROOT_DIR/mujoco/build/bin/compile" "$FINAL" "$CHECK_MJB" >/dev/null

echo "Updated generated MJCF: $GENERATED"
echo "Validated final MJCF: $FINAL"
