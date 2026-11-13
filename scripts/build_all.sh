#!/bin/bash
# Build every program in this repo (CLI tools + Qt GUI). Usage: scripts/build_all.sh [DRCF_VERSION]
# Controller 2.10.1 (GF02100101) -> DRCF_VERSION 2 (default)
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
VER="${1:-2}"
cd "$REPO"
for f in tests/connect_test.cpp tests/flange_io_status.cpp tests/egp_gripper_test.cpp; do
  ./build.sh "$f" "$VER"
done
cmake -S m0609_gui -B m0609_gui/build -DDRCF_VERSION="$VER" ${DRFL_ROOT:+-DDRFL_ROOT="$DRFL_ROOT"} >/dev/null
cmake --build m0609_gui/build -j
m0609_gui/build/pose_math_test   # offline check of the MoveL pose math (RPY <-> ZYZ)
echo "GUI: m0609_gui/build/m0609_gui"
