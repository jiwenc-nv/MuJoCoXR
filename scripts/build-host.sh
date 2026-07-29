#!/bin/sh
# Build the host tools and run the gates. This is the fast correctness loop:
# ~1 s incremental, and the ONLY place the bitwise golden trace is checked.
#
# Usage: build-host.sh [--no-gates]
# Env:   BUILD_DIR (default build), BUILD_TYPE (default Release)
#
# --no-gates is for when you want the binaries and the gates are legitimately
# red. build-web.sh takes the same flag with the same meaning; the Android
# script has none, because its gates need a device.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR=${BUILD_DIR:-build}
BUILD_TYPE=${BUILD_TYPE:-Release}
run_gates=1

while [ $# -gt 0 ]; do
  case $1 in
    --no-gates) run_gates=0 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "build-host.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
  shift
done

command -v cmake >/dev/null || {
  echo "build-host.sh: cmake — apt install cmake" >&2; exit 1; }

"$ROOT/scripts/fetch-menagerie.sh"

cd "$ROOT"
cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
cmake --build "$BUILD_DIR" --parallel

echo
echo "built: $BUILD_DIR/{teleop_replay,baseline,testspeed,ik_prototype}"

[ $run_gates -eq 1 ] || exit 0

echo
fail=0

# The cross-architecture dynamics invariant. Franka-only on purpose:
# bench/baseline.cc refuses nu < 8 with a stated reason, and generalising it
# would mean re-recording the one reference this repo has that claims host,
# wasm and aarch64 agree.
printf 'invariant franka: '
if ./"$BUILD_DIR"/baseline third_party/menagerie/franka_emika_panda/scene.xml \
     --ref baselines/host-x86_64.txt >/dev/null 2>&1; then
  echo "PASS"
else
  echo "FAIL"; fail=1
fi

# One teleop gate per staged scene. The ids are the STAGED DIRECTORY NAMES,
# not a list kept here — CMake derives those from mxr_stage_scene(), which
# derives them from src/robot_spec.c. The README used to spell them out, which
# made it a copy of the id list that docs/adding-a-robot.md does not mention
# and nothing checks.
#
# THIS IS THE ONLY PLACE THE BITWISE LOCK RUNS. teleop_replay compares
# trace_fnv1a only on the architecture the reference was recorded on and
# prints "trace comparison SKIPPED" everywhere else, so the wasm gate checks
# 15 of 16 and this checks 16 — the missing one being exactly "did this change
# alter the trajectory at all".
for scene_xml in "$BUILD_DIR"/*/ar_scene.xml; do
  [ -f "$scene_xml" ] || continue
  id=$(basename "$(dirname "$scene_xml")")
  ref=baselines/teleop-$id-host-x86_64.txt
  if [ ! -f "$ref" ]; then
    echo "teleop $id: SKIPPED — no $ref"
    continue
  fi
  printf 'teleop %s: ' "$id"
  # Exit code, not a grep: every diagnostic including `replay_check = PASS`
  # goes to stderr so stdout stays exactly the recordable block.
  log=$(mktemp)
  if ./"$BUILD_DIR"/teleop_replay "$scene_xml" --ref "$ref" >/dev/null 2>"$log"; then
    echo "PASS"
  else
    echo "FAIL"
    grep -E '\[FAIL\]|^replay_check|^[a-z_]+ (changed|differs)' "$log" >&2 || cat "$log" >&2
    fail=1
  fi
  rm -f "$log"
done

# host/ik_prototype is deliberately NOT run. It is a bring-up tool, not a
# gate: its waypoints are ±0.10 m TCP offsets tuned to the Franka, which sit
# outside the SO-101's orientation-constrained reach, so it exits non-zero on
# that scene by construction. Running it here would report a failure that
# means nothing. Invoke it by hand against one scene when tuning IK.

[ $fail -eq 0 ] || { echo; echo "build-host.sh: a gate failed" >&2; exit 1; }
echo
echo "all gates PASS"
