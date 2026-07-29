#!/bin/sh
# Build the WebXR app into build-web/ and run one wasm gate per staged scene.
#
# Usage: build-web.sh [--no-gates]
# Env:   EMSDK (default ~/emsdk), BUILD_DIR (default build-web),
#        BUILD_TYPE (default Release)
#
# --no-gates is for when you want an artifact to poke at and the gates are
# legitimately red. Measured, they cost 1.4 s against a 2.1 s no-op build and
# ~27 s for a real one, so there is no case for skipping them to save time.
#
# Serving is NOT here: over adb it is `cd build-web && python3 -m
# http.server 8000`, and wrapping a one-liner would be a second way to do what
# scripts/serve-web-tls.py's header already says it deliberately is not ("a
# script rather than a one-liner only because generating a cert and wrapping
# the socket cannot be expressed as one"). See README, "On a headset".
#
# What this encodes beyond the raw cmake lines: activating emsdk from a
# POSIX shell, the pinned-version warning, named-targets-only, and reading
# the gate verdict off the exit code. Each is commented at its site.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR=${BUILD_DIR:-build-web}
BUILD_TYPE=${BUILD_TYPE:-Release}
EMSDK=${EMSDK:-$HOME/emsdk}
run_gates=1

while [ $# -gt 0 ]; do
  case $1 in
    --no-gates) run_gates=0 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "build-web.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
  shift
done

# ---- preflight -------------------------------------------------------------
# Every missing prerequisite is reported in ONE pass. The alternative — dying
# on the first — costs a full configure per missing tool, and on a fresh
# machine there are usually two or three.
missing=""
[ -f "$EMSDK/emsdk_env.sh" ] || missing="$missing
  emsdk not found at $EMSDK
    git clone https://github.com/emscripten-core/emsdk \$HOME/emsdk
    cd \$HOME/emsdk && ./emsdk install 4.0.10 && ./emsdk activate 4.0.10
    (or set EMSDK=/path/to/emsdk)"
command -v cmake >/dev/null || missing="$missing
  cmake — apt install cmake"
# node is not checked: it ships inside emsdk, so the emsdk entry above
# already covers the only thing the gates need beyond the build.
if [ -n "$missing" ]; then
  echo "build-web.sh: missing prerequisites:$missing" >&2
  exit 1
fi

# emcc is not on PATH in a fresh shell even when emsdk is installed; sourcing
# emsdk_env.sh is what puts it there — and it also puts emsdk's own node on
# PATH, which is what runs the wasm gates below. Do it only if needed, so an
# already activated shell is left alone.
#
# THE `cd` IS REQUIRED, not tidiness. emsdk_env.sh locates itself through
# $BASH_SOURCE / $ZSH_NAME, and this script is POSIX sh where neither exists;
# sourced from anywhere else it prints "unable to determine 'emsdk'
# directory" and returns having set nothing. Its own error text names the cwd
# as the fix. The subshell keeps the cd from leaking into the build below.
if ! command -v emcmake >/dev/null; then
  # shellcheck disable=SC1091
  cd "$EMSDK" && . ./emsdk_env.sh >/dev/null 2>&1
  cd "$ROOT"
fi
command -v emcmake >/dev/null || {
  echo "build-web.sh: sourced $EMSDK/emsdk_env.sh but emcmake is still not on PATH" >&2
  echo "  the sdk is probably installed but not activated:" >&2
  echo "  cd $EMSDK && ./emsdk activate 4.0.10" >&2
  exit 1; }

# THE PIN IS LOAD-BEARING, not hygiene — docs/validation-web.md records which
# emsdk versions miscompile this target. A mismatch is a warning rather than
# an error because a newer sdk usually works and blocking would be worse than
# saying so.
want_emcc=4.0.10
have_emcc=$(emcc --version 2>/dev/null | head -1 | sed -n 's/.*clang-like replacement + linker emulating GNU ld) \([0-9.]*\).*/\1/p')
[ "$have_emcc" = "$want_emcc" ] || echo "build-web.sh: warning: emcc is ${have_emcc:-unknown}, this tree pins $want_emcc (see docs/validation-web.md)" >&2

# ---- scenes ----------------------------------------------------------------
# Idempotent and self-checking: it early-exits when every scene is already
# fetched and widens the sparse checkout when one is not.
"$ROOT/scripts/fetch-menagerie.sh"

# ---- configure -------------------------------------------------------------
cd "$ROOT"
# An emsdk upgrade leaves a cache pointing at a compiler that no longer
# exists, and cmake's error for that names a path rather than the cause.
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
  cached=$(sed -n 's/^CMAKE_CXX_COMPILER:[^=]*=//p' "$BUILD_DIR/CMakeCache.txt")
  if [ -n "$cached" ] && [ ! -x "$cached" ]; then
    echo "build-web.sh: cached compiler $cached is gone (emsdk moved or upgraded); reconfiguring from scratch"
    rm -rf "$BUILD_DIR"
  fi
fi
emcmake cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

# ---- build -----------------------------------------------------------------
# NAMED TARGETS ONLY, NEVER `all`. Upstream MuJoCo's add_subdirectory(wasm) is
# unconditional under Emscripten and points CMAKE_RUNTIME_OUTPUT_DIRECTORY at
# ${CMAKE_SOURCE_DIR}/wasm/dist — which, under FetchContent, is a wasm/dist
# inside THIS repo. Building `all` also drags in a target that needs `tsc`,
# which is not a dependency of anything here. CMakeLists.txt says the same
# thing at its wasm block; this script is the other place it has to be true.
cmake --build "$BUILD_DIR" --parallel --target mxr baseline teleop_replay

echo
echo "built: $BUILD_DIR/mxr.js, mxr.wasm, mxr.data (+ baseline, teleop_replay)"

# ---- gates -----------------------------------------------------------------
if [ $run_gates -eq 1 ]; then
  echo
  # The scene ids are the STAGED DIRECTORY NAMES, not a list kept here. CMake
  # derives those from mxr_stage_scene(), which derives them from
  # src/robot_spec.c. Hardcoding them would add a copy of the id list that
  # nothing checks — the failure src/robot_spec.h warns about.
  fail=0
  for scene_xml in "$BUILD_DIR"/*/ar_scene.xml; do
    [ -f "$scene_xml" ] || continue
    id=$(basename "$(dirname "$scene_xml")")
    ref=baselines/teleop-$id-host-x86_64.txt
    if [ ! -f "$ROOT/$ref" ]; then
      echo "gate $id: SKIPPED — no $ref"
      continue
    fi
    printf 'gate %s: ' "$id"
    # THE EXIT CODE IS THE VERDICT, not a grep. bench/teleop_replay.cc puts
    # every diagnostic — including the `replay_check = PASS` line — on
    # STDERR, because stdout is exactly the recordable block that
    # `teleop_replay ... > baselines/...` captures. Grepping stdout for the
    # verdict finds nothing and reads as a failure; grepping stderr would
    # work but re-derives what `return g_failures == 0 ? 0 : 1` already says.
    log=$(mktemp)
    if (cd "$BUILD_DIR" && node teleop_replay.js "/$id/ar_scene.xml" --ref "/$ref") \
        >/dev/null 2>"$log"; then
      echo "PASS"
    else
      echo "FAIL"
      # Only the check lines and the verdict — the trace block is thousands
      # of lines and none of it says what went wrong.
      grep -E '\[FAIL\]|^replay_check|^[a-z_]+ (changed|differs)' "$log" >&2 || cat "$log" >&2
      fail=1
    fi
    rm -f "$log"
  done
  # The bitwise trace hash is skipped off its recording architecture by
  # design, so a wasm PASS is the model-independent half of the gate: frame
  # conventions, engage jump, slew compliance, the census, the solver config
  # block and pos_med. See bench/teleop_replay.cc.
  [ $fail -eq 0 ] || { echo "build-web.sh: a gate failed" >&2; exit 1; }
fi
