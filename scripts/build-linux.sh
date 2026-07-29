#!/bin/sh
# Build the Linux OpenXR client into build-linux/. Runs against any desktop
# OpenXR runtime; the one this was written for is NVIDIA CloudXR, which streams
# to a headset over the network.
#
# Usage: build-linux.sh
# Env:   BUILD_DIR (default build-linux), BUILD_TYPE (default Release)
#
# NO --no-checks FLAG, because there are no checks here to skip — the same
# reason build-android.sh has none. Everything this target can be checked on
# without hardware is either already covered by build-host.sh (which builds the
# identical mxr_core from the identical source list) or needs a running
# CloudXR service, which a build script has no business requiring. The one
# headless check that IS worth running is `build-linux/mujocoxr --probe`, and
# it lives in docs/validation-linux.md, as the half of gate 2 that needs no
# headset — not as a gate number of its own, because the gate numbers mean the
# same thing on every target.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR=${BUILD_DIR:-build-linux}
BUILD_TYPE=${BUILD_TYPE:-Release}

while [ $# -gt 0 ]; do
  case $1 in
    -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "build-linux.sh: unknown argument '$1'" >&2; exit 2 ;;
  esac
  shift
done

# ---- preflight -------------------------------------------------------------
# All of it in ONE pass, same as build-android.sh: a fresh box is missing two
# or three of these and failing on the first would cost a configure per gap.
# Each entry names the fix, not just the gap.
missing=""
command -v cmake >/dev/null || missing="$missing
  cmake — apt install cmake"

# Same tool and same reason as the Android build: the SPIR-V C headers are
# generated with `-V --vn <symbol>`, which shaderc's glslc cannot emit.
command -v glslangValidator >/dev/null || missing="$missing
  glslangValidator — apt install glslang-tools"

pkg-config --exists vulkan 2>/dev/null || [ -f /usr/include/vulkan/vulkan.h ] \
  || missing="$missing
  Vulkan headers/loader — apt install libvulkan-dev
    (CloudXR advertises no OpenGL graphics binding, so Vulkan is not a
     preference here: XR_KHR_vulkan_enable2 is the only way in.)"

# NOT USED BY THIS APP, AND STILL REQUIRED — which is why it gets a sentence
# rather than a word. The fetched OpenXR SDK includes src/cmake/presentation.cmake
# unconditionally on Linux; PRESENTATION_BACKEND defaults to `xlib`, and that
# file raises a FATAL_ERROR if the Xlib headers are absent. The loader itself
# links only Threads. We open no window, so this is a configure-time cost only.
pkg-config --exists x11 2>/dev/null || [ -f /usr/include/X11/Xlib.h ] \
  || missing="$missing
  X11 headers — apt install libx11-dev
    (not used at run time: the fetched OpenXR SDK's presentation.cmake
     hard-fails at configure without them, even though this app opens no
     window. See the XCB/Wayland comment in CMakeLists.txt.)"

if [ -n "$missing" ]; then
  echo "build-linux.sh: missing prerequisites:$missing" >&2
  exit 1
fi

# ---- scenes ----------------------------------------------------------------
"$ROOT/scripts/fetch-menagerie.sh"

# ---- configure -------------------------------------------------------------
cd "$ROOT"
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DMUJOCOXR_BUILD_LINUX_XR=ON

# ---- build -----------------------------------------------------------------
# NAMED TARGET ONLY, never `all` — same rule as the other two scripts. Here it
# also keeps the fetched OpenXR SDK's own extra targets out of the build.
cmake --build "$BUILD_DIR" --parallel --target mujocoxr

echo
echo "built: $BUILD_DIR/mujocoxr"
echo
echo "The runtime is a HOST-LEVEL SINGLETON and this program never starts it."
echo "In another terminal:"
echo "  cd /code && scripts/run_cloudxr_runtime.sh"
echo "Two variables that script gets wrong or leaves for you, both of which"
echo "surface as XR_ERROR_RUNTIME_UNAVAILABLE (-51) and nothing else:"
echo "  XR_RUNTIME_JSON=\$HOME/.cloudxr/openxr_cloudxr.json   (its own default"
echo "                                                        does not exist)"
echo "  NV_CXR_RUNTIME_DIR=\$HOME/.cloudxr/run                (must match the"
echo "                                                        service) Then:"
echo "  $BUILD_DIR/mujocoxr --probe          # no headset needed"
echo "  $BUILD_DIR/mujocoxr --scene franka"
echo
echo "Bring-up gates: docs/validation-linux.md"
