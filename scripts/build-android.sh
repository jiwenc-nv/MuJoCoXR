#!/bin/sh
# Cross-compile the Android/OpenXR app for arm64-v8a into build-android/ and
# package a signed APK.
#
# Usage: build-android.sh [--no-apk] [--install] [out.apk]
# Env:   ANDROID_NDK (default ~/Android/ndk/android-ndk-r27c)
#        ANDROID_SDK (default ~/Android/sdk)
#        BUILD_DIR (default build-android), BUILD_TYPE (default Release)
#
# --no-apk stops after the build. That is the configuration validation gate 1
# uses: it pushes build-android/{baseline,testspeed,teleop_replay} over adb
# and never needs an APK.
set -eu

ROOT=$(cd "$(dirname "$0")/.." && pwd)
BUILD_DIR=${BUILD_DIR:-build-android}
BUILD_TYPE=${BUILD_TYPE:-Release}
NDK=${ANDROID_NDK:-$HOME/Android/ndk/android-ndk-r27c}
SDK=${ANDROID_SDK:-$HOME/Android/sdk}
BT_VERSION=35.0.0
PLATFORM_VERSION=android-32
make_apk=1
install=0
out=""

while [ $# -gt 0 ]; do
  case $1 in
    --no-apk) make_apk=0 ;;
    --install) install=1 ;;
    -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "build-android.sh: unknown flag '$1'" >&2; exit 2 ;;
    *) out=$1 ;;
  esac
  shift
done
out=${out:-$ROOT/mujocoxr.apk}

# ---- preflight -------------------------------------------------------------
# All of it in ONE pass. This target has five independent prerequisites and
# nothing on a fresh machine has all five, so failing on the first would mean
# five edit-run cycles. Each entry names the fix, not just the gap.
missing=""
command -v cmake >/dev/null || missing="$missing
  cmake — apt install cmake"
[ -f "$NDK/build/cmake/android.toolchain.cmake" ] || missing="$missing
  Android NDK r26+ not found at $NDK
    sdkmanager --install 'ndk;27.2.12479018', or set ANDROID_NDK"

# glslangValidator compiles app/openxr/shaders/*.{vert,frag} to SPIR-V and
# CMakeLists.txt marks it REQUIRED in the shared OpenXR+Vulkan block — so it
# gates EVERY
# Android configure, including a --no-apk CLI-tools build that links no
# shader at all. The NDK ships shader-tools/*/glslc, which is shaderc and is
# NOT a drop-in: the build invokes `-V --vn <symbol>` to emit a C header, and
# glslc has no --vn. Install glslang rather than pointing at glslc.
command -v glslangValidator >/dev/null || missing="$missing
  glslangValidator — apt install glslang-tools
    (the NDK's shader-tools/glslc is shaderc, not glslang: no --vn, so it
     cannot emit the SPIR-V C headers this build consumes)"

if [ $make_apk -eq 1 ]; then
  [ -x "$SDK/build-tools/$BT_VERSION/aapt2" ] || missing="$missing
  Android SDK build-tools $BT_VERSION not found under $SDK
    sdkmanager --install 'build-tools;$BT_VERSION', or set ANDROID_SDK"
  [ -f "$SDK/platforms/$PLATFORM_VERSION/android.jar" ] || missing="$missing
  Android platform $PLATFORM_VERSION not found under $SDK
    sdkmanager --install 'platforms;$PLATFORM_VERSION'"
  command -v zip >/dev/null || missing="$missing
  zip — apt install zip  (app/android/package-apk.sh adds lib/ and assets/
    to the aapt2 output with it)"
  command -v keytool >/dev/null || missing="$missing
  keytool — apt install default-jdk  (generates the debug keystore; apksigner
    needs a JRE regardless)"
fi
[ $install -eq 0 ] || command -v adb >/dev/null || missing="$missing
  adb — apt install android-sdk-platform-tools  (needed by --install)"

if [ -n "$missing" ]; then
  echo "build-android.sh: missing prerequisites:$missing" >&2
  [ $make_apk -eq 1 ] && echo "
  (--no-apk skips the SDK, zip and keytool requirements and still builds the
   .so and the CLI tools that validation gate 1 pushes over adb.)" >&2
  exit 1
fi

# ---- scenes ----------------------------------------------------------------
"$ROOT/scripts/fetch-menagerie.sh"

# ---- configure -------------------------------------------------------------
cd "$ROOT"
# An NDK upgrade leaves a cache pointing at a compiler that no longer exists.
if [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
  cached=$(sed -n 's/^CMAKE_CXX_COMPILER:[^=]*=//p' "$BUILD_DIR/CMakeCache.txt")
  if [ -n "$cached" ] && [ ! -x "$cached" ]; then
    echo "build-android.sh: cached compiler $cached is gone (NDK moved or upgraded); reconfiguring from scratch"
    rm -rf "$BUILD_DIR"
  fi
fi
# -D_POSIX_C_SOURCE=200809L is REQUIRED, not tidiness: bionic gates
# localtime_r on it where glibc exposes it under _GNU_SOURCE, and upstream
# MuJoCo calls it. Flag-level only — this project never patches upstream.
cmake -S . -B "$BUILD_DIR" \
  -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared \
  -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
  -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L"

# ---- build -----------------------------------------------------------------
# `mujocoxr` is the app; the three CLI tools are what validation-android.md
# gate 1 pushes over adb. Named rather than `all` for the same reason the web
# build is: nothing here wants upstream's own sample and test targets.
cmake --build "$BUILD_DIR" --parallel \
  --target mujocoxr baseline testspeed teleop_replay

echo
echo "built: $BUILD_DIR/libmujocoxr.so (+ baseline, testspeed, teleop_replay)"

if [ $make_apk -eq 0 ]; then
  echo
  echo "no APK (--no-apk). To run the on-device gates, see docs/validation-android.md"
  exit 0
fi

# ---- package ---------------------------------------------------------------
# Delegated, not reimplemented: app/android/package-apk.sh owns the aapt2 ->
# zip -> zipalign -> apksigner sequence and the per-scene asset flattening,
# and a second copy of that would be a second thing to keep in step with the
# scene table.
ANDROID_SDK="$SDK" ANDROID_NDK="$NDK" \
  "$ROOT/app/android/package-apk.sh" "$BUILD_DIR" "$out"

if [ $install -eq 1 ]; then
  echo
  adb install -r "$out"
  echo "launch: adb shell am start -n com.nvidia.mujocoxr/android.app.NativeActivity"
  echo "logs:   adb logcat -s mujocoxr"
fi
