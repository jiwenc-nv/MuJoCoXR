#!/bin/sh
# Package the MuJoCoXR APK from an arm64 CMake build dir (no Gradle):
# aapt2 link -> zip native libs + flattened Franka assets -> zipalign ->
# apksigner (auto-generated debug keystore).
#
# Usage: package-apk.sh <android-build-dir> [output.apk]
# Env: ANDROID_SDK (default ~/Android/sdk), ANDROID_NDK (default r27c path),
#      MENAGERIE (default third_party/menagerie — run scripts/fetch-menagerie.sh)
set -eu

BUILD=${1:?usage: package-apk.sh <android-build-dir> [output.apk]}
OUT=$(realpath -m "${2:-mujocoxr.apk}")
SDK=${ANDROID_SDK:-$HOME/Android/sdk}
NDK=${ANDROID_NDK:-$HOME/Android/ndk/android-ndk-r27c}
MENAGERIE=${MENAGERIE:-$(cd "$(dirname "$0")/.." && pwd)/third_party/menagerie}
if [ ! -f "$MENAGERIE/franka_emika_panda/scene.xml" ]; then
  echo "Franka scene not found at $MENAGERIE — run scripts/fetch-menagerie.sh" >&2
  exit 1
fi
BT=$SDK/build-tools/35.0.0
PLATFORM=$SDK/platforms/android-32/android.jar
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

# Native libs: app + shared MuJoCo (decoder plugins self-register only in the
# shared layout) + OpenXR loader + shared STL (required: two STL-using .so's).
mkdir -p "$WORK/lib/arm64-v8a"
cp "$BUILD/libmujocoxr.so" "$WORK/lib/arm64-v8a/"
cp "$BUILD/lib/libmujoco.so" "$WORK/lib/arm64-v8a/"
find "$BUILD/_deps/openxr-build" -name libopenxr_loader.so \
  -exec cp {} "$WORK/lib/arm64-v8a/" \;
cp "$NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so" \
  "$WORK/lib/arm64-v8a/"

# Assets, flattened: VFS keys are basename-stripped and lowercased, so the
# meshdir prefix is irrelevant; the 67 files have unique basenames.
mkdir -p "$WORK/assets/franka"
cp "$MENAGERIE/franka_emika_panda/scene.xml" \
   "$MENAGERIE/franka_emika_panda/panda.xml" \
   "$MENAGERIE/franka_emika_panda/assets/"* "$WORK/assets/franka/"
# Menagerie NOTICE packaging: the Franka model is Apache-2.0.
cp "$MENAGERIE/franka_emika_panda/LICENSE" "$WORK/assets/franka/LICENSE"

"$BT/aapt2" link -o "$WORK/base.apk" --manifest "$HERE/AndroidManifest.xml" \
  -I "$PLATFORM" --min-sdk-version 29 --target-sdk-version 32

(cd "$WORK" && zip -qr base.apk lib assets)
"$BT/zipalign" -f 4 "$WORK/base.apk" "$WORK/aligned.apk"

KS=$HOME/.android/debug.keystore
if [ ! -f "$KS" ]; then
  mkdir -p "$(dirname "$KS")"
  keytool -genkeypair -keystore "$KS" -storepass android \
    -alias androiddebugkey -keypass android -keyalg RSA -keysize 2048 \
    -validity 10000 -dname "CN=Android Debug,O=Android,C=US"
fi
"$BT/apksigner" sign --ks "$KS" --ks-pass pass:android \
  --ks-key-alias androiddebugkey --key-pass pass:android \
  --out "$OUT" "$WORK/aligned.apk"

echo "APK: $OUT"
echo "Install: adb install -r $OUT"
echo "Logs:    adb logcat -s mujocoxr"
