#!/bin/sh
# Package the MuJoCoXR APK from an arm64 CMake build dir (no Gradle):
# aapt2 link -> zip native libs + one flattened asset dir per scene ->
# zipalign -> apksigner (auto-generated debug keystore).
#
# Usage: package-apk.sh <android-build-dir> [output.apk]
# Env: ANDROID_SDK (default ~/Android/sdk), ANDROID_NDK (default r27c path),
#      MENAGERIE (default third_party/menagerie — run scripts/fetch-menagerie.sh)
set -eu

BUILD=${1:?usage: package-apk.sh <android-build-dir> [output.apk]}
OUT=$(realpath -m "${2:-mujocoxr.apk}")
SDK=${ANDROID_SDK:-$HOME/Android/sdk}
NDK=${ANDROID_NDK:-$HOME/Android/ndk/android-ndk-r27c}
MENAGERIE=${MENAGERIE:-$(cd "$(dirname "$0")/../.." && pwd)/third_party/menagerie}
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

# Assets, one dir per scene id, each flattened: VFS keys are basename-stripped
# and lowercased, so the meshdir prefix is irrelevant. app/android/assets.cc
# builds ONE VFS from ONE dir, so basenames only have to be unique WITHIN a
# scene (they are: 67 for the Franka, 19 for the SO101) — the two dirs are
# free to both contain an ar_scene.xml and a LICENSE, and do.
#
# ADDING A ROBOT: one `stage_scene` line, and nothing else in this file. The
# id must match MxrScene::id in src/robot_spec.c and the mxr_stage_scene() call
# in CMakeLists.txt — see the note above that call for which of the three is
# authoritative.
#
# The fetch guard lives INSIDE this function on purpose. It used to be a
# separate `for d in <menagerie dirs>` loop up top, which made the Menagerie
# directory list a second hardcoded copy in this one file — so a third robot
# added the way this comment describes would have skipped the guard and failed
# later, at `cp`, with a message about a missing file instead of a message
# about a missing fetch.
stage_scene() {  # <id> <menagerie_dir> [extra menagerie file...]
  id=$1; dir=$2; shift 2
  [ -d "$MENAGERIE/$dir" ] || {
    echo "$dir not found at $MENAGERIE — run scripts/fetch-menagerie.sh" >&2
    exit 1
  }
  mkdir -p "$WORK/assets/$id"
  cp "$MENAGERIE/$dir/assets/"* "$WORK/assets/$id/"
  for f in "$@"; do
    cp "$MENAGERIE/$dir/$f" "$WORK/assets/$id/"
  done
  # The AR composition (robot + table, no skybox/floor) — what the app loads.
  cp "$HERE/../../assets/$id/ar_scene.xml" "$WORK/assets/$id/"
  # Menagerie NOTICE packaging: both models are Apache-2.0.
  cp "$MENAGERIE/$dir/LICENSE" "$WORK/assets/$id/LICENSE"
}
# Only what ar_scene.xml <include>s. Menagerie's own scene.xml is NOT staged:
# it adds a skybox and a ground plane, which is exactly what the AR wrapper
# exists to leave out, and assets.cc compiles ar_scene.xml and nothing else.
stage_scene franka franka_emika_panda panda.xml
stage_scene so101 robotstudio_so101 so101.xml

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
