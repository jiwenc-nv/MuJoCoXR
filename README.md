# MuJoCoXR

MuJoCo physics running **fully on-device** on a standalone Android XR headset
(Quest-class, arm64-v8a), with the Franka Emika Panda teleoperated by an XR
controller. Raw OpenXR + Vulkan + the MuJoCo C API — no game engine, no
Python, no desktop in the loop.

## Features

- **On-device physics** — unmodified upstream MuJoCo v3.10.0 (fetched and
  pinned at build time, zero source patches) compiled GL-free via the
  filament-mjr-compat flag set; the unmodified MuJoCo Menagerie Franka scene
  steps at 500 Hz on the headset CPU.
- **Raw OpenXR shell** — `NativeActivity` + Khronos loader:
  `XR_KHR_vulkan_enable2` device handshake, `LOCAL_FLOOR` reference space
  (STAGE/LOCAL fallbacks), full session lifecycle, Touch-controller action
  set (grip pose, trigger, squeeze, A).
- **Scene-specific Vulkan renderer** — consumes the renderer-agnostic
  `mjvScene`: meshes de-indexed at load by welding (vertex, normal) index
  pairs, one pipeline with 128-byte push constants, per-eye stereo passes
  driven exclusively by `XrView` pose/fov, procedural checker floor, one
  directional light.
- **Clutched teleop** — squeeze-hold clutch latches controller→target
  offsets in MuJoCo world coordinates (zero jump on engage, by
  construction); damped-least-squares IK (6D task, 7-DOF arm) with a
  nullspace home-posture bias and gravity feed-forward writes the position
  servos; targets rate-limited at 1.5 m/s / 3 rad/s; trigger drives the
  gripper (inverted 0–255 range, 255 = open); A resets to the home
  keyframe; recentering auto-disengages the clutch.
- **Owned frame conventions** — `app/frames.h` is the single owner of the
  OpenXR↔MuJoCo mapping (`q_ws = (0.5, 0.5, −0.5, −0.5)` wxyz; xyzw↔wxyz
  quaternion reordering at every crossing), with an in-headset world-axes
  gizmo to verify handedness per axis.
- **Cross-platform determinism harness** — a deterministic 2 s control
  excitation with a recorded host reference (`baselines/`); the same binary
  replays it on-device and checks final `qpos` (L∞) and energy, so a broken
  port can't pass by merely holding pose.
- **CLI benchmark** — a verbatim copy of upstream `sample/testspeed.cc` for
  measuring `mj_step` cost over adb before trusting the frame budget.
- **Gradle-less packaging** — one shell script builds a signed APK with
  `aapt2`/`zipalign`/`apksigner`; scene assets flow APK `assets/` →
  `AAssetManager` → MuJoCo VFS.

## Requirements

- CMake ≥ 3.16, a C/C++17 toolchain, network access at configure time
  (MuJoCo, OpenXR SDK and their dependencies are fetched and pinned)
- Android NDK r26+ and `glslangValidator` (Debian/Ubuntu: `glslang-tools`)
  for the headset build
- Android SDK build-tools + platform jar + a JDK for APK packaging
- git (for `scripts/fetch-menagerie.sh`)

## Build & run

Fetch the Franka scene (sparse, pinned; used by host tools and the APK):

```
scripts/fetch-menagerie.sh
```

Host tools (IK prototype + baseline recorder):

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/ik_prototype third_party/menagerie/franka_emika_panda/scene.xml
./build/baseline     third_party/menagerie/franka_emika_panda/scene.xml
```

Headset app:

```
cmake -S . -B build-android \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L"
cmake --build build-android --parallel
android/package-apk.sh build-android mujocoxr.apk
adb install -r mujocoxr.apk
adb logcat -s mujocoxr
```

An existing MuJoCo checkout can be used instead of the download with
`-DMUJOCOXR_MUJOCO_DIR=/path/to/mujoco` (must be v3.10.0).

In the headset: squeeze to clutch the green target marker to your hand,
trigger to close the gripper, A to reset the scene. Follow
[docs/on-device-validation.md](docs/on-device-validation.md) for the ordered
bring-up gates (benchmark → handedness → teleop acceptance → soak).

## Layout

- `src/` — DLS IK solver, target rate limiter, MuJoCo error hooks (shared
  by host tools and the app)
- `app/` — the XR app: OpenXR shell, Vulkan context, `mjvScene` renderer,
  frame conventions, clutched teleop, asset loading, GLSL shaders
- `host/` — host-side IK prototype (convergence + nullspace checks)
- `bench/` — invariant baseline recorder/checker + `testspeed` benchmark
- `baselines/` — recorded host reference for the on-device invariant check
- `android/` — AndroidManifest + APK packaging script
- `scripts/` — Menagerie scene fetcher (sparse, pinned)
- `docs/` — on-device validation guide

## License notes

MuJoCo and the MuJoCo Menagerie Franka model are Apache-2.0; the packaging
script ships the model's LICENSE inside the APK.
