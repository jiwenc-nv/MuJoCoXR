# MuJoCoXR

MuJoCo physics running **fully on-device** on a standalone XR headset
(Quest-class), with the Franka Emika Panda teleoperated by an XR controller.
The MuJoCo C API with two thin shells over one shared core: raw
OpenXR + Vulkan on Android, and WebXR + WebGL2 in the headset browser. No
game engine, no Python in the app, no desktop in the loop.

## Features

- **On-device physics** — unmodified upstream MuJoCo v3.10.0 (fetched and
  pinned at build time, zero source patches) compiled GL-free via the
  filament-mjr-compat flag set; the unmodified MuJoCo Menagerie Franka scene
  steps at 500 Hz on the headset CPU — composed onto a table in an AR
  scene (`assets/ar_scene.xml`) without touching the upstream files.
- **Raw OpenXR shell** — `NativeActivity` + Khronos loader:
  `XR_KHR_vulkan_enable2` device handshake, `LOCAL_FLOOR` reference space
  (STAGE/LOCAL fallbacks), full session lifecycle, Touch-controller action
  set (grip pose, trigger, squeeze, A).
- **AR passthrough** — alpha-blend environment mode when the runtime
  offers it (Pico/Quest-class MR): the scene renders over the camera feed;
  no skybox, no ground plane. The robot stands on a virtual table whose
  legs meet your physical floor.
- **Scene-specific Vulkan renderer** — consumes the renderer-agnostic
  `mjvScene`: meshes de-indexed at load by welding (vertex, normal) index
  pairs, one pipeline with 128-byte push constants, per-eye stereo passes
  driven exclusively by `XrView` pose/fov, one directional light.
- **Clutched teleop** — squeeze-hold clutch latches controller→target
  offsets in MuJoCo world coordinates (zero jump on engage, by
  construction); damped-least-squares IK (6D task, 7-DOF arm) with a
  nullspace home-posture bias and gravity feed-forward writes the position
  servos; targets rate-limited at 1.5 m/s / 3 rad/s; trigger drives the
  gripper (inverted 0–255 range, 255 = open); A resets to the home
  keyframe; recentering auto-disengages the clutch.
- **Owned frame conventions** — `src/frames.h` is the single owner of the
  XR↔MuJoCo mapping (`MXR_Q_MJ_FROM_XR = (0.5, 0.5, −0.5, −0.5)` wxyz;
  xyzw↔wxyz quaternion reordering at every crossing), with a world-axes gizmo
  to verify handedness per axis. The axis map is checked on every build by
  `bench/teleop_replay`, on host and wasm alike.
- **Cross-platform determinism harness** — a deterministic 2 s control
  excitation with a recorded host reference (`baselines/`); the same binary
  replays it on-device and checks final `qpos` (L∞) and energy, so a broken
  port can't pass by merely holding pose.
- **CLI benchmark** — a verbatim copy of upstream `sample/testspeed.cc` for
  measuring `mj_step` cost over adb before trusting the frame budget.
- **Gradle-less packaging** — one shell script builds a signed APK with
  `aapt2`/`zipalign`/`apksigner`; scene assets flow APK `assets/` →
  `AAssetManager` → MuJoCo VFS.
- **WebXR target** — the same core compiled to wasm32, with a WebGL2
  renderer and a ~425-line hand-written ES module for the session, the
  gamepad and the opaque-framebuffer bridge. Threads are off, so no
  `SharedArrayBuffer` and **no COOP/COEP headers**; `python3 -m http.server`
  plus `adb reverse` is the whole deployment.
- **Headless verification** — `bench/teleop_replay` gives the teleop logic
  its first execution outside a headset: a golden trajectory that locks the
  refactor, plus the frame-convention axis map, zero engage jump and slew
  compliance. It links the portable core into a host binary, which is also
  what keeps the core free of shell dependencies.

## Requirements

- CMake ≥ 3.16, a C/C++17 toolchain, network access at configure time
  (MuJoCo, OpenXR SDK and their dependencies are fetched and pinned)
- Android NDK r26+ and `glslangValidator` (Debian/Ubuntu: `glslang-tools`)
  for the Android build
- Android SDK build-tools + platform jar + a JDK for APK packaging
- emsdk, pinned to **4.0.10**, for the browser build (see
  [docs/validation-web.md](docs/validation-web.md) for why the pin matters)
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
./build/teleop_replay build/franka/ar_scene.xml \
    --ref baselines/teleop-host-x86_64.txt
```

The host build is the common prefix of all three targets — it compiles and
links the entire portable core, so it is the fastest way to find out whether
a change to physics, teleop or frame conventions broke anything.

Browser app (Quest Browser, or desktop Chrome's DevTools WebXR panel):

```
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web --parallel --target mxr baseline teleop_replay
cd build-web && python3 -m http.server 8000
# on a headset: adb reverse tcp:8000 tcp:8000, then open http://localhost:8000
```

Build **named targets only, never `all`** — upstream's wasm subdirectory is
added unconditionally under Emscripten and writes into the source tree.

Headset app:

```
cmake -S . -B build-android \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L"
cmake --build build-android --parallel
app/android/package-apk.sh build-android mujocoxr.apk
adb install -r mujocoxr.apk
adb logcat -s mujocoxr
```

An existing MuJoCo checkout can be used instead of the download with
`-DMUJOCOXR_MUJOCO_DIR=/path/to/mujoco` (must be v3.10.0).

In the headset: squeeze to clutch the green target marker to your hand,
trigger to close the gripper, A to reset the scene. Follow
[docs/validation-android.md](docs/validation-android.md) for the ordered
bring-up gates (benchmark → XR skeleton → handedness → teleop acceptance →
soak). The browser target has the same five gates in
[docs/validation-web.md](docs/validation-web.md), and more of them are
green — nothing here has ever run on a headset.

## Layout

- `src/` — the portable core, shared by both shells and by the host tools:
  frame conventions, clutched teleop, DLS IK, the frame loop
  (`sim_scene`), mesh building, rate limiter, logging, error hooks. What
  makes this a real tier is the `mxr_core` CMake target's restricted source
  list, built in the default host configuration on every build — not the
  directory name.
- `app/android/` — the OpenXR + Vulkan shell: XR session, Vulkan context,
  `mjvScene` renderer, APK asset loading, GLSL shaders, manifest and
  packaging script
- `app/web/` — the WebXR + WebGL2 shell: the C ABI (`abi.h`), renderer,
  GLSL ES shaders, `shell.js`, `index.html`
- `host/` — host-side IK prototype (convergence + nullspace checks)
- `bench/` — invariant baseline recorder/checker, teleop replay, `testspeed`
- `baselines/` — recorded references: the cross-platform invariant, and the
  host teleop-refactor lock
- `scripts/` — Menagerie scene fetcher (sparse, pinned)
- `docs/` — per-target validation guides, with shared gate numbering

## License notes

MuJoCo and the MuJoCo Menagerie Franka model are Apache-2.0; the packaging
script ships the model's LICENSE inside the APK.
