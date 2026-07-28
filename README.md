# MuJoCoXR

MuJoCo physics running **fully on-device** on a standalone XR headset
(Quest-class), with a robot arm teleoperated by an XR controller — a Franka
Emika Panda or an SO-101, picked at load time.
The MuJoCo C API with two thin shells over one shared core: raw
OpenXR + Vulkan on Android, and WebXR + WebGL2 in the headset browser. No
game engine, no Python in the app, no desktop in the loop.

## Features

- **On-device physics** — unmodified upstream MuJoCo v3.10.0 (fetched and
  pinned at build time, zero source patches) compiled GL-free via the
  filament-mjr-compat flag set; the unmodified MuJoCo Menagerie scenes
  step at 500 Hz on the headset CPU — composed onto a table in an AR
  scene (`assets/<id>/ar_scene.xml`) without touching the upstream files.
- **Two robots** — a Franka Emika Panda (7 dof) and an SO-101 (5 dof), each
  a scene the user picks: a DOM menu before entering on the web, and the B
  button to cycle in-headset on Android. The robot is identified by PROBING
  the loaded model, so no robot id is ever passed in. Adding a third is a
  checklist, all of it data —
  [docs/adding-a-robot.md](docs/adding-a-robot.md) is authoritative and this
  bullet deliberately does not restate the count.
- **Raw OpenXR shell** — `NativeActivity` + Khronos loader:
  `XR_KHR_vulkan_enable2` device handshake, `LOCAL_FLOOR` reference space
  (STAGE/LOCAL fallbacks), full session lifecycle, Touch-controller action
  set (grip pose, trigger, squeeze, A, B).
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
  construction); damped-least-squares IK — a 6D task against a 7-dof arm on
  the Franka and a rank-deficient 5-dof one on the SO-101, with the
  rotation-vs-position weight and the nullspace home-posture bias tabulated
  per robot — plus gravity feed-forward writes the position servos; targets
  rate-limited at 1.5 m/s / 3 rad/s; trigger drives the gripper through that
  robot's tabulated jaw endpoints (Franka 255 → 0, SO-101 1.745 → 0 rad; the
  polarity is per-model, not a convention); A resets to the home keyframe;
  recentering auto-disengages the clutch.
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

Fetch the robot scenes (sparse, pinned; used by host tools and the APK):

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
    --ref baselines/teleop-franka-host-x86_64.txt
./build/teleop_replay build/so101/ar_scene.xml \
    --ref baselines/teleop-so101-host-x86_64.txt
```

Both must print `replay_check = PASS`. One reference per scene, named for
the scene id; `baseline` and `testspeed` stay Franka-only (see the `nu < 8`
comment in `bench/baseline.cc`).

The host build is the common prefix of all three targets — it compiles and
links the entire portable core, so it is the fastest way to find out whether
a change to physics, teleop or frame conventions broke anything.

Browser app (Quest Browser, or desktop Chrome's DevTools WebXR panel):

```
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web --parallel --target mxr baseline teleop_replay
cd build-web && python3 -m http.server 8000   # then open http://localhost:8000
```

Build **named targets only, never `all`** — upstream's wasm subdirectory is
added unconditionally under Emscripten and writes into the source tree.

### On a headset

WebXR requires a **secure context**. `https://` and `localhost` qualify;
`http://<LAN-IP>:8000` does not, and on an insecure origin `navigator.xr` is
undefined — which looks identical to a browser with no WebXR at all. The
shell tells the two apart so you do not go and install a different browser.

Prefer adb. It needs no certificate and no warning to click through, because
localhost is trusted by definition:

```
adb reverse tcp:8000 tcp:8000                      # over USB
# wireless: adb tcpip 5555 && adb connect <headset-ip>:5555, then reverse
```

then open `http://localhost:8000` in the headset browser.

Where adb is not an option, serve TLS with a self-signed certificate:

```
scripts/serve-web-tls.py build-web 8443            # prints the URL to open
```

The certificate carries the address in `subjectAltName`, because Chromium
rejects a CN-only certificate outright instead of offering the warning you
need to click through. Accept it once per origin. **Unverified on a headset
browser** — the bypass-then-secure-context behaviour is how self-signed WebXR
development normally works, but nothing here has run on a device; if the
browser still refuses to enter XR, use adb.

Neither server compresses. `mxr.data` is 34 MB on the wire (5 MB gzipped), so
over Wi-Fi the first load is slow and the browser cache is what makes the
second one fast. If the page never loads at all, check the dev box firewall
before anything else — the port has to be reachable.

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
trigger to close the gripper, A to reset the scene, B to cycle to the next
robot (Android only — on the web, B ends the session and robots are picked
from the page menu). Follow
[docs/validation-android.md](docs/validation-android.md) for the ordered
bring-up gates (benchmark → XR skeleton → handedness → teleop acceptance →
soak). The browser target has the same five gates in
[docs/validation-web.md](docs/validation-web.md), and more of them are
green — nothing here has ever run on a headset.

## Layout

- `src/` — the portable core, shared by both shells and by the host tools:
  frame conventions, the robot/scene tables (`robot_spec`), clutched teleop,
  DLS IK, the frame loop (`sim_scene`), mesh building, rate limiter, logging,
  error hooks. What
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
- `baselines/` — recorded references: the cross-platform invariant, and one
  host teleop lock per scene (`teleop-<id>-host-x86_64.txt`)
- `scripts/` — Menagerie scene fetcher (sparse, pinned), and a TLS dev
  server for headsets that cannot be reached over adb
- `docs/` — per-target validation guides with shared gate numbering, and
  [docs/adding-a-robot.md](docs/adding-a-robot.md)

## License notes

MuJoCo and the MuJoCo Menagerie Franka and SO-101 models are Apache-2.0; the
packaging script ships each model's LICENSE inside the APK, in that model's
own asset directory.
