# On-device validation

Ordered gates for bringing MuJoCoXR up on a Quest-class arm64-v8a headset.
Run them in order — each later gate assumes the earlier ones hold.

Prereqs: Android NDK r26+ (`$ANDROID_NDK`), device authorized over adb.
All commands run from the repo root.

## 1. Engine benchmark + invariant check (before trusting anything else)

Cross-compile the CLI tools:

```
cmake -S . -B build-android \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L"
cmake --build build-android --parallel --target baseline testspeed
```

(`-D_POSIX_C_SOURCE=200809L` is required: bionic gates `localtime_r` on it,
unlike glibc under `_GNU_SOURCE`. Flag-level only — upstream MuJoCo is never
patched.)

Push and run:

```
scripts/fetch-menagerie.sh
DEV=/data/local/tmp/mujocoxr
adb shell mkdir -p $DEV
adb push build-android/lib/libmujoco.so \
  $ANDROID_NDK/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/aarch64-linux-android/libc++_shared.so \
  build-android/baseline build-android/testspeed \
  baselines/host-x86_64.txt $DEV/
adb push third_party/menagerie/franka_emika_panda $DEV/franka_emika_panda
adb shell chmod +x $DEV/baseline $DEV/testspeed

# 67-mesh load + decoder registration + dynamics invariant vs host reference
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./baseline franka_emika_panda/scene.xml --ref host-x86_64.txt"

# thermal-soaked median: one warm-up, then 5 measured rollouts
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./testspeed franka_emika_panda/scene.xml 20000 1" >/dev/null
for i in 1 2 3 4 5; do
  adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./testspeed franka_emika_panda/scene.xml 20000 1" | grep 'per second'
done
```

Expect `invariant_check = PASS`, then apply the threading decision
(budget: 72 Hz frames, ~7 × 2 ms steps + IK + scene extraction ≤ ~11 ms):

| Median `mj_step` cost | Decision |
|---|---|
| ≤ 0.5 ms/step (≥ 2000 steps/s) | single-threaded frame loop confirmed (as coded) |
| 0.5–2 ms/step | move stepping to a dedicated sim thread (same contracts) |
| > 2 ms/step | tune solver iterations, then timestep (4 ms is safe under `implicitfast`) |

## 2. XR skeleton — stable 72 Hz + input

Install the APK (see README) and watch `adb logcat -s mujocoxr`: a
`grip p=... trig=...` line once per second, clean session transitions on
HMD sleep/wake, no swapchain errors. Confirm 72 Hz with < 1% dropped frames
using the runtime's perf HUD (e.g. OVR Metrics on Quest).

## 3. Render + handedness (BEFORE trusting any teleop motion)

- The Panda stands on the checker floor in its home pose, ~1 m in front of
  the user (robot base at the MuJoCo origin; user forward = MJ +x).
- World-axes gizmo at the robot base: red = MJ +x (away from the user),
  green = MJ +y (user's LEFT), blue = MJ +z (up) — REP-103 (x fwd, y left,
  z up).
- Per-axis handedness: while squeezing, move the controller forward / left /
  up and confirm the green target marker moves the same way.
- Any mismatch means the `frames.h` constant is wrong for this runtime:
  stop, fix, re-check. 72 Hz must hold with the full scene.

## 4. Teleop acceptance

- Clutch engage/disengage (squeeze > 0.8 / < 0.6): zero marker jump.
- Marker tracks the hand under the 1.5 m/s / 3 rad/s rate limits; the arm
  follows. Logcat prints `teleop: engaged | target-TCP: X mm, Y deg` once a
  second — expect ≤ 2 cm / 5° while moving at ~0.25 m/s.
- Trigger closes the gripper monotonically (255 = open, inverted mapping).
- A resets to `home` and re-anchors the marker at the TCP.
- Recentering mid-clutch auto-disengages
  (`teleop: clutch auto-disengaged (recenter)`); B does nothing (unbound).
- Feel tuning knobs: `lambda` / `ns_gain` in `src/ik_dls.c`, clutch `scale_`
  in `app/teleop.h`.

## 5. Soak

10-minute thermal soak re-running gates 3 and 4. The Menagerie license
NOTICE ships in the APK (`assets/franka/LICENSE`).
