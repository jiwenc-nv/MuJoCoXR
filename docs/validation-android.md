# Validation — Android / OpenXR

Ordered gates for bringing MuJoCoXR up on a Quest-class arm64-v8a headset.
Run them in order — each later gate assumes the earlier ones hold.

Gate numbers, titles and the PASS / UN-RUN / N/A / PARTIAL legend are shared
across every target and live in
[validation-gates.md](validation-gates.md) — read that first if you have not.

Most gates here are UN-RUN, and both other targets have more green. Why that
inversion is real rather than an accident of who wrote what is in
[validation-gates.md](validation-gates.md); the short version is that nothing
in this repo has ever run on a headset, and this is the target that needs one
most.

Prereqs: Android NDK r26+ (`$ANDROID_NDK`), `glslangValidator`, a device
authorized over adb. All commands run from the repo root.

## 1. Engine benchmark + invariant check

**Status: UN-RUN** — `blocked on: a physical arm64 device over adb.`
The same binary and the same reference pass on host and on wasm32; see
[validation-web.md](validation-web.md) gate 1.

Cross-compile the CLI tools:

```
cmake -S . -B build-android \
  -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-29 \
  -DANDROID_STL=c++_shared -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-D_POSIX_C_SOURCE=200809L"
cmake --build build-android --parallel \
  --target baseline testspeed teleop_replay
```

(`teleop_replay` is built here rather than in gate 4 so that one configure
serves both; gate 4 only runs it.)

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
  build-android/baseline build-android/testspeed build-android/teleop_replay \
  baselines/host-x86_64.txt baselines/teleop-franka-host-x86_64.txt \
  baselines/teleop-so101-host-x86_64.txt $DEV/
adb push third_party/menagerie/franka_emika_panda $DEV/franka_emika_panda
# Each ar_scene.xml is first-party and lives in assets/<id>/; CMake stages it
# beside the Menagerie files it includes, so push the STAGED trees, not
# third_party. One directory per scene id.
adb push build-android/franka $DEV/franka
adb push build-android/so101 $DEV/so101
adb shell chmod +x $DEV/baseline $DEV/testspeed $DEV/teleop_replay

# 67-mesh load + decoder registration + dynamics invariant vs host reference
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./baseline franka_emika_panda/scene.xml --ref host-x86_64.txt"

# thermal-soaked median: one warm-up, then 5 measured rollouts
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./testspeed franka_emika_panda/scene.xml 20000 1" >/dev/null
for i in 1 2 3 4 5; do
  adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./testspeed franka_emika_panda/scene.xml 20000 1" | grep 'per second'
done
```

Expect `invariant_check = PASS`, and read `qpos_linf_diff` against the four
bands in [validation-web.md](validation-web.md) — those bands are shared
policy, not per-target. In particular `1e-6 … 1e-3` reads green and must not
be landed on, and `kQposTol` is never widened.

arm64 is the target most likely to diverge: it has `FMADD` in the base ISA
and clang defaults to `-ffp-contract=on`, which is exactly why the build
forces `-ffp-contract=off` everywhere.

Then apply the threading decision (budget: 72 Hz frames, ~7 × 2 ms steps +
IK + scene extraction ≤ ~11 ms):

| Median `mj_step` cost | Decision |
|---|---|
| ≤ 0.5 ms/step (≥ 2000 steps/s) | single-threaded frame loop confirmed (as coded) |
| 0.5–2 ms/step | move stepping to a dedicated sim thread (same contracts) |
| > 2 ms/step | tune solver iterations, then timestep (4 ms is safe under `implicitfast`) |

## 2. XR skeleton — session, pacing and input

**Status: UN-RUN** — `blocked on: a physical headset.`

Install the APK (see README) and watch `adb logcat -s mujocoxr`: a
`grip p=... trig=...` line once per second, clean session transitions on
HMD sleep/wake, no swapchain errors. Confirm 72 Hz with < 1% dropped frames
using the runtime's perf HUD (e.g. OVR Metrics on Quest).

At startup the log must show which reference space won, and the calibration:

```
reference space: LOCAL_FLOOR
frames: t_mj_from_xr = (-1.000 0.000 -0.730) m, clock = XrFrameState::predictedDisplayTime
```

`STAGE` or `LOCAL` instead of `LOCAL_FLOOR` is a finding, not a detail — see
the three-cause table under gate 3. (Android falls back here where the web
shell hard-fails. That asymmetry is deliberate: changing it is a behaviour
change to a target with no hardware to re-verify on, so it is recorded as a
follow-up rather than made inside this landing.)

If `dt has been 0 for 11 frames running` appears, the clock named on that
line has stopped advancing — physics is frozen while rendering stays perfect,
which is otherwise invisible.

**Input characterization** — ramp trigger and squeeze 0 → full → 0 while
watching the once-a-second grip line, which reads
`grip p=(… … …) q=(… … … …) trig=… sqz=…`. Require max ≥ 0.95 and monotonic.
A full squeeze saturating below 0.85 makes the 0.8 engage threshold
unreachable and forces `kEngageThreshold`/`kReleaseThreshold` to become
parameters rather than constants. **UN-RUN** — `blocked on: a physical
controller.`

## 3. Render + handedness (BEFORE trusting any teleop motion)

**Status: PARTIAL.** The arithmetic half is **PASS**; everything requiring
eyes is **UN-RUN** — `blocked on: a physical headset.`

The frame convention no longer needs a device. `bench/teleop_replay` checks
it on every build:

```
frames: 1 m fwd -> MJ +x           [ok]
frames: XR -Z/+Y/+X axis map       [ok]
frames: mat4 inverts the point map [ok]
```

**PASS** 2026-07-28, host x86_64 and wasm32. This is the half of this gate
that never needed hardware and had never been run on any target.

Requiring a headset:

- The Panda stands on its table in the passthrough view, ~1 m in front of
  the user with the table legs on the physical floor (user forward = MJ +x).
- World-axes gizmo at the robot base: red = MJ +x (away from the user),
  green = MJ +y (user's LEFT), blue = MJ +z (up) — REP-103 (x fwd, y left,
  z up).
- Per-axis handedness: while squeezing, move the controller forward / left /
  up and confirm the green target marker moves the same way.
- **Stand still and look down: the ground plane is at your feet, not your
  waist.** The gizmo cannot catch a whole-scene vertical offset — handedness
  stays perfectly correct while everything floats.

**A wrong floor height has three causes that present identically.** Work down
this table; do not start by editing the constant:

| Symptom | Cause | Fix |
|---|---|---|
| Scene ~1.6 m high, gizmo correct | Non-floor reference space; the LOCAL fallback puts the origin at head height | Read the `reference space:` line from gate 2 |
| Scene off by ~0.2 m | The runtime's floor estimate is wrong for this room | Re-run the runtime's guardian/boundary setup |
| Scene off by the table height | `MXR_T_MJ_FROM_XR.z` is wrong for this deployment | Edit `src/frames.h` — it is a workspace **calibration** and is expected to differ per room |

`MXR_Q_MJ_FROM_XR` is the other kind of constant: a handedness **convention**
fixed by the two specs, which cannot be "wrong for this room". If the gizmo
is wrong, that is the bug. If only the height is wrong, it is not.

72 Hz must hold with the full scene.

**Passthrough alpha — unresolved on ALL THREE targets. Do not fix blind on
one:**

| Symptom | Cause | Fix |
|---|---|---|
| Translucent decor (engaged marker α=0.5, idle α=0.35) looks washed out or too bright over passthrough | OpenXR `ALPHA_BLEND` is specified as **premultiplied**; `scene.frag` emits unpremultiplied | Both shaders need `rgb *= a` **and** the colour blend factor must become `ONE`. Two coupled changes across **three targets and two shaders** — the Vulkan shader is shared by Android and Linux. Fix all three together or they diverge. |

## 4. Teleop acceptance

**Status: PARTIAL.** The semantics are **PASS** headlessly; the acceptance
checks are **UN-RUN** — `blocked on: a physical headset.`

`bench/teleop_replay` drives a scripted engage → move → release → recenter →
tracking-loss → A-reset sequence on every build:

```
teleop: engage pos jump bitwise 0     [ok]
teleop: engage quat jump <= 1e-12 rad [ok]   worst 2.22e-16 rad
teleop: slew within rate limits       [ok]   worst 0.24 of the bound
```

**PASS** 2026-07-28, host x86_64 and wasm32. The engage claim is exact for
position and accurate to ~2e-16 rad for orientation; `bench/teleop_replay.cc`
explains why the two halves are asserted at different bounds, and
`src/teleop.h` explains where the orientation residual actually comes from
(the `float` grip quaternion, whose defect is a magnitude error and so
rotationally inert — not double rounding).

`teleop_replay` also cross-compiles and links for arm64 — it is in gate 1's
`--target` list, its binary and its reference are in gate 1's `adb push`, and
it is `chmod +x`'d there, so this needs no setup of its own:

```
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./teleop_replay franka/ar_scene.xml --ref teleop-franka-host-x86_64.txt"
adb shell "cd $DEV && LD_LIBRARY_PATH=$DEV ./teleop_replay so101/ar_scene.xml --ref teleop-so101-host-x86_64.txt"
```

That checks the frame convention, the engage and slew assertions and the
geometry census, and **skips the bitwise trace comparison loudly**, because
those references record `arch = x86_64` and this build reports `aarch64`. That
is the correct answer, not a failure.

Two of the recorded fields ARE compared here, and they are the reason running
this on arm64 is worth anything at all now that the hash is skipped:

- `opt_timestep` / `opt_integrator` / `opt_cone` / `opt_impratio` — the
  scene's solver configuration. A wrapper XML that silently loses its
  `<option>` block (see `assets/so101/ar_scene.xml`) fails as a named line.
- `pos_med_mm` — median target-to-TCP distance over the engaged frames,
  against a 1.5x + 1 mm tolerance. Verified identical to two decimals between
  x86_64 and wasm32 (Franka 23.69 mm, SO-101 2.23 mm), so a material
  difference on arm64 is a finding, not noise.

**UN-RUN** — `blocked on: a physical arm64 device over adb.`

Requiring a headset:

- Clutch engage/disengage (squeeze > 0.8 / < 0.6): zero marker jump.
- Marker tracks the hand under the 1.5 m/s / 3 rad/s rate limits; the arm
  follows. Logcat prints `teleop: engaged | target-TCP: X mm, Y deg` once a
  second — expect ≤ 2 cm / 5° while moving at ~0.25 m/s. **This is a
  hand-motion figure and is not the `pos_med_mm` the replay gate records**;
  that one is measured over a scripted trajectory that drives the arm much
  harder, and the Franka's shipped value is 23.69 mm. A Franka printing ~2 cm
  here is nominal, not broken.
- Trigger closes the gripper monotonically, from that robot's tabulated
  `gripper_open` at trigger 0 to `gripper_closed` at 1 (Franka 255 → 0,
  SO-101 1.745 → 0; both are "low = closed" but that is a coincidence of the
  two models, not a rule). A `gripper endpoints ... fall outside` warning at
  startup means `src/robot_spec.c` disagrees with the model.
- A resets to `home` and re-anchors the marker at the TCP.
- Recentering mid-clutch auto-disengages
  (`teleop: clutch auto-disengaged (recenter)`).
- **B cycles the scene.** Read the note below before treating this as a menu.
- **Grip lever-arm check**: engage, then twist ~90° about the controller's
  own axis without translating. Accept ≤ 5 mm of target motion. > 2 cm means
  the WebXR and OpenXR grip origins genuinely differ, and the fix is a
  documented tool-frame offset — **not** fiddling with `MXR_Q_MJ_FROM_XR`.
- **Visibility resume**: headset off and back on mid-clutch. Expect the
  clutch to have **dropped**, **no target jump**, and **no `sim deficit`
  line** — but which log line says so depends on the runtime, and both
  outcomes are correct:

  | Runtime takes the session | Mechanism | Log line to expect |
  |---|---|---|
  | ... through `STOPPING` (`session_running()` true → false) | `SimScene::EndSession` | **none** — `EndSession` disengages silently |
  | ... `VISIBLE` → `SYNCHRONIZED` → `FOCUSED`, never `STOPPING` | grip poses stop locating, so `grip_valid` goes false | `teleop: clutch auto-disengaged (tracking lost)` |

  `session_running_` is cleared only on `STOPPING` / `EXITING` /
  `LOSS_PENDING` (`XrShell::HandleSessionStateChange` in
  `src/openxr/xr_shell.cc`, which latches the edge that
  `XrShell::TakeSessionEndEdge()` hands to the shell), so on a runtime that
  only drops focus, the second row is the path taken and `EndSession` never
  fires.
  Either row satisfies the acceptance above; a **third** outcome — clutch
  still engaged on resume — is the failure. On the `EndSession` path the
  clock also re-latches, so the first resumed frame has `dt = 0` rather than
  a clamped 0.1 s that owes ~50 steps against a cap of 15; on the
  `grip_valid` path the session never stopped, so there was no gap to owe.
  Before this landing the loop merely `continue`d, and the expectation
  documented here (one `sim deficit` line) described behaviour the code did
  not produce. **UN-RUN** — `blocked on: a physical headset.`

### Switching robots on Android — this is not a menu

**Android has no in-headset scene menu, and cannot have one today: this stack
has no text rendering, so there is nothing to draw one with.** The app starts
on the first scene in `src/robot_spec.c` and **B switches to the next one**.
Choosing a scene before entering is Web-only (`?scene=<id>`, see
[validation-web.md](validation-web.md)).

That is a deviation from "a menu on start", stated as one rather than
described as a feature. It was taken because the alternative — one launcher
icon per robot — is *measured dead*: two
`<activity android:name="android.app.NativeActivity">` entries link and
install, but they collapse to a single `ComponentName`, so only one is ever
resolved, and distinguishing them requires subclassing `NativeActivity`,
which requires Java, which ends `android:hasCode="false"`. The tiebreaker was
the failure shape: the launcher approach fails **at install**, with no path
to any robot at all, while B-cycling fails **only if you press B**.

Because there is no label to read in-headset, **every cycle logs one line**,
and it is the only way to answer "which robot am I on":

```
adb logcat -s mujocoxr | grep '^.*scene:'
# scene: Franka Emika Panda (franka), 1 of 2
# scene: SO-101 (so101), 2 of 2
```

Acceptance for a cycle: the arm is replaced, the clutch is **dropped** (the
swap runs `EndSession` first), the marker re-anchors at the new robot's TCP,
and there is no `sim deficit` line on the following frame. A failed load
degrades to the clear-color loop with `assets/<id> not found in APK` — the
app stays usable on nothing, which is the bounded-downward case the design
chose. **UN-RUN** — `blocked on: a physical headset.`

Feel tuning is now per robot and lives in one place: the row in
`src/robot_spec.c`. `w_rot`, `lambda`, `ns_gain` and `clutch_scale` are
fields there, each with the measurement that chose it written beside it.
**`w_rot`, `clutch_scale` and the robot's placement are the three numbers
most likely to move on the first on-device session**; when one does,
re-record that scene's baseline in the same commit and say which number moved.

## Behaviour changed by the Linux-client landing

Android has no hardware to re-verify on, so changes reaching it are listed
rather than assumed harmless:

- the `oculus/touch_controller` hard-fail is gone (≥1 accepted profile now
  suffices), and `khr/generic_controller` is suggested where advertised;
- `XR_KHR_composition_layer_depth` is **enabled and depth is submitted** where
  the runtime advertises it — which Quest does. The render pass stores depth
  instead of discarding it, so expect a small bandwidth cost on a tiler and,
  in exchange, reprojection that has something to work with. A runtime without
  the extension keeps the app-owned depth image and the `DONT_CARE` store;
- teardown order is XR-then-Vulkan (was the reverse, a latent use-after-free);
- one new `VALID && !TRACKED` warning line.

## 5. Soak

**Status: UN-RUN** — `blocked on: a physical headset.`

10-minute thermal soak re-running gates 3 and 4. The Menagerie license
NOTICE ships in the APK (`assets/franka/LICENSE`).
