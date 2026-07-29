# Validation — Linux / desktop OpenXR (CloudXR)

Ordered gates for bringing MuJoCoXR up as a Linux OpenXR client, streaming to
a headset through the NVIDIA CloudXR runtime. Run them in order — each later
gate assumes the earlier ones hold.

Gate numbers, titles and the PASS / UN-RUN / N/A / PARTIAL legend are shared
across every target and live in
[validation-gates.md](validation-gates.md) — read that first if you have not.

> **This is the first target whose session bring-up can be demonstrated with
> no headset in the building**, and the reason is structural rather than
> lucky: the CloudXR runtime is a service on this machine, so everything up to
> and including `xrCreateSession`, swapchain allocation and the reference-space
> decision runs against a real runtime with a virtual head device. Everything
> that needs a *human wearing something* — controllers, handedness, teleop
> feel — is still UN-RUN, and is still the majority of what matters.

Prereqs: `cmake`, `glslang-tools`, `libvulkan-dev`, `libx11-dev`, and a
CloudXR runtime. `scripts/build-linux.sh` preflights the first four by name.

## Build

```
scripts/build-host.sh          # run this first — see README
scripts/build-linux.sh
```

`build-linux.sh` has no `--no-checks` flag because it has no checks. Gate 1 on
this target is free: it *is* `build-host.sh`, the same `mxr_core` from the same
source list compiled by the same host compiler.

## Run — the runtime is a host-level singleton

**This program never starts, stops or supervises the runtime.** Another
process may own it. In a second terminal:

```
cd /code && scripts/run_cloudxr_runtime.sh
```

then:

```
build-linux/mujocoxr --probe            # no headset needed
build-linux/mujocoxr --scene franka
build-linux/mujocoxr --help
```

Three flags, and `--scene` takes an **id**, never a path — the id is the only
name a scene has (`src/robot_spec.c`, and the comment at `mxr_stage_scene` in
`CMakeLists.txt`). A wrong id lists the valid ones and exits before any OpenXR
call is made.

There is **no B-cycle and no in-process scene swap** on this target, and that
is a deletion rather than an omission. Android cycles scenes on B because it
has no other input device and no way to draw a menu; a desktop has a command
line. Restoring it would mean re-adding the most delicate code in the app
(`app/android/main.cc`'s teardown ordering) to buy what an argument buys.

### Four environment traps, all measured on 2026-07-29

Every one of these presents as the same symptom — `XR_ERROR_RUNTIME_UNAVAILABLE
(-51)` or a crash — and they have four different fixes.

| Symptom | Cause | Fix |
|---|---|---|
| `-51`, loader says "failed to load ... no such file" | `run_cloudxr_runtime.sh` exports `XR_RUNTIME_JSON=$HOME/.cloudxr/share/openxr/1/openxr_cloudxr.json`, which **does not exist**. The manifest on disk is `$HOME/.cloudxr/openxr_cloudxr.json`. | `export XR_RUNTIME_JSON=$HOME/.cloudxr/openxr_cloudxr.json`. The script lives in a different repo; fix it there. |
| `-51`, `ipc_client_socket_connect: Failed to connect to socket ~/.cache/ipc_cloudxr` | The client looks for the IPC socket in its default location; the service was told to put it somewhere else. | `export NV_CXR_RUNTIME_DIR=$HOME/.cloudxr/run` — the **same value the service was started with**. |
| `ipc_client_check_git_tag: client library version X does not match service version Y` | The installed client `libopenxr_cloudxr.so` and the running `cloudxr-service` are different builds. | Point `XR_RUNTIME_JSON` at the manifest for the client library **co-built with the service you are running**. |
| Client dies with SIGBUS immediately after that message when `IPC_IGNORE_VERSION=1` is set | The version check was right. `IPC_IGNORE_VERSION` silences the message, not the ABI difference. | Do not set it. Fix the pairing instead. |

`cloudxr-service` also needs its own libraries on `LD_LIBRARY_PATH`
(`libcloudxr.so` from its build tree, `libNvStreamBase.so` from
`deps/streamsdk_binaries/target/bin/linux/x86_64-linux-gnu`) or it exits before
creating the socket, which then presents to the client as trap 2.

## 1. Engine benchmark + invariant check

**Status: PASS** 2026-07-29, host x86_64 — and it is free on this target,
which no other target can say.

The Linux client is a host build. `scripts/build-host.sh` compiles the same
`mxr_core` from the same source list with the same compiler and runs the
cross-architecture dynamics invariant and both golden teleop traces:

```
invariant franka: PASS
teleop franka: PASS
teleop so101: PASS
all checks PASS
```

Read `qpos_linf_diff` against the four bands in
[validation-web.md](validation-web.md) — those bands are shared policy, not
per-target. There is no separate on-device push here because there is no
device: the CPU running the physics is the CPU that recorded the reference.

## 2. XR skeleton — session, pacing and input

**Status: PARTIAL.** Session bring-up is **PASS**; everything requiring a
controller is **UN-RUN** — `blocked on: a CloudXR-connected headset.`

Measured 2026-07-29, x86_64, against `NVIDIA™ CloudXR™ Runtime 6.1.0` built
from `/code/CXR/Runtime`, **no headset attached**:

```
[mujocoxr I] OpenXR runtime: NVIDIA™ CloudXR™ Runtime 'HEAD-HASH-NOTFOUND' 6.1.0, API 1.1 granted (local_floor_ext=0)
[mujocoxr I] environment blend mode: ALPHA_BLEND (passthrough)
[mujocoxr I] suggested bindings: /interaction_profiles/oculus/touch_controller
[mujocoxr I] suggested bindings: /interaction_profiles/khr/simple_controller
[mujocoxr I] interaction profiles accepted: 2
[mujocoxr I] runtime Vulkan API range: 281474976710656..287953294926545919
[mujocoxr I] reference space: LOCAL_FLOOR
[mujocoxr I] swapchain 0: 2048x1792, 3 images, format 43
[mujocoxr I] swapchain 1: 2048x1792, 3 images, format 43
[mujocoxr I] session state -> IDLE / READY / SYNCHRONIZED / VISIBLE / FOCUSED
```

`format 43` is `VK_FORMAT_R8G8B8A8_SRGB`, the first preference.

**The headless half of this gate — runnable with no headset, and with the
service down.** (There is no "gate 0": the gate numbers are the same five on
every target, and this is gate 2 splitting the way gate 3 already does.)
`--probe` brings the instance up, runs the whole instance-scope action setup,
reports what was accepted and exits without creating a session. With the
service **not** running it is the fastest possible statement of that fact:

```
$ build-linux/mujocoxr --probe                      # exit 1, 3 ms
[mujocoxr E] xrCreateInstance: XR_ERROR_RUNTIME_UNAVAILABLE (-51) — the OpenXR runtime is installed but not running.
[mujocoxr E]   XR_RUNTIME_JSON = /home/jiwenc/.cloudxr/openxr_cloudxr.json
[mujocoxr E]   For CloudXR, start the service first: cd /code && scripts/run_cloudxr_runtime.sh
```

With it running:

```
$ build-linux/mujocoxr --probe                      # exit 0, 17 ms
[mujocoxr I] probe: instance OK, 2 interaction profiles accepted, system found
```

**What `--probe` does NOT tell you.** `xrSuggestInteractionProfileBindings` is
app→runtime: a profile accepted there is one the runtime knows how to bind,
**not** one it will bind to whatever controller connects. The line that answers
that is `interaction profile (right): ...`, logged at `FOCUSED` from a real
session — and with no controllers attached it correctly reads `NONE bound`.

**Input characterization** — ramp trigger and squeeze 0 → full → 0 while
watching the once-a-second grip line. Require max ≥ 0.95 and monotonic; a full
squeeze saturating below 0.85 makes the 0.8 engage threshold unreachable.
**UN-RUN** — `blocked on: a physical controller.`

**"I see nothing" checklist**, in order — each step is a different question,
and the first two are answered by the traps table above rather than restated:

1. `pgrep -a cloudxr-service` — is the service up at all?
2. `mujocoxr --probe` — exit 0? If not, its output names which trap you hit.
3. Does a headset client appear in the runtime's own log? Nothing here can
   tell you that; only the runtime knows whether a client attached.
4. `interaction profile (right):` — `NONE bound` means the runtime has no
   controller, **not** that the bindings are wrong.
5. `sim deficit` lines with no picture mean physics runs and rendering does
   not; `dt has been 0` means the reverse.

### Interaction profiles — which fallback you actually get

The suggestion set is: `oculus/touch_controller`,
`bytedance/pico4*` (gated on `XR_BD_controller_interaction`),
`khr/generic_controller` (gated on `XR_KHR_generic_controller`), and
`khr/simple_controller`. **No single profile is mandatory; bring-up fails only
if none is accepted.**

`khr/generic_controller` is the one worth caring about. It is the only
advertised fallback carrying grip pose **and** an analog trigger **and** an
analog squeeze **and** two buttons, so clutch, gripper, reset and — on other
targets — scene cycling all work on it. Its component paths are
`primary/click` and `secondary/click`, **not** `a`/`b` and not `menu`.

| Runtime | `XR_KHR_generic_controller` | Consequence |
|---|---|---|
| CloudXR 6.2.1 (installed at `~/.cloudxr`, Jul 2026) | advertised | best fallback available |
| CloudXR 6.1.0 (built from `/code/CXR/Runtime`) | **not** advertised | falls back to `khr/simple_controller`: no analog gripper, no second button |

The gating is not optional. `xrCreateInstance` returns
`XR_ERROR_EXTENSION_NOT_PRESENT` for any unadvertised name in the enabled list,
so enabling it unconditionally would be a hard startup failure on 6.1.0 — and
on Android, where nobody can check. **The `khr/generic_controller` suggestion
path itself is UN-RUN** — `blocked on: a runtime that advertises it.` The
6.1.0 run above exercised the *absence* branch, not the presence branch.

## 3. Render + handedness (BEFORE trusting any teleop motion)

**Status: PARTIAL.** The arithmetic half and the reference-space decision are
**PASS**; everything requiring eyes is **UN-RUN** —
`blocked on: a CloudXR-connected headset.`

`bench/teleop_replay` checks the frame convention on every build (gate 1).

**The reference space is the finding of this landing, and it needed two
changes that only work together.** `XR_EXT_local_floor` is **not advertised**
by this runtime — confirmed above as `local_floor_ext=0`. The old code drove
its `LOCAL_FLOOR → STAGE → LOCAL` fallback purely off that extension string, so
it would have started at STAGE. But `XR_EXT_local_floor` was promoted into
OpenXR **1.1**, where the space is core and no extension is advertised for it.
So the client now requests 1.1 and treats a granted 1.1 as sufficient:

- Request 1.1 alone → still STAGE, silently. The predicate never asks.
- Fix the predicate alone → still STAGE. At 1.0 the space genuinely is absent.
- Both → `reference space: LOCAL_FLOOR`, measured.

If a future runtime rejects 1.1, the client retries at 1.0 and says so at
`LOGW`, naming the consequence — a STAGE fallback and a possibly wrong floor
datum. That path is **UN-RUN**; this runtime granted 1.1.

Requiring a headset:

- The robot stands on its table ~1 m in front of you, table legs on the floor.
- World-axes gizmo at the base: red = MJ +x (away), green = MJ +y (your LEFT),
  blue = MJ +z (up) — REP-103.
- Per-axis handedness while squeezing: forward / left / up all agree.
- **Stand still and look down: the floor is at your feet, not your waist.**

**A wrong floor height has four causes here and only three on Android.** Work
down the table; do not start by editing the constant.

| Symptom | Cause | What to check |
|---|---|---|
| Scene ~1.6 m high | The app space is `LOCAL`. Monado's `1.6` is the eye-height literal it uses to build the LOCAL space (`b_space_overseer.c:924-927`); the floor space then **overwrites** that y with STAGE's (`:945-949`), so **`LOCAL_FLOOR` can never carry the 1.6**. 1.6 m of offset is therefore *proof the space is `LOCAL`*, not a head-tracking race | The `reference space:` line — it will say `LOCAL`. If it says `LOCAL_FLOOR`, this is not your row |
| Scene has the right height but the wrong standoff/yaw | The app space fell back to `STAGE`, which keeps the floor datum and loses your position and facing | The `reference space:` line, and the `API ... granted` value beside it. `OXR_RECENTER_STAGE=1` remaps STAGE onto LOCAL_FLOOR (`oxr_space.c:113-116`) and is the escape hatch that gives the standoff back |
| Scene off by ~0.2 m | The runtime's floor estimate is wrong for this room | Re-run the guardian/boundary setup. This one is a *measurement* problem and no environment variable fixes it |
| Scene off by the table height | `MXR_T_MJ_FROM_XR.z` is wrong for this deployment | `src/frames.h` — a workspace **calibration**, expected to differ per room |

`MXR_Q_MJ_FROM_XR` is the other kind of constant: a handedness **convention**
fixed by two specs, which cannot be "wrong for this room".

**Passthrough alpha — unresolved on ALL THREE targets. Do not fix blind on
one:** OpenXR `ALPHA_BLEND` is specified as premultiplied and `scene.frag`
emits unpremultiplied. Both shaders need `rgb *= a` **and** the colour blend
factor must become `ONE`. This runtime reports `ALPHA_BLEND`, so the defect is
reachable here.

## 4. Teleop acceptance

**Status: PARTIAL.** The semantics are **PASS** headlessly (gate 1); the
acceptance checks are **UN-RUN** — `blocked on: a CloudXR-connected headset.`

Loading, IK init and the frame convention were exercised on a live session:

```
[mujocoxr I] model loaded: nq=9 nv=9 nu=8 nmesh=67
[mujocoxr I] geometry: 396815 vertices, 409806 indices, 67 meshes
[mujocoxr I] teleop: robot 'hand', 7 arm joints, w_rot=1 clutch_scale=1
[mujocoxr I] frames: t_mj_from_xr = (-1.000 0.000 -0.730) m, clock = XrFrameState::predictedDisplayTime
```

Requiring a headset — same list as
[validation-android.md](validation-android.md) gate 4 (clutch engage/disengage
with zero jump, rate-limited tracking, monotonic gripper, A resets, recenter
auto-disengages, grip lever-arm ≤ 5 mm, visibility resume), with two
differences that belong to this target:

- **There is no B-cycle.** Robots are chosen with `--scene <id>` at launch.
- **Constant lag is transport delay, not a bug.** CloudXR's
  `ClientController::getTrackedPose` ignores its `at_timestamp_ns` argument and
  returns the last received measurement verbatim — there is no forward
  extrapolation, so `xrLocateSpace(…, predictedDisplayTime)` and
  `xrLocateSpace(…, now)` return the same pose. That removes the
  engage-registration error a predicting runtime would introduce, and it means
  what you feel is the link. Re-open this if a CloudXR release starts honouring
  that parameter.

### Symptom → cause → what to check, for the link

| Symptom | Cause | What to check |
|---|---|---|
| Target freezes for ~0.1 s, then jumps to the hand | The stale-pose window. CloudXR clears `TRACKED` at **100 ms** and `VALID` at **200 ms** without a fresh measurement (`nv_settings.hpp:85-86` — a runtime **default**, so a future release can move both numbers). Between them the runtime serves the last pose it received and flags it live | `input: grip pose VALID but NOT TRACKED` — evaluated every frame, logged on the rising edge, with a lifetime count on the periodic `input:` line. If it fires, that is the trigger to add the TRACKED bits to the grip mask |
| **`sim deficit` continuously, with no headset attached** | **Not a defect, and not the link.** With nothing connected the runtime free-runs instead of pacing to a display, so `xrWaitFrame` returns as fast as it likes and every frame owes more steps than the cap concedes. Observed on every headless run in gate 2 above | Nothing. Judge pacing only against a real streaming session — this row exists so the *absence* of a headset is not mistaken for the presence of a problem |
| Arm lurches once per frame after a stall, **with a headset streaming** | `sim deficit` under link load — a different cause from the row above. A stalled frame clamps dt to 0.1 s, the step cap concedes ~15 steps ≈ 30 ms, so the **target** advanced 100 ms of slew while **physics** advanced 30 | Count the `sim deficit` lines per minute; it is the frequency that matters, not the presence. **Do NOT tighten `kMaxLinRate` to hide it** — `src/teleop.cc:11-30` already considered and rejected exactly that, and slowing the target to match a dropped frame makes teleop worse everywhere else |
| Jaw never closes | The bound profile has no analog trigger — `khr/simple_controller` has `select/click` and nothing else | `interaction profile (right): ...` at `FOCUSED`. If it is `khr/simple_controller`, this is expected, not broken |
| Constant lag, otherwise correct tracking | Transport delay (above). Not a prediction bug — there is no prediction | Measure it; do not "fix" it in `src/` |

## 5. Soak

**Status: UN-RUN** — `blocked on: a CloudXR-connected headset.`

10-minute soak re-running gates 3 and 4 over the link. Watch for thermal or
bandwidth-driven frame drops, and for the `VALID && !TRACKED` line.

| Symptom | Cause | What to check |
|---|---|---|
| Clutch still engaged after the headset sleeps and wakes | **The failure.** Two outcomes are correct — `EndSession` (silent) or `teleop: clutch auto-disengaged (tracking lost)` — and a **third**, still-engaged, is not. See the same table in [validation-android.md](validation-android.md) gate 4 | Which of the two log lines appeared. Neither appearing *and* the clutch held is the bug |
| A later run cannot reach the runtime at all, having worked before | A previous client died without `xrDestroySession` and wedged the host-level singleton | `pgrep -a cloudxr-service`, then the four traps above. This is what the SIGINT handler exists to prevent — if it happened, say how the client was killed |

**Clean exit is PASS** 2026-07-29: `SIGINT` unwinds the frame loop and tears
down in the order XR-then-Vulkan, exiting 0. That order is load-bearing — see
`app/linux/main.cc`; the reverse order segfaults inside the runtime, which is
how the bug was found.
