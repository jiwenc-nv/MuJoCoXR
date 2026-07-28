# Validation — WebXR / browser

Ordered gates for bringing MuJoCoXR up in a headset browser. Gate numbers and
titles are identical to [validation-android.md](validation-android.md), so
"gate 3" means the same thing in both conversations. Every gate line carries
exactly one of **PASS** (with date, machine and the measured number),
**UN-RUN** (with `blocked on:`) or **N/A** — never blank. A gate as a whole
may also be **PARTIAL**, which is not a fourth state for a line: it means the
gate decomposes, and every PARTIAL must name a PASS half with its measured
number and an UN-RUN half with its own `blocked on:`. A bare PARTIAL with no
decomposition is the same as blank.

> **This document has more green than the Android one, and that inversion is
> real.** The reflex is to assume Android is the validated target and web is
> the experiment. It is the reverse: no gate in this repo has ever been run
> on a headset, and the web target is the one that can be exercised without
> one. Do not read a PASS here as evidence about Android.

## Which target do I want?

| You want to | Use |
|---|---|
| Change physics, teleop or frame conventions and know within seconds whether you broke something | Host: `cmake -S . -B build && ./build/teleop_replay build/franka/ar_scene.xml --ref baselines/teleop-franka-host-x86_64.txt` (and the `so101` twin) |
| Check the engine is deterministic on a new architecture | `baseline` on that target vs `baselines/host-x86_64.txt` |
| See the scene, iterate on the renderer or the shell in seconds | Web (this document) |
| Measure real motion-to-photon latency, or ship | Android ([validation-android.md](validation-android.md)) |

## Build

```
scripts/fetch-menagerie.sh
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web --parallel --target mxr baseline teleop_replay
```

**Build named targets only, never `all`.** Upstream's `wasm/CMakeLists.txt`
is added unconditionally under Emscripten and points its output directory at
`${CMAKE_SOURCE_DIR}/wasm/dist`, which under FetchContent is *this* repo.
Building `all` writes upstream's artifacts into the source tree.

**emsdk is pinned to 4.0.10.** It is the version this was built and measured
against, and the version whose `GL.getNewId`/`GL.framebuffers` internals the
opaque-framebuffer bridge in `shell.js` depends on. That bridge uses
Emscripten runtime internals rather than a stable API, so an emsdk bump is a
change that must re-run gate 3, not a routine upgrade.

## Serve and open

```
cd build-web && python3 -m http.server 8000
```

Python 3.12 already returns `Content-Type: application/wasm` for `.wasm`; no
custom server is needed. On a headset, `adb reverse tcp:8000 tcp:8000` (or
`adb tcpip 5555 && adb connect <headset-ip>:5555` first, for wireless) then
open `http://localhost:8000` — **localhost is a secure context, so WebXR
works with no TLS and no certificates.**

Where adb is not available, `scripts/serve-web-tls.py build-web 8443` serves
the same tree over HTTPS with a self-signed certificate carrying the address
in `subjectAltName` — Chromium rejects a CN-only certificate outright rather
than offering the click-through you need. The warning is accepted once per
origin. **UN-RUN on a headset browser:** `blocked on: a device.` The
adb path is the one to trust until that changes.

Neither server compresses, so `mxr.data` is 34 MB on the wire against the
4.98 MB it gzips to. First load over Wi-Fi is slow by that ratio; the browser
cache is what makes the second one fast.

**No COOP/COEP headers are required.** Every WebXR-plus-wasm tutorial says
otherwise, and following one will cost you an hour. Those headers exist to
enable `SharedArrayBuffer` for pthreads; this build sets
`MUJOCO_WASM_THREADS=OFF`, so there is no `SharedArrayBuffer` and no
cross-origin isolation requirement. (MuJoCo compiles meshes serially under
Emscripten regardless of that flag, so nothing is lost.)

Headset-browser logs: `chrome://inspect` on a host machine with the device
over adb. `window.mxr` in that console exposes `{ module, abi }` — a
debugging handle, not a public surface. Filter the console on `mujocoxr`:
shell lines are tagged `[mujocoxr]` and core lines `[mujocoxr I|W|E]`, so one
filter catches both and the level letter tells you which side of the C ABI a
line came from.

## 1. Engine benchmark + invariant check

**Status: PASS** — 2026-07-28, x86_64 host, node 24.18, emsdk 4.0.10.

```
cd build-web && node baseline.js /franka/scene.xml --ref /baselines/host-x86_64.txt
```

```
qpos_linf_diff = 2.1684043449710089e-19  [ok, tol 0.001]
energy_diff = 0 2.2204460492503131e-16  [ok]
motion_check = 0.30379140305321106  [ok, min 0.1]
invariant_check = PASS
```

**Measured L∞ = 2.17e-19.** Interpretation bands, shared with
[validation-android.md](validation-android.md):

| Band | Meaning | Action |
|---|---|---|
| `≤ 1e-9` | expected | PASS, record the number |
| `1e-9 … 1e-6` | libm-only noise (musl vs glibc, ~1 ulp) | PASS, record, note it here |
| `1e-6 … 1e-3` | **the gate reads green and you must not land on it** | STOP, run the ladder below |
| `> 1e-3` | FAIL | STOP |

`kQposTol = 1e-3` is never widened, and there is exactly one reference file.
A `baselines/wasm32.txt` would degrade this gate from "MuJoCo's dynamics are
architecture-independent" to "wasm reproduces wasm", which tests nothing the
port needs tested — and it is a one-way door.

**Diagnostic ladder, if anything exceeds 1e-9.** Do not touch the reference
until a step below exonerates the build:

1. Compare a model checksum host vs target (`qpos0`, `body_mass`,
   `body_inertia`, `geom_size`, `dof_damping`, `nmeshface`, `nmeshgraph`).
   This separates *compiler* divergence (OBJ parse, qhull) from *dynamics*
   divergence. If the checksum differs, the baseline is not the problem.
2. Shorten the horizon: compare at 100 steps and at 1000. Flat at 100 and
   blown at 1000 means growth, not a single-step event.
3. Bisect to the step where L∞ crosses 1e-9 and print `d->nefc` / `d->ncon`
   on both sides. If those differ, that is the finding, and it is a bug
   rather than a tolerance question.
4. Only then: change the excitation so the terminal state is not marginal and
   re-record `host-x86_64.txt` — **as its own explicitly argued commit**,
   never quietly inside another change.

`bench/excitation.h` is frozen. Its `255.0*(1.0-close)` no longer appears in
`teleop.cc` at all — that mapping now reads its endpoints from the robot
table and reduces to this expression only for the Franka — but the two still
round differently (`float` vs `double` operand), so a well-meaning "unify the
gripper mapping" refactor silently invalidates the recorded reference. The
file says so at length; believe it.

**Related measurements, same run:**

| Measurement | Value |
|---|---|
| `mj_loadXML`, wasm under node | **0.58 s** (host serial: 0.34 s) |
| `mxr.data` on the wire, gzipped | 4.77 MB |
| `mxr.wasm` on the wire, gzipped | 1.01 MB |

Load time drives the asset-format decision. At 0.58 s under node — call it
1.7–2.9 s on a Quest Browser — the answer is: ship the XML, no second path.
Revisit only if device-referred load exceeds 20 s, and then as a *client-side
cache* (`mj_saveModel` into Cache Storage, produced and consumed by the same
wasm binary) rather than a shipped `.mjb`, which costs 3× the wire bytes.

## 2. XR skeleton — session, pacing and input

**Status: PARTIAL.** Non-XR startup is **PASS**; the immersive session is
**UN-RUN** — `blocked on: a headset, or a desktop Chrome with the DevTools
WebXR panel.`

**PASS** 2026-07-28, headless Chromium 1194 + SwiftShader — the module
initialises, compiles the scene and reports a clean failure when
`immersive-ar` is absent rather than throwing:

```
[mujocoxr] loading wasm…
[mujocoxr I] MuJoCoXR web starting (MuJoCo 3.10.0)
[mujocoxr] compiling the Franka scene (67 meshes)…
[mujocoxr I] model loaded: nq=9 nv=9 nu=8 nmesh=67
[mujocoxr I] geometry: 396815 vertices, 409806 indices, 67 meshes
[mujocoxr I] frames: t_mj_from_xr = (-1.000 0.000 -0.730) m, clock = XRFrame.predictedDisplayTime
```

**HARD PREREQUISITE — `XRFrame.predictedDisplayTime` must be present.**
Check this first; it gates every later reading on this page. On the first
frame of each session the shell latches exactly one clock and logs it:

```
[mujocoxr] session: clock = XRFrame.predictedDisplayTime, renderState near/far = 0.05/50, viewer y = 1.6xx m
[mujocoxr I] frames: clock = XRFrame.predictedDisplayTime (latched at session entry)
```

If instead you see `clock = requestAnimationFrame t
(XRFrame.predictedDisplayTime absent)` plus a `console.warn`, the UA does not
expose it. **This is expected on WebKit and has historically been true of the
Chrome DevTools WebXR panel**, which is the tool this gate and gate 3c tell
you to use. The fallback is a real, different timebase — latched for the
whole session so the two are never mixed — and every number this page records
must then say which clock produced it.

What must NOT happen, and is now structurally prevented: `undefined * 1e-3`
is `NaN`, `NaN` crosses the C ABI as a double, and every downstream guard
fails open because every NaN comparison is false — the 0.1 s clamp, the
`dt_frame == 0` watchdog, and `rate_slew.h`'s `n > max_step && n > 0` all
take their else branch. Measured on this tree before the fix: an engaged
target took its full 0.400 m goal **in one frame**, with the 1.5 m/s limit
bypassed and nothing in the log. `src/sim_scene.cc` now maps any non-finite
dt to 0, so the existing stalled-clock watchdog names it on every shell:

```
[mujocoxr E] dt has been 0 for 11 frames running — clock source '…' is not advancing
```

**Scene selection, testable in a plain browser tab with no XR device:**

- Opening the page with **no** `?scene=` shows one link per entry in
  `src/robot_spec.c`'s scene table and the status `pick a robot above`, and
  **compiles nothing** — `Enter AR` stays disabled. The links are built
  from `mxr_menu_count/id/label`, so `app/web/index.html` contains no robot
  name at all and a scene cannot exist in C while being unreachable here.
- Clicking a robot navigates to `?scene=<id>` and **reloads**. The reload is
  the teardown: it reclaims the GL context, the wasm heap, the compiled model
  and `SimScene`'s latched clock in one step, which is why there is no
  `mxr_unload_model` and no `Destroy()` in `app/web/scene_renderer.h`.
- `?scene=so101` loads directly. `?scene=nonsense` reports
  `unknown scene "nonsense" — available: franka, so101` rather than failing
  inside `mj_loadXML`; the path is built from the table's copy of the id, not
  from the query string.
- The chosen scene's link is outlined (`aria-current="page"` — they are anchors,
  not buttons; see `shell.js`'s `buildMenu`).

**UN-RUN, requiring a real session** (desktop Chrome → DevTools → ⋮ → More
tools → **WebXR**, pick a device, then Enter AR):

- `immersive-ar` supported and entered.
- **B (`buttons[5]`) ends the session** and returns to the page, where the
  menu is live again. B is bound entirely in `app/web/shell.js` and crosses
  no ABI: ending a session is an `XRSession` operation, and the core has
  nothing to do with it. (The Android shell binds the same button to cycle
  scenes in-process, because it has no page to return to.)
- The granted reference space is `local-floor`. The shell requests it with
  **no fallback list** and hard-fails on throw, because WebXR's fallback is
  `local-floor → local` and `local` sits at head height at session start —
  which lifts the entire scene ~1.6 m while leaving handedness perfectly
  correct, so the axes gizmo still passes. A silent fallback no gate can
  catch is worse than a hard failure. Failure surfaces as
  `could not enter AR: …` on the status line.
- `renderState.depthNear/depthFar` read back as 0.05 / 50, not WebXR's
  default 0.1 / 1000 (which clips the gripper at arm's length). The shell
  **queries** these from wasm (`mxr_near_z`/`mxr_far_z` over
  `src/mesh_buffers.h`) rather than writing literals, and logs both the
  values it sent and the values the UA applied — so a mismatch between the
  two targets' clip planes is visible rather than inferred.
- `viewer y` on the session line is 1.4–1.9 m for a standing user. ~0 means
  the origin is at head height, i.e. a whole-scene 1.6 m offset that the axes
  gizmo cannot catch. This is measurable in the DevTools panel today.
- One-shot input line: `mapping=xr-standard buttons=… profiles=[…]`. A
  mapping other than `xr-standard` logs loudly and binds only the trigger,
  because button indices are not guaranteed otherwise.
- rAF pacing at the display refresh. **Only
  `XRSession.requestAnimationFrame` is used, never the window queue** — both
  would double-step physics. Exactly one clock is latched per session (see
  the prerequisite above); mixing the two timebases would inject a 20–50 ms
  offset that survives the 0.1 s clamp as a real dt, worth up to 15 extra
  steps and a 4.5 cm slew jump.
- Zero console errors and zero `gl.getError()` over 10 s.

**Input characterization is headset-only** — `blocked on: a physical
controller.` The DevTools panel supplies synthetic 0/1 buttons, which is a
useful negative test but cannot tell you whether a real full squeeze
saturates below 0.8. See gate 2 in the Android document; the thresholds are
shared.

## 3. Render + handedness (BEFORE trusting any teleop motion)

**Status: PARTIAL.** Arithmetic, the framebuffer bridge and mono rendering
are **PASS**; stereo and anything physical are **UN-RUN**.

**PASS** 2026-07-28, host x86_64 and wasm32 — frame convention, on every
build via `bench/teleop_replay`:

```
frames: 1 m fwd -> MJ +x           [ok]
frames: XR -Z/+Y/+X axis map       [ok]
frames: mat4 inverts the point map [ok]
```

**PASS** 2026-07-28, headless Chromium + SwiftShader — the
opaque-framebuffer bridge, which was the highest-risk unknown in this port.
WebXR hands JS an opaque `WebGLFramebuffer` that Emscripten's GL layer never
created, so C has no integer name for it. `shell.js` registers it via
`GL.getNewId(GL.framebuffers)`; a framebuffer registered that way was bound
from wasm and read back as the current binding. A 512×512 draw through it
produced the Franka with correct shading and the +z gizmo upright.

- Geometry in the browser is identical to host: 396,815 vertices, 409,806
  indices, 67 meshes.
- **This is the check inline XR cannot do.** An inline session has
  `layer.framebuffer === null`, so `fbo` is 0 and the bridge is never
  exercised. Any automated inline test would structurally miss it.

**UN-RUN, DevTools WebXR panel** — `blocked on: a desktop Chrome session.`

- Two viewports, not swapped, and **cleared once before the view loop, not
  per view.** (`mxr_begin_frame` / `mxr_draw_view` makes the per-view clear
  unrepresentable, but confirm the picture.)
- Head-motion parallax, and the scene stationary while the head moves —
  catches `view.transform.matrix` being used where `.inverse.matrix` belongs.
- Axes gizmo colours in the expected screen quadrants.

**UN-RUN, headset only** — `blocked on: a physical headset.`

- Table legs meet the physical floor: ±3 cm is invisible, > 5 cm is a fail.
- **Stand still and look down: the ground plane is at your feet, not your
  waist.** The gizmo cannot catch a whole-scene vertical offset.
- Passthrough alignment and stereo comfort.

**A wrong floor height has three causes that present identically:**

| Symptom | Cause | Fix |
|---|---|---|
| Scene ~1.6 m high, gizmo correct | The granted reference space is not floor-origin | Check the console: the shell refuses to enter rather than fall back, so this should be impossible here — if you see it, that guarantee broke |
| Scene off by ~0.2 m | The UA's `local-floor` floor estimate differs from the OpenXR runtime's on the same device | Compare against Android on the same headset; record both |
| Scene off by the table height | `MXR_T_MJ_FROM_XR.z` is wrong for this deployment | Edit `src/frames.h` and rebuild — it is a workspace **calibration**, expected to differ per room |

`MXR_Q_MJ_FROM_XR` is the other kind of constant: a handedness **convention**
that cannot be wrong "for this room". Wrong gizmo means that is the bug;
wrong height only, and it is not.

## 4. Teleop acceptance

**Status: PARTIAL.** Semantics are **PASS** headlessly; acceptance is
**UN-RUN** — `blocked on: a physical headset and controller.`

**PASS** 2026-07-28, host x86_64 and wasm32:

```
teleop: engage pos jump bitwise 0     [ok]
teleop: engage quat jump <= 1e-12 rad [ok]   worst 2.22e-16 rad (host)
                                             worst 1.09e-16 rad (wasm32)
teleop: slew within rate limits       [ok]   worst 0.24 of the bound
ref: geometry census matches          [ok]
ref: triangle set matches             [ok]
```

The orientation half is measured as an **angle**, via `mju_subQuat` — the
same sign-invariant metric the slew check uses. Raw components would not
survive the quaternion double cover: `q` and `−q` are the same rotation but
differ component-wise by up to 1.41, so a reformulation returning `−q_t0` —
physically identical, genuinely zero-jump — would turn the gate red. The
`1e-12 rad` bound is anchored, not chosen for headroom: a Franka joint
encoder step is ~2e-5 rad and one frame of full-rate slew is 4.17e-2 rad, so
nothing a bug or a human can produce sits under it.

The geometry census and the dereferenced-triangle checksum are identical on
host and wasm32 — mesh parsing and qhull agree bit-for-bit across
architectures. The **trace** hash is deliberately host-only: it is a bitwise
lock on the refactor, and 360 frames of IK and physics amplify the 1e-19
per-step divergence gate 1 measures and tolerates. Running `--ref` off-host
skips that one comparison and says so.

Headset-only acceptance is the same list as
[validation-android.md](validation-android.md) gate 4, including the grip
lever-arm check and the visibility-resume check. Two web-specific notes:

- Losing tab visibility reports `grip_valid = false`, so the clutch
  auto-disengages through the same path as lost tracking. The accumulator's
  catch-up cap handles the dt side of the resume; nothing handles a pose
  jump, which is why the clutch drops.
- The catch-up cap is `2*0.014/timestep + 1 = 15` steps — a **latency bound
  expressed in absolute seconds**. 0.014 s was one frame at 72 Hz, so on a
  90 Hz browser it silently becomes 2.7 frames rather than 2. Record the
  measured effective bound here once a headset runs it: **UN-RUN.**

## 5. Soak

**Status: UN-RUN** — `blocked on: a headset session.`

10 minutes in-session, re-running gates 3 and 4. Watch
`window.mxr.module.HEAP8.length` flat after the first minute, JS heap flat,
and frame-period p99 stable. Thermal behaviour is device-only.

## Symptom → cause

| Symptom | Likely cause | Where to look |
|---|---|---|
| Blank page, no status text | The ES module failed to load; `mxr.js` is a module and needs `type="module"` | Browser console |
| `this page is not a secure context…` | Served over plain `http://` from a non-localhost host. `navigator.xr` is undefined on an insecure origin, which looks identical to "no WebXR" — the shell distinguishes them so you do not go install a different browser | `adb reverse tcp:8000 tcp:8000` + `http://localhost:8000`, or `scripts/serve-web-tls.py` for HTTPS |
| Scene renders, physics frozen, no errors, `clock = requestAnimationFrame t` | The UA has no `XRFrame.predictedDisplayTime`; expected on WebKit | Gate 2's hard prerequisite |
| `model load failed: mj_loadXML failed …` | The staged tree is missing or flattened; `meshdir="assets"` needs the subdirectory intact | `build-web/franka/`, and the `file(COPY …)` block in CMakeLists.txt |
| All 67 meshes fail to load, link was clean | MuJoCo not linked `--whole-archive`; `obj_decoder`/`stl_decoder` self-register from file-scope constructors and get dropped | `MXR_MUJOCO` in CMakeLists.txt |
| Crash or corruption during mesh compile | Stack overflow: emcc defaults to 64 KB and qhull over 396,791 vertices needs more | `-sSTACK_SIZE=4MB` |
| `#version directive must occur on the first line` | The shader embedder emitted a leading newline | `cmake/embed_shader.cmake` |
| One eye black, the other correct | Clearing inside the view loop | Should be structurally impossible; check `mxr_begin_frame` |
| Scene renders but never moves; no errors | dt is always 0 — the clock stopped | `dt has been 0 for 11 frames running` names the latched source |
| Physics lurches after re-entering the session | The clutch stayed latched or the accumulator kept the gap | `mxr_end_session` should fire on the session `end` event |
| `recenter_edge asserted 4 frames running` | The shell is wiring a level where an edge is required | `shell.js`'s `recenterEdge` must be cleared each frame |
| Entire scene ~1.6 m too high | A non-floor reference space was granted | Should be impossible — entry hard-fails; see gate 3 |
| Gripper never closes | The robot's tabulated `gripper_closed`/`gripper_open` are outside the model's `actuator_ctrlrange` | `gripper endpoints (closed …, open …) fall outside '…' ctrlrange` at startup |
| Translucent marker washed out over passthrough | The premultiplied-alpha question, unresolved on **both** targets | See the alpha row in validation-android.md gate 3 — fix both together |
| The page loads a stale `shell.js` | Browser cache; the build copies it correctly | Hard-reload |

## Deliberately not here

No `package.json`, no npm, no bundler, no TypeScript, no Playwright in the
repo. CMake fetches and pins every dependency, and a second dependency
manager with no relationship to the build is a cost with no matching benefit
— particularly for browser tests, where the one thing worth automating
(the framebuffer bridge) is exactly the thing an inline session cannot reach.
Reopen when CI exists, or when a headset has passed gate 3 once.
