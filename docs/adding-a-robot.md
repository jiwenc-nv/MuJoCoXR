# Adding a robot

The tree ships two: a Franka Emika Panda (7 dof) and an SO-101 (5 dof). This
is the checklist for a third, and the design claim it is testing is that the
list below is *complete* — if your robot needs an edit to a file not named
here, the design was wrong for it and that is worth saying out loud in the
commit rather than working around.

**There are two procedures.** Adding a *robot* is the six steps below. Adding
a second *scene for a robot the tree already has* is shorter, different, and
is the change this design was actually built for — see "A second scene for an
existing robot" further down. **Do not follow the six-step list for that
case**: it would add a duplicate `kRobots` row, and two rows whose `tcp_body`
both resolve is a hard failure at load.

## The checklist (a new robot)

1. **`scripts/fetch-menagerie.sh`** — append the Menagerie directory to
   `DIRS`. The early-exit and the sparse-checkout set both derive from that
   one list, so nothing else in the script changes. An existing checkout
   widens automatically because the early-exit tests *every* directory.

2. **`assets/<id>/ar_scene.xml`** — the AR composition: `<include>` the robot,
   add a table sized to its reach, and author a keyframe named `home`.
   `src/ik_dls.c` requires `home` and will refuse the model without it. The
   table's **top must sit at z = 0** with the robot base on it: `src/frames.h`
   ships one floor calibration for every scene and that plane is what makes it
   shared. Copy `assets/so101/ar_scene.xml` and read its comments first — both
   of them are decisions, not description.

3. **`src/robot_spec.c`** — one row in `kRobots` and one in `kScenes`. Fill
   the tuned numbers by reading the field comments in `src/robot_spec.h`;
   every one of them carries its units, its provenance and how to measure it.
   This file is **authoritative for the scene id**; see below.

4. **`CMakeLists.txt`** — one `mxr_stage_scene(<id> <menagerie_dir> ...)` call.
   The wasm preload lines are a `foreach` over the staged ids and need no edit.

5. **`app/android/package-apk.sh`** — one `stage_scene <id> <menagerie_dir>`
   line. (Genuinely one: the fetch guard lives inside `stage_scene`, so there
   is no second list of Menagerie directories in that file to keep in sync.)

6. **`baselines/teleop-<id>-host-x86_64.txt`** — record it, and restore the
   comment header by hand (`teleop_replay` does not emit it):

   ```
   ./build/teleop_replay build/<id>/ar_scene.xml > baselines/teleop-<id>-host-x86_64.txt
   ```

Nothing else. In particular **no edit** to `src/ik_dls.{h,c}`,
`src/teleop.{h,cc}`, `src/sim_scene.{h,cc}`, `src/frames.h`, `app/web/abi.h`,
`app/web/shell.js`, `app/web/index.html`, `app/web/main.cc`,
`app/android/AndroidManifest.xml`, `app/android/main.cc` or
`app/android/assets.cc`.

That claim is about a new **row**. A new **field** on `MxrScene` is a
different matter: the web ABI's `mxr_menu_*` accessors are hand-written
per-field twins, so a fourth `MxrScene` field needs a fourth export in
`app/web/abi.h` and `app/web/main.cc`. Nothing in this checklist needs one.

## A second scene for an existing robot

This is the likeliest next entry — `/code/Lab` already ships
`Isaac-Stack-Cube-SO101-IK-Abs-v0`, a second SO-101 scene — and it is why
`kRobots` and `kScenes` are two tables with no foreign key between them.

Do steps 2, 4, 5 and 6, and in step 3 add **only** the `kScenes` row.

- **No `kRobots` row.** The arm is unchanged, so its tuned numbers are
  unchanged, and `mxr_robot_probe` will match the existing row by `tcp_body`.
  A duplicate row would make two rows resolve against the same model, which
  is a hard failure at load naming both — the failure the two-table split
  exists to avoid.
- **It does need its own baseline.** The reference is per *scene*, not per
  robot: it records the geometry census, the triangle checksum and the
  `opt_*` block, all of which move when the table or the props move, and a
  `pos_med_mm` that depends on where the scene puts things relative to the
  scripted trajectory. Record it the same way as step 6.

## The id is the only name

`MxrScene::id` is simultaneously the CMake staging directory (`build/<id>/`),
the APK asset directory (`assets/<id>/`), the wasm MEMFS mount (`/<id>/`) and
the web `?scene=` token. Those four are **mechanised** — the build and the
shells derive the path from the table row.

The baseline filename stem, `baselines/teleop-<id>-host-x86_64.txt`, is
**not**: nothing derives it. It is a `--ref` argument a human types, and it
follows the convention only because everyone keeps doing so.

The id is written down in three places — `src/robot_spec.c`, `CMakeLists.txt`
and `package-apk.sh` — and **`src/robot_spec.c` is authoritative**; the other
two exist to put bytes where it says they are. No build step can check them
against each other. A disagreement therefore surfaces at *load*, as a named
path: `mxr_last_error()` on the web, `assets/<id> not found in APK` on
logcat. That is also why the id crosses as a string and never as an index — an
index would silently load a different robot after a table reorder, and
`?scene=` is persisted outside the process in bookmarks and reloads.

## The robot is probed, never named

There is no `--robot` flag and no robot id in any signature.
`mxr_robot_probe()` looks up every row's `tcp_body` against the loaded model
and requires **exactly one** to resolve. So **`tcp_body` must be unique across
the table.** This is the one hard constraint a new row can violate. Two rows
that both resolve is a hard failure at load naming both, not a silent pick.

## Picking the tuned numbers

Every tuned field documents its own units, provenance, default and
measurement procedure at its declaration in **`src/robot_spec.h`**, and the
two shipped rows in `src/robot_spec.c` carry the sweeps they were chosen
from. That is the reference; it is not repeated here. Three things are worth
knowing before you open it:

- **`w_rot` is the one that matters and has no formula.** Two candidate
  derivations were built and both were killed by measurement. Start at the
  tool length, sweep down, stop at the knee, and record the sweep *and the
  trajectory it was measured on* in the row's comment.
- **`ns_gain` must be 0 unless the arm has more than 6 joints.** Not "6 or
  more" — **more than 6**. A nonsingular 6-dof arm on a 6-D task has no
  nullspace at all, so the whole home bias lands on the task command as
  uncommanded tool motion. Of the arms Menagerie ships, UR5e, UR10e and
  ufactory_lite6 are the 6-dof ones this rule catches; kinova_gen3 and
  ufactory_xarm7 have 7 and want a non-zero gain. Count the arm joints in the
  model — the same product line ships in both widths, so the name will not
  tell you.
- **Near full extension the IK folds**, on any arm, and a low `w_rot` makes
  the tail worse rather than better: at the reach limit the arm cannot
  satisfy position either, and a low `w_rot` stops it trading orientation
  away to try. On the SO-101 that measures ~155 mm worst-case at `w_rot =
  0.05` against ~40 mm at 0.10, over random reachable targets — a workspace
  the shipped gate script never enters. **This is expected behaviour, not an
  IK bug.** Nothing in the tree fixes it; the domain-correct fix is a
  manipulability clamp on the *target*, which nobody has written.

## The gates

```
cmake --build build --parallel
./build/teleop_replay build/<id>/ar_scene.xml --ref baselines/teleop-<id>-host-x86_64.txt
```

Each scene has its own reference. Two of the recorded fields are
architecture-independent and are therefore checked on **every** target, while
`trace_fnv1a` is a bitwise lock skipped off the recording architecture:

- `opt_timestep` / `opt_integrator` / `opt_cone` / `opt_impratio`
- `pos_med_mm`, against a 1.5x + 1 mm tolerance

Note that `pos_med_mm` **ratchets**: it is a regression check against the
recorded value, so re-recording after a 2x regression bakes that regression
into the new bound. It is not an absolute quality bar, and it cannot be one —
the Franka's shipped figure is 23.69 mm and the SO-101's is 2.23 mm, because
the Franka's `clutch_scale` sends the script twice as far.

`bench/baseline.cc` and `bench/testspeed.cc` stay Franka-only by design; see
the `nu < 8` comment in `baseline.cc` for why generalising them is a separate
commit.

## Placing the robot somewhere other than the origin

Both shipped wrappers put the robot at the world origin. If a headset session
reports that the arm reads too small or too far away, the verified recipe —
`<attach>` inside a `<frame pos>`, with `<compiler conflict="merge"/>` to stop
MuJoCo 3.10.0 silently discarding the child model's `<option>` block — is
recorded with its three measured constraints and its trigger in
**`assets/so101/ar_scene.xml`**, at the top of the file, where someone editing
a wrapper will actually meet it. It is measured and deliberately unused; read
it there before reaching for `<attach>`.
