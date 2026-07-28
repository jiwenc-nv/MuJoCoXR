// The two tables that make this app multi-robot, and the only place a robot
// or a scene is named.
//
// WHAT IS HERE: `MxrRobot` — one row per arm, holding model-file names plus
// the tuned constants the teleop stack cannot derive — and `MxrScene` — one
// row per loadable scene, holding an id and a menu label, and nothing else.
// THERE IS NO FOREIGN KEY BETWEEN THEM. A scene does not name its robot and a
// robot does not list its scenes; the link is made at load time by probing the
// model (mxr_robot_probe). That absence is deliberate and is what the rest of
// this comment argues for.
//
// They are TWO tables on purpose, and the reason is the change most likely to
// come next. A scene is not a robot: /code/Lab already ships a second SO101
// scene (Isaac-Stack-Cube-SO101-IK-Abs-v0), so the third entry this repo
// grows is more likely a second SO101 scene than a third arm. Under one fused
// table that would mean a duplicated robot row — and because the robot is
// PROBED from the model rather than named (see mxr_robot_probe), a duplicated
// row makes `tcp_body` stop discriminating and turns the "exactly one match"
// rule into a hard failure at load. One table would make the likeliest next
// change brick the app; two tables make it one MxrScene row.
//
// Neither array is exported. They are `static const` inside robot_spec.c and
// reachable only through the accessors below, so nothing can hold a pointer
// into a table it can also index past.

#ifndef MUJOCOXR_SRC_ROBOT_SPEC_H_
#define MUJOCOXR_SRC_ROBOT_SPEC_H_

#include <mujoco/mujoco.h>

#ifdef __cplusplus
extern "C" {
#endif

// Capacity of every per-arm array in the tree, sized by the widest arm in the
// table (the Franka's 7). It is a CAPACITY, not a count: the actual number of
// arm joints is IkDls::narm, from mxr_robot_narm() below, and the SO101 is 5.
//
// AN 8-DOF ARM NEEDS THIS RAISED, and that is one number and nothing else.
// What is sized off it: MxrRobot::joint here, and IkDls's dofadr / qposadr /
// act / ctrl_lo / ctrl_hi / qhome — all in-struct, no allocation, every loop
// bounded by `narm`. It is left at the widest shipped arm rather than padded
// because AN OVER-LONG ROW CANNOT FAIL SILENTLY: a `.joint` initialiser with
// more entries than the array is a hard compile error in C ("excess elements
// in array initializer") pointing at the offending row, so the person who
// needs the bump is told by the compiler, at the line they are editing,
// before anything runs.
enum { MXR_MAX_ARM = 7 };

// One arm, named entirely in model-file terms plus the numbers the teleop
// stack cannot derive. Every field is either a name resolved through
// mj_name2id or a tuned constant; nothing here is a resolved id, because this
// struct is `static const` and ids are per-model. Resolution lands in IkDls.
typedef struct {
  // THE DISCRIMINATOR. mxr_robot_probe identifies the loaded model by looking
  // up this one body name, so it must be unique across the whole table. It is
  // also the frame ik_dls_tcp reports: the TCP pose is this body's pose
  // offset by tcp_offset, so `tcp_body` chooses the tool ORIENTATION as well
  // as the position.
  //
  // The two robots' tool frames DO NOT AGREE, and that is fine only because
  // of an invariant stated at src/teleop.cc's orientation block. Measured at
  // each robot's home: the Franka's `hand` frame and the SO101's `gripper`
  // frame are 135.85 deg apart, and the SO101's own authored tool site
  // (`gripperframe`) is exactly 90.0000 deg about +y from the `gripper` body
  // frame used here. Nothing corrects for either, because nothing has to —
  // the teleop orientation path is purely relative.
  const char* tcp_body;
  // Arm joints, base to tool. Order is load-bearing: it is the column order
  // of the Jacobian and the row order of dq. Fill a LEADING RUN of slots and
  // leave the rest NULL — a gap ({"a", NULL, "c"}) is not supported and would
  // silently truncate the arm, because the count is mxr_robot_narm(), which
  // measures the leading run. Note also that a row using all MXR_MAX_ARM slots
  // (the Franka does) has no NULL terminator at all, so nothing may scan for
  // one.
  const char* joint[MXR_MAX_ARM];
  // Position-servo actuator for the jaw. Named rather than found by position:
  // the Franka's is the 8th actuator and the SO101's is the 6th, and "the one
  // after the arm joints" is a coincidence of both models rather than a rule.
  const char* gripper_act;

  // TCP position in the tcp_body frame, in METRES: the grasp midpoint, so the
  // operator's clutch pivots about the point between the jaws rather than a
  // servo housing.
  //
  // WHERE THE NUMBER COMES FROM, because the two shipped rows got it by
  // different routes and the difference is the part to get right:
  //   - SO101: copied verbatim from so101.xml's authored `gripperframe` site
  //     `pos`. That works only because the site is a direct child of the
  //     `gripper` body, so its `pos` already IS an offset in this frame. Its
  //     ORIENTATION is 90 deg off (see tcp_body) and is not read here.
  //   - Franka: no authored site sits at the grasp midpoint, so 0.103 m along
  //     the +z tool axis was taken off panda.xml's finger geometry.
  // So: use a tool site's `pos` if one hangs under tcp_body; transform it
  // first if the site hangs under a different body; measure the midpoint
  // between the jaw geoms if there is no site at all. Verify whichever route
  // you took the way the SO101's was verified — forward-kinematic the offset
  // at the `home` keyframe and compare against the site's world position
  // (that check matched to 4e-17 m).
  mjtNum tcp_offset[3];

  // Weight applied to the three ROTATION rows of the task error and Jacobian.
  // 1.0 means "1 rad of orientation error is worth 1 m of position error".
  //
  // MEASURED, NOT DERIVED, and the distinction is the useful part. Two
  // candidate derivations were built and both were killed by the same test:
  // tool length gives 0.098 for the SO101, and sigma_min(J_pos)/sigma_max(J_rot)
  // gives 0.0495 — a 1 % match to the measured 0.05 — but that same expression
  // predicts 0.177 for the Panda, whose measured optimum is 1.0 and monotone.
  // A formula that is right on one arm and 5.6x wrong on the other is a fit.
  //
  // The statable rule, which extends to robot #3 better than a formula would:
  // on a FULL-RANK arm the optimum is 1.0, because the natural metric is
  // achievable and any weight is a distortion — so a 6-dof arm needs no sweep
  // at all. On a RANK-DEFICIENT arm the optimum is wherever unachievable
  // orientation stops corrupting achievable position, and that boundary
  // depends on the distribution of commanded residuals, not on the Jacobian.
  // It is a property of the task, not of the kinematics. Start a sweep at the
  // tool length and walk down to the knee.
  mjtNum w_rot;

  // Damping in the DLS solve: A = J J' + lambda^2 I6. Its units are those of a
  // singular value of the weighted J — metres of TCP motion per radian of
  // joint motion — so it is comparable against the sigmas quoted in the rows
  // below, and that comparison is the whole tuning story: a task direction is
  // attenuated by lambda^2/sigma^2, so raising lambda makes the arm calmer
  // near a singularity and lazier everywhere else.
  //
  // Both shipped rows use 0.05, and 0.05 is where a third robot should start.
  // To check it, sweep 0.02 -> 0.10 and read `pos_med_mm` off
  // bench/teleop_replay. If the median is flat across that sweep the arm is
  // not damping-limited in motion and the default stands; only an observed
  // jitter or stall at full extension argues for moving it. That sweep was run
  // on the SO101 (3.2 -> 3.3 mm, non-monotone) and is why its row keeps the
  // Franka's value — see the note there, which also records the one number
  // that would justify lowering it.
  mjtNum lambda;

  // Gain on the nullspace bias toward the `home` KEYFRAME posture — the same
  // keyframe ik_dls_init refuses the model without. 0 disables the term.
  //
  // MUST BE 0 UNLESS narm > 6, and "6" is the trap: a nonsingular 6-dof arm on
  // a 6-D task has dim N(J) = 0, so there is no nullspace to bias and the
  // entire term lands on the task command as uncommanded tool motion. The rule
  // is narm > 6, NOT "6 or more" — and the likeliest third robot in this
  // ecosystem is exactly the 6-dof case that reads as safe and is not.
  // Counted, not recalled, because an earlier draft of this comment cited two
  // arms that do not fit: of the arms Menagerie actually ships, UR5e, UR10e
  // and ufactory_lite6 are the 6-dof ones this rule catches, while kinova_gen3
  // (7) and ufactory_xarm7 (7) are genuinely redundant and want a non-zero
  // gain. Check narm against the model rather than the marketing name — the
  // same product line ships in both widths. Measured by forcing narm = 6 on
  // the Franka: ns_gain = 0.1 takes the position error 21.255 -> 31.958 mm and
  // the spurious rotation 0.507 -> 0.979 deg, buying nothing.
  //
  // src/ik_dls.c's nullspace block states the mechanism (it computes a DAMPED
  // projector, which is not a projector); the SO101 row in robot_spec.c
  // records what the leak measures on a real arm.
  mjtNum ns_gain;
  // Control-display ratio for the clutch: target moves clutch_scale metres
  // per metre of hand travel. An operator human-factors parameter, and on a
  // short arm a reach limiter — see robot_spec.c.
  mjtNum clutch_scale;

  // Jaw endpoints in actuator-ctrl units, TABULATED rather than derived from
  // actuator_ctrlrange. Deriving would silently follow a Menagerie bump;
  // tabulating turns one into a warning (src/teleop.cc checks that both
  // endpoints still lie inside ctrlrange). `closed` may be numerically above
  // `open` — polarity lives here, not in the mapping.
  mjtNum gripper_closed;
  mjtNum gripper_open;
} MxrRobot;

// One loadable scene. Distinct from MxrRobot because two scenes can share an
// arm; see the header comment.
typedef struct {
  // THE ONLY NAME. `id` is simultaneously the CMake staging directory
  // (build/<id>/), the APK asset directory (assets/<id>/), the wasm MEMFS
  // mount point (/<id>/ar_scene.xml), the web `?scene=` token and the
  // baselines/teleop-<id>-host-x86_64.txt stem. That convention is what lets
  // the shells build a path from a table row instead of carrying their own
  // per-robot literals.
  //
  // THIS FILE IS AUTHORITATIVE. CMakeLists.txt's mxr_stage_scene() and
  // package-apk.sh's stage_scene() each repeat the id, and no build step can
  // check them against this one; a disagreement surfaces at load as
  // mj_loadXML failing on a path that names the id (mxr_last_error on web,
  // logcat on Android). That failure mode is why the id crosses as a string
  // rather than an index — an index would silently load the wrong robot.
  const char* id;
  // Shown to a human. The one UI string in the tree: the web menu buttons and
  // the Android B-cycle's `scene:` log line, which on a headset with no text
  // rendering is the ONLY way to learn what you just switched to.
  const char* label;
} MxrScene;

// Identify the loaded model. Looks up every table row's `tcp_body` and
// requires EXACTLY ONE to resolve; returns NULL otherwise, with *why pointing
// at a static string naming how many rows matched and which.
//
// The robot is probed and never named: no --robot flag, no robot id crossing
// any boundary, and no way for a caller to assert a robot the model is not.
// Measured on the shipping table: the two ROWS share zero names — 0 of the 12
// joint names they carry between them (7 + 5) and 0 of the 2 actuator names —
// so `tcp_body` alone separates them. (The MODELS carry 9 and 8 named joints
// and actuators respectively; the rows name only the arm.)
//
// `why` may be NULL. If it is not, it is ALWAYS written: NULL on success, and
// on failure a pointer to a static buffer private to this function — it is not
// shared with mxr_last_error(). That buffer is overwritten by the next
// mxr_robot_probe call and this is not thread-safe, so log the string before
// probing again.
const MxrRobot* mxr_robot_probe(const mjModel* m, const char** why);

// Number of arm joints in a row: the length of the leading run of non-NULL
// `joint` entries, capped at MXR_MAX_ARM.
int mxr_robot_narm(const MxrRobot* r);

// The scene catalogue, in menu order. mxr_scene_at returns NULL out of range;
// mxr_scene_by_id returns NULL for an unknown id (which is a user-supplied
// `?scene=` token, so it must be handled, not asserted).
int             mxr_scene_count(void);
const MxrScene* mxr_scene_at(int i);
const MxrScene* mxr_scene_by_id(const char* id);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCOXR_SRC_ROBOT_SPEC_H_
