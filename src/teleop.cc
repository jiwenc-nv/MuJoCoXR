#include "teleop.h"

#include <cmath>

#include "frames.h"
#include "mxr_log.h"
#include "rate_slew.h"

namespace {

// SHARED ACROSS ROBOTS, not a src/robot_spec.c column, and that survived a
// serious attempt to make it one. A 0.30 m/s cap for the SO101 was proposed,
// measured, and withdrawn by the persona who proposed it: against a realistic
// operator sweep (0.5 Hz, +-0.10 m, peak 0.31 m/s) it makes mean lag 2.2x
// WORSE — 4.9 mm against 2.2 mm at 1.5 — because a rate limiter can only ADD
// lag to a signal already below the cap. It helps only on discontinuous jumps,
// and the clutch latch already removes those by construction.
//
// The finding that motivated the cap is real and is NOT addressed here:
// torque saturation runs at 39-43 % of frames on the SO101. But it is at
// 39-43 % at EVERY cap including 0.20, because saturation is a servo and
// gravity property, not a command-rate one. The cap was the wrong instrument;
// MxrRobot::clutch_scale is the one that actually moved this arm's numbers.
//
// These two are also duplicated in bench/teleop_replay.cc, deliberately, and
// its SlewWatch only checks an UPPER bound. So a per-robot rate would have to
// be a TIGHTENING, which that gate passes vacuously. TRIGGER, if a human runs
// the SO101 and reports lag: max_lin_rate becomes a table column AND
// bench/teleop_replay.cc grows a second bound. Do not add one without the
// other.
constexpr mjtNum kMaxLinRate = 1.5;   // m/s target rate limit
constexpr mjtNum kMaxAngRate = 3.0;   // rad/s target rate limit
constexpr float kEngageThreshold = 0.8f;   // squeeze hysteresis
constexpr float kReleaseThreshold = 0.6f;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

bool Teleop::Init(const mjModel* m, const mjData* d) {
  // Every member back to its default initialiser, in one statement that
  // cannot drift from the member list. Init is now a RE-init: the Android
  // shell destroys and rebuilds the whole scene to switch robots, so this
  // object is initialised over a used one. The three members Init used to
  // write left engaged_, a_down_prev_, recenter_run_, frame_ and the two
  // latched engage poses from the PREVIOUS robot — a scene switch made
  // mid-clutch would have resumed engaged over a stale p_c0_.
  *this = Teleop{};

  const char* why = "(no reason reported)";
  if (ik_dls_init(&ik_, m, &why) != 0) {
    // Log `why`, never just the failure: a false return leaves
    // SimScene::teleop_ready_ false, and a not-ready SimScene still renders.
    // The symptom is a robot that draws perfectly and never moves.
    LOGE("ik_dls_init failed: %s", why);
    return false;
  }
  LOGI("teleop: robot '%s', %d arm joints, w_rot=%g clutch_scale=%g",
       ik_.spec->tcp_body, ik_.narm, ik_.spec->w_rot, ik_.spec->clutch_scale);

  // The jaw endpoints are TABULATED (src/robot_spec.c), not derived from
  // actuator_ctrlrange: ctrlrange supplies the scale but not the polarity,
  // and both shipped robots are "low = closed" only by coincidence —
  // Menagerie's Robotiq 2F-85 is 0..255 with 0 = OPEN. Check the tabulated
  // endpoints still lie inside the model's range rather than following it, so
  // a Menagerie bump becomes a warning instead of a silent re-scaling. Without
  // this the failure is silent: MuJoCo clamps and the jaw stops at the wrong
  // place, or never closes at all.
  const mjtNum* r = m->actuator_ctrlrange + 2*ik_.gripper_act;
  if (m->actuator_ctrllimited[ik_.gripper_act] &&
      (ik_.spec->gripper_closed < r[0] || ik_.spec->gripper_closed > r[1] ||
       ik_.spec->gripper_open < r[0] || ik_.spec->gripper_open > r[1])) {
    LOGW("gripper endpoints (closed %g, open %g) fall outside '%s' ctrlrange "
         "(%g, %g): src/robot_spec.c disagrees with this model",
         ik_.spec->gripper_closed, ik_.spec->gripper_open,
         ik_.spec->gripper_act, r[0], r[1]);
  }
  ik_dls_tcp(&ik_, d, target_pos_, target_quat_);
  return true;
}

void Teleop::Reset(const mjModel* m, mjData* d) {
  int home = mj_name2id(m, mjOBJ_KEY, "home");
  if (home >= 0) {
    mj_resetDataKeyframe(m, d, home);
  }
  mj_forward(m, d);
  ik_dls_tcp(&ik_, d, target_pos_, target_quat_);
  engaged_ = false;
  LOGI("teleop: home reset");
}

void Teleop::Update(const mjModel* m, mjData* d, const InputState& input,
                    double dt) {
  ++frame_;
  // A is reported as a raw level; the edge is derived here so every shell
  // shares one definition. Suppressed on the first frame so a button already
  // held at session start does not fire a reset.
  const bool a_edge = input.a_down && !a_down_prev_ && frame_ > 1;
  a_down_prev_ = input.a_down;
  if (a_edge) {
    Reset(m, d);
  }

  // recenter_edge is a shell-latched one-frame event. A shell that wired a
  // level here would keep the clutch permanently disengaged with nothing
  // logged; this turns that into a named diagnosis.
  if (input.recenter_edge) {
    if (++recenter_run_ == 4) {
      LOGW("recenter_edge asserted 4 frames running — the shell is reporting "
           "a level, not an edge");
    }
  } else {
    recenter_run_ = 0;
  }

  // Gripper is direct: trigger 0 -> open, 1 -> closed, affine between, with
  // both endpoints from the robot's table row. Written as
  // `closed + span*(1 - t)` rather than the algebraically equal
  // `open + t*(closed - open)` for a bitwise reason, not a stylistic one: the
  // former reduces to the Franka's original `255.0*(1.0 - t)` exactly (span
  // is 255.0, closed is +0.0), so the golden trace is unmoved. The latter
  // rounds differently and would have forced a re-record.
  //
  // bench/excitation.h carries `255.0*(1.0 - close)`, which is what the line
  // below reduces to for the Franka's (0, 255) endpoints, and it must NOT be
  // unified with it — `t` here is float and `close` there is double, so the
  // two round differently, and the recorded baseline was taken through that
  // one. The full warning lives there.
  {
    float t = input.trigger < 0 ? 0 : (input.trigger > 1 ? 1 : input.trigger);
    const mjtNum closed = ik_.spec->gripper_closed;
    d->ctrl[ik_.gripper_act] =
        closed + (ik_.spec->gripper_open - closed)*(1.0 - t);
  }

  if (input.recenter_edge || !input.grip_valid) {
    // Recenter moves the reference space under the controller; lost tracking
    // jumps the pose on regain. Either way: drop the clutch, hold the target.
    if (engaged_) {
      LOGI("teleop: clutch auto-disengaged (%s)",
           input.recenter_edge ? "recenter" : "tracking lost");
    }
    engaged_ = false;
  } else {
    // Controller pose into MuJoCo world before any delta is formed (a delta
    // formed in the XR reference space differs by conjugation with R).
    mjtNum p_c[3], q_c[4];
    mxr_pos_mj_from_xr(input.grip_pos, p_c);
    mxr_quat_mj_from_xr(input.grip_quat, q_c);

    if (!engaged_ && input.squeeze > kEngageThreshold) {
      engaged_ = true;
      mju_copy(p_c0_, p_c, 3);
      mju_copy(q_c0_, q_c, 4);
      mju_copy(p_t0_, target_pos_, 3);
      mju_copy(q_t0_, target_quat_, 4);  // zero engage jump by construction
    } else if (engaged_ && input.squeeze < kReleaseThreshold) {
      engaged_ = false;
    }

    if (engaged_) {
      mjtNum goal_pos[3], q_c0_inv[4], q_delta[4], goal_quat[4];
      for (int i = 0; i < 3; ++i) {
        goal_pos[i] = p_t0_[i] + ik_.spec->clutch_scale*(p_c[i] - p_c0_[i]);
      }
      // THE ORIENTATION IS A DELTA, AND IT MUST STAY ONE. Four parts, in the
      // order an editor will meet them:
      //
      // 1. WHAT IS TRUE. q_delta is the controller's rotation SINCE ENGAGE,
      //    and it is applied to q_t0_ — the tool's own orientation at engage.
      //    The absolute orientation of the tool frame never enters.
      // 2. WHY THAT MATTERS HERE AND NOWHERE ELSE. The two shipped robots do
      //    not agree on what the tool frame is: measured at their homes, the
      //    Franka's `hand` frame and the SO101's `gripper` frame are 135.85
      //    deg apart, and the SO101's own authored tool site is a further
      //    90.0000 deg about +y from the body frame used here. Neither
      //    divergence is corrected anywhere in this tree.
      // 3. WHAT BREAKS IT. Any rewrite that maps the controller orientation
      //    ONTO the tool instead of composing a delta with it — "point the
      //    gripper where the hand points", a fixed q_offset per robot, or
      //    initialising q_t0_ from anything but ik_dls_tcp. Each of those
      //    turns those two numbers from irrelevant into a per-robot
      //    correction table that has to be measured on hardware.
      // 4. HOW YOU WOULD FIND OUT. You would not, on the Franka: it is the
      //    robot whose frame the constant would be fitted to. The SO101 would
      //    engage with the jaw rotated ~136 deg and look like a mounting bug.
      mju_negQuat(q_c0_inv, q_c0_);
      mju_mulQuat(q_delta, q_c, q_c0_inv);
      mju_mulQuat(goal_quat, q_delta, q_t0_);
      mju_normalize4(goal_quat);
      mxr_slew_target(goal_pos, goal_quat, kMaxLinRate, kMaxAngRate, dt,
                      target_pos_, target_quat_);
    }
  }

  // DLS toward the (held or moving) target, every frame.
  mjtNum dq[MXR_MAX_ARM];
  ik_dls_solve(&ik_, m, d, target_pos_, target_quat_, dq);
  ik_dls_write_ctrl(&ik_, m, d, dq);

  // Free target-TCP debug logline (robotics targets are observed here).
  if (frame_ % 72 == 0) {
    mjtNum p[3], q[4], dp[3], w[3];
    ik_dls_tcp(&ik_, d, p, q);
    mju_sub3(dp, target_pos_, p);
    mju_subQuat(w, target_quat_, q);
    LOGI("teleop: %s | target-TCP: %.1f mm, %.2f deg",
         engaged_ ? "engaged" : "idle", 1000*mju_norm3(dp),
         mju_norm3(w)*180/kPi);
  }
}

void Teleop::AppendMarker(mjvScene* scn) const {
  if (scn->ngeom >= scn->maxgeom) {
    return;
  }
  const mjtNum size[3] = {0.02, 0.02, 0.02};
  mjtNum mat[9];
  mju_quat2Mat(mat, target_quat_);
  const float engaged_rgba[4] = {0.2f, 1.0f, 0.3f, 0.5f};
  const float idle_rgba[4] = {0.7f, 0.7f, 0.7f, 0.35f};
  mjv_initGeom(scn->geoms + scn->ngeom++, mjGEOM_BOX, size, target_pos_, mat,
               engaged_ ? engaged_rgba : idle_rgba);
}
