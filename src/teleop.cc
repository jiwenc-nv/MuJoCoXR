#include "teleop.h"

#include <cmath>

#include "frames.h"
#include "mxr_log.h"
#include "rate_slew.h"

namespace {

constexpr mjtNum kMaxLinRate = 1.5;   // m/s target rate limit
constexpr mjtNum kMaxAngRate = 3.0;   // rad/s target rate limit
constexpr float kEngageThreshold = 0.8f;   // squeeze hysteresis
constexpr float kReleaseThreshold = 0.6f;
constexpr double kPi = 3.14159265358979323846;

}  // namespace

bool Teleop::Init(const mjModel* m, const mjData* d) {
  if (ik_dls_init(&ik_, m) != 0) {
    LOGE("ik_dls_init failed");
    return false;
  }
  gripper_act_ = mj_name2id(m, mjOBJ_ACTUATOR, "actuator8");
  if (gripper_act_ < 0 && m->nu > IK_NARM) {
    gripper_act_ = IK_NARM;  // 8th actuator by position
  }
  // The 255 below is a per-model remap (panda.xml:275 says so in its own
  // comment). Assert it rather than deriving it: actuator_ctrlrange supplies
  // the scale but not the polarity, and Menagerie's Robotiq 2F-85 is also
  // 0..255 with 0 = OPEN. Without this the failure is silent — MuJoCo clamps
  // and the gripper simply never closes.
  if (gripper_act_ >= 0) {
    const mjtNum* r = m->actuator_ctrlrange + 2*gripper_act_;
    if (r[0] != 0.0 || r[1] != 255.0) {
      LOGW("gripper ctrlrange is (%g, %g), not (0, 255): the 255*(1-trigger) "
           "mapping below is wrong for this model", r[0], r[1]);
    }
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
  // A is reported as a raw level; the edge is derived here so both shells
  // share one definition. Suppressed on the first frame so a button already
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

  // Gripper is direct: inverted range, 255 = open per the home keyframe.
  // bench/excitation.h:27 carries a line that looks identical to the one
  // below and must NOT be unified with it — `t` here is float and `close`
  // there is double, so the two round differently, and the recorded
  // baseline was taken through that one. The warning lives there.
  if (gripper_act_ >= 0) {
    float t = input.trigger < 0 ? 0 : (input.trigger > 1 ? 1 : input.trigger);
    d->ctrl[gripper_act_] = 255.0*(1.0 - t);
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
        goal_pos[i] = p_t0_[i] + scale_*(p_c[i] - p_c0_[i]);
      }
      mju_negQuat(q_c0_inv, q_c0_);
      mju_mulQuat(q_delta, q_c, q_c0_inv);
      mju_mulQuat(goal_quat, q_delta, q_t0_);
      mju_normalize4(goal_quat);
      mxr_slew_target(goal_pos, goal_quat, kMaxLinRate, kMaxAngRate, dt,
                      target_pos_, target_quat_);
    }
  }

  // DLS toward the (held or moving) target, every frame.
  mjtNum dq[IK_NARM];
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
