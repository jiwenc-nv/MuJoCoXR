// Rate-limited target conditioning shared by the host IK prototype and the
// on-device teleop (~1.5 m/s, ~3 rad/s).

#ifndef MUJOCOXR_SRC_RATE_SLEW_H_
#define MUJOCOXR_SRC_RATE_SLEW_H_

#include <mujoco/mujoco.h>

// Moves (pos, quat) toward (goal_pos, goal_quat) by at most max_lin*dt /
// max_ang*dt.
//
// A reachable goal is hit to a few ulp, NOT exactly. Position is exact —
// pos + 1.0*(goal - pos) is goal in IEEE-754. Orientation is not: measured
// 2026-07-28 over the bench/teleop_replay trajectory, mju_subQuat followed
// by mju_quatIntegrate at an unclamped step reproduces the goal quaternion
// bitwise in 2400 of 2800 cases and misses by ~1.4e-17 per component in the
// other 400. That is a precision, not a lag — but it is not exactness, and
// SlewWatch only checks the UPPER bound, so nothing would catch the claim
// being wrong. State the measured behaviour rather than the tidy one.
//
// dt must be finite, because a NaN dt makes `n > max_step && n > 0` false
// and takes the s = 1.0 branch — the rate limit switched off rather than
// relaxed. There are two callers and each guarantees it differently:
// src/sim_scene.cc derives dt from a shell-supplied display clock and
// clamps a non-finite one to 0 before Teleop::Update sees it, and
// host/ik_prototype.cc:96 passes a fixed m->opt.timestep that never reaches
// SimScene at all. A third caller owes its own guarantee; neither of these
// covers it.
static inline void mxr_slew_target(const mjtNum goal_pos[3],
                                   const mjtNum goal_quat[4], mjtNum max_lin,
                                   mjtNum max_ang, mjtNum dt, mjtNum pos[3],
                                   mjtNum quat[4]) {
  mjtNum dp[3];
  mju_sub3(dp, goal_pos, pos);
  mjtNum n = mju_norm3(dp);
  mjtNum max_step = max_lin*dt;
  mjtNum s = (n > max_step && n > 0) ? max_step/n : 1.0;
  pos[0] += s*dp[0];
  pos[1] += s*dp[1];
  pos[2] += s*dp[2];

  mjtNum w[3];
  mju_subQuat(w, goal_quat, quat);
  n = mju_norm3(w);
  mjtNum max_ang_step = max_ang*dt;
  if (n > max_ang_step && n > 0) {
    w[0] *= max_ang_step/n;
    w[1] *= max_ang_step/n;
    w[2] *= max_ang_step/n;
  }
  mju_quatIntegrate(quat, w, 1);
}

#endif  // MUJOCOXR_SRC_RATE_SLEW_H_
