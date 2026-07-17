// Rate-limited target conditioning shared by the host IK prototype and the
// on-device teleop (~1.5 m/s, ~3 rad/s).

#ifndef MUJOCOXR_SRC_RATE_SLEW_H_
#define MUJOCOXR_SRC_RATE_SLEW_H_

#include <mujoco/mujoco.h>

// Moves (pos, quat) toward (goal_pos, goal_quat) by at most max_lin*dt /
// max_ang*dt. mju_subQuat/mju_quatIntegrate are exact inverses when the step
// is unclamped, so a reachable goal is hit exactly.
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
