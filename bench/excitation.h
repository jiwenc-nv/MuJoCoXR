// Deterministic scripted ctrl excitation for the short-horizon invariant
// check: sweep joints 1/4/6
// and close the gripper from the `home` keyframe, so a dynamics-broken build
// cannot pass by merely holding pose. Amplitudes sit inside the explicit
// actuator ctrlranges (actuator4 -3.0718..-0.0698, actuator6 -0.0175..3.7525).
// ctrl is evaluated at each step's start time and held across the step; the
// same source compiles on host and device so the trajectory is identical.

#ifndef MUJOCOXR_BENCH_EXCITATION_H_
#define MUJOCOXR_BENCH_EXCITATION_H_

#include <math.h>

static const double kExcitationDuration = 2.0;  // s, short horizon by design

static inline void excitation_ctrl(double t, const double* home_ctrl, int nu,
                                   double* ctrl) {
  const double kPi = 3.14159265358979323846;
  for (int i = 0; i < nu; ++i) {
    ctrl[i] = home_ctrl[i];
  }
  // non-commensurate frequencies: no sweep completes a full period by t = 2 s,
  // so the final state sits generically away from home
  ctrl[0] = home_ctrl[0] + 0.40*sin(2*kPi*0.45*t);  // joint1
  ctrl[3] = home_ctrl[3] + 0.30*sin(2*kPi*0.80*t);  // joint4
  ctrl[5] = home_ctrl[5] + 0.35*sin(2*kPi*1.15*t);  // joint6
  double close = t/1.5 > 1 ? 1 : t/1.5;
  ctrl[7] = 255.0*(1.0 - close);                    // gripper: 255 = open
}

#endif  // MUJOCOXR_BENCH_EXCITATION_H_
