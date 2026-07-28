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
  // DO NOT UNIFY THIS LINE WITH src/teleop.cc's GRIPPER MAPPING, and the
  // temptation is now STRONGER rather than weaker, because that mapping has
  // been parameterised: teleop.cc no longer spells `255.0*(1.0 - x)` at all.
  // It reads `closed + (open - closed)*(1.0 - t)` from the robot table, which
  // reduces to this expression bit-for-bit ONLY for the Franka's endpoints
  // (0, 255) — which is exactly what makes "these are the same, share them"
  // look true.
  //
  // Two reasons they are not. The OPERAND differs: teleop.cc's `t` is float
  // and rounded to float before use, while `close` here is double, so the two
  // round differently even where the algebra agrees. And the ENDPOINTS differ
  // per robot, while this file is Franka-only by construction (baseline.cc
  // refuses nu < 8). baselines/host-x86_64.txt was recorded through THIS
  // expression, so a well-meaning "share the gripper mapping" refactor
  // silently invalidates the reference — the gate stays green against a
  // reference that no longer describes the code. Re-recording it is a
  // separate, explicitly argued commit (docs/validation-web.md gate 1 has the
  // procedure).
  ctrl[7] = 255.0*(1.0 - close);                    // gripper: 255 = open
}

#endif  // MUJOCOXR_BENCH_EXCITATION_H_
