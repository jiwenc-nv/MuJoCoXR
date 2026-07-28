// Clutched DLS teleop of whichever arm the scene contains:
// squeeze-hold latches offsets — p_target = p_t0 + s*(p_c - p_c0),
// q_target = (q_c (x) q_c0^-1) (x) q_t0 — with controller poses already
// mapped into MuJoCo world, so engage is zero-jump by construction. Exact
// for position (measured bitwise, bench/teleop_replay); for orientation,
// 2.2e-16 rad over the replay script and 5.7e-16 rad over a 2800-pose
// sweep, measured 2026-07-28 and asserted at each half's own bound rather
// than rounding the claim off in either direction.
//
// Where that residual comes from, because the obvious answer is wrong:
// it is NOT double rounding. InputState::grip_quat is float[4], so q_c
// arrives with a norm defect e of order 3e-08 — eight orders above double
// eps — and q_c (x) negQuat(q_c0) inherits (1+e)^2, i.e. about 2e. The
// RATIO is the checkable claim; the digit is not, because e is whatever
// float rounding happens to give the sample at hand — three 14400-pair
// sweeps over differently spaced orientations measured 6.0e-08, 6.9e-08 and
// 9.2e-08. A later re-measurement anywhere in 6-9e-08 is the same fact, not
// drift; a value outside it means the input defect changed.
// That defect is entirely a MAGNITUDE error and rotationally inert:
// mju_negQuat is the CONJUGATE, so for a quaternion of norm 1+e the product
// is (1+e)^2 times the identity, which is the same rotation. Hence the
// ANGLE stays ~1e-16 whether or not mju_normalize4 runs (measured: 5.09e-16
// rad without, 5.66e-16 rad with).
// Keep the mju_normalize4 at teleop.cc:122 anyway — mju_subQuat is
// specified on unit quaternions — but price it honestly: it sits inside
// `if (engaged_)`, so it is one sqrt on EVERY ENGAGED FRAME at 72-90 Hz,
// not one per engage. And do not believe it is holding back four orders of
// magnitude, or chase the 3e-08 by widening grip_quat to double: it is not
// in the angle.
//
// Target rate-limited; DLS + nullspace home bias every frame; the jaw is a
// direct affine map from the trigger onto the robot's tabulated open/closed
// endpoints; A = home reset. B never reaches the core — both shells bind it
// themselves (web ends the session, Android cycles the scene), because
// switching robots destroys and rebuilds this object and so cannot be a
// method on it. Auto-disengage on recenter (reviewer advisory) and on lost
// grip tracking.
//
// EVERY per-robot number is read through ik_.spec, which points at the
// src/robot_spec.c row the model was probed into. Nothing in this class is
// tuned, and nothing here is allowed to test which robot it is driving.

#ifndef MUJOCOXR_SRC_TELEOP_H_
#define MUJOCOXR_SRC_TELEOP_H_

#include <mujoco/mujoco.h>

#include "frames.h"
#include "ik_dls.h"

class Teleop {
 public:
  bool Init(const mjModel* m, const mjData* d);  // target starts at the TCP
  // Once per frame, after the shell has sampled input and before the physics
  // steps. Owns the A-button edge (see InputState::a_down).
  void Update(const mjModel* m, mjData* d, const InputState& input, double dt);
  void AppendMarker(mjvScene* scn) const;  // translucent target-pose box
  void Reset(const mjModel* m, mjData* d);
  // Drop the clutch, hold the target. Unlike Reset this leaves the scene
  // alone: a session ending is not a request to move the robot home.
  void Disengage() { engaged_ = false; }

  bool engaged() const { return engaged_; }
  const mjtNum* target_pos() const { return target_pos_; }
  const mjtNum* target_quat() const { return target_quat_; }

 private:
  IkDls ik_;
  bool engaged_ = false;
  bool a_down_prev_ = false;   // core-owned A edge
  int64_t recenter_run_ = 0;   // consecutive frames recenter_edge was true
  int64_t frame_ = 0;
  mjtNum p_c0_[3] = {0}, q_c0_[4] = {1, 0, 0, 0};  // controller at engage
  mjtNum p_t0_[3] = {0}, q_t0_[4] = {1, 0, 0, 0};  // target at engage
  mjtNum target_pos_[3] = {0}, target_quat_[4] = {1, 0, 0, 0};
  // NO MEMBER OF THIS CLASS CARRIES TUNING, and none should. Every tuned
  // number is a column of MxrRobot, read through ik_.spec. In particular the
  // clutch motion scale is MxrRobot::clutch_scale and must not come back here
  // behind a setter: it is a property of the arm's reach, not of a session,
  // so nothing should be able to change it at runtime.
};

#endif  // MUJOCOXR_SRC_TELEOP_H_
