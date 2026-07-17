// Clutched DLS teleop of the Franka arm:
// squeeze-hold latches offsets — p_target = p_t0 + s*(p_c - p_c0),
// q_target = (q_c (x) q_c0^-1) (x) q_t0 — with controller poses already
// mapped into MuJoCo world, so engage is zero-jump by construction. Target
// rate-limited; DLS + nullspace home bias every frame; gripper
// ctrl[7] = 255*(1 - trigger) (inverted: 255 = open); A = home reset;
// B stays unbound. Auto-disengage on recenter (reviewer advisory) and on
// lost grip tracking.

#ifndef MUJOCOXR_APP_TELEOP_H_
#define MUJOCOXR_APP_TELEOP_H_

#include <mujoco/mujoco.h>

#include "ik_dls.h"
#include "xr_shell.h"

class Teleop {
 public:
  bool Init(const mjModel* m, const mjData* d);  // target starts at the TCP
  // Once per frame, after xrSyncActions and before the physics steps.
  void Update(const mjModel* m, mjData* d, const XrInputState& input,
              double dt);
  void AppendMarker(mjvScene* scn) const;  // translucent target-pose box
  void Reset(const mjModel* m, mjData* d);

 private:
  IkDls ik_;
  int gripper_act_ = -1;
  bool engaged_ = false;
  int64_t frame_ = 0;
  mjtNum p_c0_[3] = {0}, q_c0_[4] = {1, 0, 0, 0};  // controller at engage
  mjtNum p_t0_[3] = {0}, q_t0_[4] = {1, 0, 0, 0};  // target at engage
  mjtNum target_pos_[3] = {0}, target_quat_[4] = {1, 0, 0, 0};
  mjtNum scale_ = 1.0;  // clutch motion scale s
};

#endif  // MUJOCOXR_APP_TELEOP_H_
