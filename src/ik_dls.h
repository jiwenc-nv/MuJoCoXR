// Damped-least-squares IK for the Franka Panda arm (7 hinge dofs, 6D task):
//   dq = J' (J J' + lambda^2 I6)^-1 e
// 6x6 solve via mju_cholFactor/mju_cholSolve; rotation error from local-frame
// mju_subQuat rotated into world; nullspace home-posture bias projected
// through the same damped pseudoinverse. No Eigen, no allocation.

#ifndef MUJOCOXR_SRC_IK_DLS_H_
#define MUJOCOXR_SRC_IK_DLS_H_

#include <mujoco/mujoco.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { IK_NARM = 7 };

typedef struct {
  int hand_body;           // body id of "hand"
  int dofadr[IK_NARM];     // dof index of each arm joint
  int qposadr[IK_NARM];    // qpos index of each arm joint
  int act[IK_NARM];        // position-servo actuator driving each arm joint
  mjtNum tcp_offset[3];    // TCP in the hand frame: grasp midpoint (0,0,0.103)
  mjtNum lambda;           // DLS damping (0.05-0.1)
  mjtNum ns_gain;          // nullspace home bias gain; 0 disables (one edit)
  mjtNum qhome[IK_NARM];   // home posture for the bias
} IkDls;

// Resolve ids from the unmodified Menagerie panda model; set default gains.
// Returns 0 on success, -1 if any name/actuator is missing.
int ik_dls_init(IkDls* ik, const mjModel* m);

// World pose of the TCP from current mjData kinematics.
void ik_dls_tcp(const IkDls* ik, const mjData* d, mjtNum pos[3],
                mjtNum quat[4]);

// One DLS step toward (target_pos, target_quat); writes joint deltas dq[7].
void ik_dls_solve(const IkDls* ik, const mjModel* m, const mjData* d,
                  const mjtNum target_pos[3], const mjtNum target_quat[4],
                  mjtNum dq[IK_NARM]);

// Write ctrl[arm] = clamp(qpos + dq, actuator ctrlrange). Gripper untouched.
void ik_dls_write_ctrl(const IkDls* ik, const mjModel* m, mjData* d,
                       const mjtNum dq[IK_NARM]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCOXR_SRC_IK_DLS_H_
