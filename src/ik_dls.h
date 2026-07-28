// Damped-least-squares IK for a position-servo arm, plus the model ids that
// solving one requires resolving:
//   dq = J' (J J' + lambda^2 I6)^-1 e
// 6x6 solve via mju_cholFactor/mju_cholSolve; rotation error from local-frame
// mju_subQuat rotated into world; nullspace home-posture bias projected
// through the same damped pseudoinverse. No Eigen, no allocation.
//
// It is deliberately NOT only the mathematics. The solver is written against
// one arm at a time, and which arm that is comes from src/robot_spec.c — so
// this file owns the RESOLUTION step as well: mj_name2id over that robot's
// table row, once, into the id arrays below. Everything downstream (the task
// loop in src/teleop.cc, both shells) reads resolved ids and never a name.
// The arm is 5 dofs on one shipped robot and 7 on the other; the task is 6D
// on both, so J is 6xnarm and may be rank-deficient in either direction.

#ifndef MUJOCOXR_SRC_IK_DLS_H_
#define MUJOCOXR_SRC_IK_DLS_H_

#include <mujoco/mujoco.h>

#include "robot_spec.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  // The table row this was resolved from — the single source for every tuned
  // number (lambda, w_rot, ns_gain, clutch_scale, the jaw endpoints). Held
  // rather than copied so there is exactly one copy of each constant.
  //
  // ONLY mxr_robot_probe MAY SET THIS, and what it returns always points into
  // the `static const` tables in src/robot_spec.c, which outlive everything.
  // Do not repoint it at a local to override a tuning value: nothing here
  // enforces the lifetime, an IkDls is copyable and small enough to look
  // harmless as a member, and the resulting dangle would surface as one arm
  // driving with plausible-but-wrong gains rather than as a crash. If a binary
  // needs to sweep a constant, sweep the table row.
  const MxrRobot* spec;
  int narm;                    // arm joints actually resolved; <= MXR_MAX_ARM
  int tcp_body;                // body id of spec->tcp_body
  int dofadr[MXR_MAX_ARM];     // dof index of each arm joint
  int qposadr[MXR_MAX_ARM];    // qpos index of each arm joint
  int act[MXR_MAX_ARM];        // position servo driving each arm joint
  int gripper_act;             // actuator id of spec->gripper_act
  // Per-joint ctrl bounds, precomputed as actuator_ctrlrange INTERSECT
  // jnt_range. Both exist and they are NOT the same constraint.
  //
  // On the SO101, 2 of the 5 ARM joints have an intersection that differs from
  // ctrlrange: shoulder_lift by 1e-6 rad (immaterial) and wrist_roll by 0.0973
  // rad, whose ctrlrange is that much WIDER than the joint can travel. (The
  // model has 4 such joints of 6, but the other two are the gripper, and
  // ik_dls_write_ctrl deliberately does not clamp it.) So wrist_roll is the
  // whole of this, and it is worth the precompute for what MEASURING the old
  // path showed:
  //
  //   clamp to ctrlrange     qpos 2.745940  |qfrc_constraint| 2.937977 N.m
  //                          qfrc_actuator  2.940000          nlim 1
  //   clamp to intersection  qpos 2.743845  |qfrc_constraint| 0.000000 N.m
  //                          qfrc_actuator  0.002011          nlim 0
  //
  // 2.940000 is EXACTLY actuator_forcerange for every SO101 joint. Commanding
  // to ctrlrange alone did not merely stall the joint short of its target: it
  // parked wrist_roll at 100 % of rated torque against a live
  // mjCNSTR_LIMIT_JOINT constraint and held it there indefinitely, because
  // nothing in the loop ever backs off. In sim that is a wasted 2.9 N.m; on
  // the real servo it is a stalled motor at rated current until thermal
  // shutdown, and it is the strongest reason this intersection exists.
  //
  // The Panda's two ranges are bitwise equal on all 7 joints, which is why
  // taking the intersection is a structural identity there rather than a
  // change that happens not to fire — and therefore why its golden trace
  // cannot detect a bug in this line. The SO101's can.
  mjtNum ctrl_lo[MXR_MAX_ARM];
  mjtNum ctrl_hi[MXR_MAX_ARM];
  mjtNum qhome[MXR_MAX_ARM];   // home posture, from the `home` keyframe
} IkDls;

// Probe src/robot_spec.c for the robot this model IS, then resolve its names.
// Returns 0 on success; on failure returns -1 and points *why at a static
// string naming what could not be resolved. `why` must NOT be NULL (unlike
// mxr_robot_probe's, which may be) — see below for why every caller wants it.
//
// *why matters more than it looks: the one caller (Teleop::Init) turns a -1
// into teleop_ready_ == false, and a not-ready SimScene still renders. The
// failure therefore looks like a robot that draws perfectly and never moves,
// which on a headset is indistinguishable from a dead controller. This string
// is what tells the two apart, so callers must log it.
int ik_dls_init(IkDls* ik, const mjModel* m, const char** why);

// World pose of the TCP from current mjData kinematics. `quat` is the
// tcp_body frame, which is NOT a common convention across robots — see
// MxrRobot::tcp_body and the orientation block in src/teleop.cc.
void ik_dls_tcp(const IkDls* ik, const mjData* d, mjtNum pos[3],
                mjtNum quat[4]);

// One DLS step toward (target_pos, target_quat); writes ik->narm joint deltas.
void ik_dls_solve(const IkDls* ik, const mjModel* m, const mjData* d,
                  const mjtNum target_pos[3], const mjtNum target_quat[4],
                  mjtNum dq[]);

// Write ctrl[arm] = clamp(qpos + dq, ctrl_lo, ctrl_hi). Gripper untouched.
void ik_dls_write_ctrl(const IkDls* ik, const mjModel* m, mjData* d,
                       const mjtNum dq[]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCOXR_SRC_IK_DLS_H_
