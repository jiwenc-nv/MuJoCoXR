// Everything expressed in an XR reference space is declared here and
// converted here, exactly once. Two runtimes feed this header — OpenXR
// (LOCAL_FLOOR, with STAGE/LOCAL fallbacks) and WebXR ('local-floor') — so
// the names say `xr`, never `stage`: OpenXR's STAGE is a distinct reachable
// value of the same variable, and calling the whole class of spaces "stage"
// is false whenever the fallback fires. Rules:
//
//   - Quaternions cross as xyzw (both XR runtimes); MuJoCo is [w,x,y,z].
//     Reorder on EVERY crossing.
//   - R_mj_from_xr = Rz(-90deg) * Rx(+90deg). Axis map: XR -Z -> MJ +x,
//     XR +Y -> MJ +z, XR +X -> MJ -y. Testable definition: a point 1 m in
//     front of the user at eye height h lands at MuJoCo (+1, 0, h).
//     bench/teleop_replay checks exactly that.
//   - Crossing rules: p_mj = R * p_xr + t;  q_mj = q_mj_from_xr (x) q_xr.
//
// Identifier direction reads by adjacency: p_mj = mxr_pos_mj_from_xr(p_xr).
// Comments use that same `_from_` form and never the A_T_B robotics form, so
// the tree carries one direction vocabulary rather than two.

#ifndef MUJOCOXR_SRC_FRAMES_H_
#define MUJOCOXR_SRC_FRAMES_H_

#include <mujoco/mujoco.h>

// A handedness CONVENTION. It cannot be wrong at runtime: it is fixed by the
// two specs (both XR runtimes are y-up / -z-forward, MuJoCo is REP-103 z-up)
// and by ar_scene.xml's table. If the axes gizmo is wrong, this is the bug.
static const mjtNum MXR_Q_MJ_FROM_XR[4] = {0.5, 0.5, -0.5, -0.5};  // wxyz

// A WORKSPACE CALIBRATION, and routinely wrong: user standoff plus table
// height above the floor datum. Robot base ~1 m in front of the user; MJ z=0
// (robot base = table top, assets/ar_scene.xml) sits 0.73 m above the
// physical floor so the virtual table stands on it. Both shells log the
// value at init; see the three-cause checklist in docs/validation-*.md
// before editing it.
static const mjtNum MXR_T_MJ_FROM_XR[3] = {-1.0, 0.0, -0.73};

// The one XR-typed struct in the tree, so teleop.cc's "convert before you
// form a delta" warning is structurally enforceable: an InputState field is
// the only float that has not yet crossed. Plain float arrays, not the
// runtimes' pose types — XrPosef is {orientation, position} and WebXR hands
// over {position, orientation}, so anything with a memory layout invites the
// memcpy that compiles and is silently wrong. Shells fill this field by
// field.
struct InputState {
  bool grip_valid = false;
  float grip_pos[3] = {0, 0, 0};       // XR reference space, metres
  float grip_quat[4] = {0, 0, 0, 1};   // xyzw, the order both runtimes use
  float trigger = 0;                   // [0,1]
  float squeeze = 0;                   // [0,1]
  // RAW LEVEL, not an edge: the core owns edge detection so one definition
  // serves both shells. OpenXR's `currentState && changedSinceLastSync` at
  // one sync per frame already *is* a level diff, and WebXR's Gamepad has no
  // `changed` flag at all, so a shell-side edge would impose the weaker API
  // on both.
  bool a_down = false;
  // A genuine asynchronous event with no level to sample, so this one stays
  // a shell-latched edge: set true for exactly one frame, never cleared by
  // the core. Teleop warns if it stays asserted, which is what a shell
  // reporting a level here would look like.
  bool recenter_edge = false;
};

// XR quaternion (xyzw) -> MuJoCo world quaternion (wxyz).
static inline void mxr_quat_mj_from_xr(const float q_xr[4], mjtNum q_mj[4]) {
  const mjtNum q[4] = {q_xr[3], q_xr[0], q_xr[1], q_xr[2]};  // reorder
  mju_mulQuat(q_mj, MXR_Q_MJ_FROM_XR, q);
}

// XR reference-space point -> MuJoCo world point: R * p + t.
static inline void mxr_pos_mj_from_xr(const float p_xr[3], mjtNum p_mj[3]) {
  const mjtNum p[3] = {p_xr[0], p_xr[1], p_xr[2]};
  mju_rotVecQuat(p_mj, p, MXR_Q_MJ_FROM_XR);
  p_mj[0] += MXR_T_MJ_FROM_XR[0];
  p_mj[1] += MXR_T_MJ_FROM_XR[1];
  p_mj[2] += MXR_T_MJ_FROM_XR[2];
}

// Column-major float mat4 of xr_from_mj (the inverse of the above), for
// folding MuJoCo-world geometry into the XR reference space in a renderer:
// p_xr = R^T * (p_mj - t).
static inline void mxr_mat4_xr_from_mj(float out[16]) {
  mjtNum r[9];
  mju_quat2Mat(r, MXR_Q_MJ_FROM_XR);  // row-major R
  // Rotation part: R^T, column-major out[c*4+row] = R^T[row][c] = R[c][row]
  for (int row = 0; row < 3; ++row) {
    for (int c = 0; c < 3; ++c) {
      out[c*4 + row] = static_cast<float>(r[c*3 + row]);
    }
    out[row*4 + 3] = 0.0f;
  }
  // Translation: -R^T * t
  for (int row = 0; row < 3; ++row) {
    mjtNum v = 0;
    for (int k = 0; k < 3; ++k) {
      v += r[k*3 + row]*MXR_T_MJ_FROM_XR[k];  // R^T[row][k] = R[k][row]
    }
    out[12 + row] = static_cast<float>(-v);
  }
  out[15] = 1.0f;
}

#endif  // MUJOCOXR_SRC_FRAMES_H_
