// Single owner of the OpenXR(stage) <-> MuJoCo(world) convention:
//
//   - Quaternions: XrQuaternionf is {x,y,z,w}; MuJoCo is [w,x,y,z].
//     Reorder on EVERY crossing.
//   - world_T_stage = Rz(-90deg) * Rx(+90deg), the one owned constant:
//     q_ws = (0.5, 0.5, -0.5, -0.5) wxyz. Axis map: XR -Z -> MJ +x,
//     XR +Y -> MJ +z, XR +X -> MJ -y. Testable definition: a point 1 m in
//     front of the user at eye height h lands at MuJoCo (+1, 0, h).
//   - Crossing rules: p_mj = R_ws * p_xr + t_ws;  q_mj = q_ws (x) q_xr.
//   - t_ws places the robot base ~1 m in front of the user (robot is fixed
//     at the MJ origin; the stage is what moves). The on-device
//     axes-gizmo/per-axis handedness gate validates all of this before any
//     teleop math is trusted.

#ifndef MUJOCOXR_APP_FRAMES_H_
#define MUJOCOXR_APP_FRAMES_H_

#include <openxr/openxr.h>

#include <mujoco/mujoco.h>

static const mjtNum MXR_Q_WS[4] = {0.5, 0.5, -0.5, -0.5};  // wxyz
// Robot base ~1 m in front of the user; MJ z=0 (robot base = table top,
// assets/ar_scene.xml) sits 0.73 m above the physical floor so the virtual
// table stands on it.
static const mjtNum MXR_T_WS[3] = {-1.0, 0.0, -0.73};

// XR quaternion (xyzw) -> MuJoCo world quaternion (wxyz): q_ws (x) q_xr.
static inline void mxr_quat_xr_to_mj(const XrQuaternionf* q_xr,
                                     mjtNum q_mj[4]) {
  const mjtNum q[4] = {q_xr->w, q_xr->x, q_xr->y, q_xr->z};  // reorder
  mju_mulQuat(q_mj, MXR_Q_WS, q);
}

// XR stage point -> MuJoCo world point: R_ws * p + t_ws.
static inline void mxr_pos_xr_to_mj(const XrVector3f* p_xr, mjtNum p_mj[3]) {
  const mjtNum p[3] = {p_xr->x, p_xr->y, p_xr->z};
  mju_rotVecQuat(p_mj, p, MXR_Q_WS);
  p_mj[0] += MXR_T_WS[0];
  p_mj[1] += MXR_T_WS[1];
  p_mj[2] += MXR_T_WS[2];
}

// Column-major float mat4 of stage_T_world (inverse of world_T_stage), for
// folding MuJoCo-world geometry into XR stage space in the renderer:
// p_xr = R_ws^T * (p_mj - t_ws).
static inline void mxr_stage_from_world(float out[16]) {
  mjtNum r_ws[9];
  mju_quat2Mat(r_ws, MXR_Q_WS);  // row-major R_ws
  // Rotation part: R_ws^T, column-major out[c*4+r] = R_ws^T[r][c] = R_ws[c][r]
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[c*4 + r] = static_cast<float>(r_ws[c*3 + r]);
    }
    out[r*4 + 3] = 0.0f;
  }
  // Translation: -R_ws^T * t_ws
  for (int r = 0; r < 3; ++r) {
    mjtNum v = 0;
    for (int k = 0; k < 3; ++k) {
      v += r_ws[k*3 + r]*MXR_T_WS[k];  // R_ws^T[r][k] = R_ws[k][r]
    }
    out[12 + r] = static_cast<float>(-v);
  }
  out[15] = 1.0f;
}

#endif  // MUJOCOXR_APP_FRAMES_H_
