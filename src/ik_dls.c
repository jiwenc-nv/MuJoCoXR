#include "ik_dls.h"

#include <stdio.h>
#include <string.h>

// Jacobian buffers sized generously above both scenes' nv (Franka 9, SO101 6).
#define IK_MAXNV 32

int ik_dls_init(IkDls* ik, const mjModel* m, const char** why) {
  memset(ik, 0, sizeof(*ik));
  if (m->nv > IK_MAXNV) {
    *why = "model nv exceeds IK_MAXNV";
    return -1;
  }
  // WHICH robot is a property of the model, never an argument: see
  // mxr_robot_probe. A caller cannot assert a robot this model is not.
  const MxrRobot* spec = mxr_robot_probe(m, why);
  if (!spec) {
    return -1;  // *why already set, and names the discriminator
  }
  ik->spec = spec;
  ik->narm = mxr_robot_narm(spec);

  // Everything below reports the FIRST name that failed to resolve INSIDE the
  // matched row. That is the whole value of probing before resolving: without
  // it, a typo in one joint name is indistinguishable from "wrong robot", and
  // the caller is left bisecting five names by hand.
  static char msg[192];
  ik->tcp_body = mj_name2id(m, mjOBJ_BODY, spec->tcp_body);
  if (ik->tcp_body < 0) {
    snprintf(msg, sizeof(msg), "body '%s' vanished between probe and resolve",
             spec->tcp_body);
    *why = msg;
    return -1;
  }
  int key = mj_name2id(m, mjOBJ_KEY, "home");
  if (key < 0) {
    // Not a table name: every scene wrapper must author this keyframe, and
    // both the A-reset and qhome read it.
    snprintf(msg, sizeof(msg),
             "robot '%s' resolved, but the scene has no keyframe named 'home'",
             spec->tcp_body);
    *why = msg;
    return -1;
  }
  for (int i = 0; i < ik->narm; ++i) {
    int jid = mj_name2id(m, mjOBJ_JOINT, spec->joint[i]);
    if (jid < 0) {
      snprintf(msg, sizeof(msg), "robot '%s': joint '%s' not in this model",
               spec->tcp_body, spec->joint[i]);
      *why = msg;
      return -1;
    }
    ik->dofadr[i] = m->jnt_dofadr[jid];
    ik->qposadr[i] = m->jnt_qposadr[jid];
    ik->qhome[i] = m->key_qpos[key*m->nq + ik->qposadr[i]];
    ik->act[i] = -1;
    for (int a = 0; a < m->nu; ++a) {
      if (m->actuator_trntype[a] == mjTRN_JOINT &&
          m->actuator_trnid[2*a] == jid) {
        ik->act[i] = a;
        break;
      }
    }
    if (ik->act[i] < 0) {
      snprintf(msg, sizeof(msg),
               "robot '%s': joint '%s' has no position-servo actuator",
               spec->tcp_body, spec->joint[i]);
      *why = msg;
      return -1;
    }
    // ctrlrange INTERSECT jnt_range, once, here — see IkDls::ctrl_lo. An
    // unlimited side contributes mjMAXVAL, so an unconstrained joint clamps
    // against a bound no reachable pose can hit.
    ik->ctrl_lo[i] = -mjMAXVAL;
    ik->ctrl_hi[i] = mjMAXVAL;
    if (m->actuator_ctrllimited[ik->act[i]]) {
      const mjtNum* r = m->actuator_ctrlrange + 2*ik->act[i];
      ik->ctrl_lo[i] = r[0];
      ik->ctrl_hi[i] = r[1];
    }
    if (m->jnt_limited[jid]) {
      const mjtNum* r = m->jnt_range + 2*jid;
      ik->ctrl_lo[i] = r[0] > ik->ctrl_lo[i] ? r[0] : ik->ctrl_lo[i];
      ik->ctrl_hi[i] = r[1] < ik->ctrl_hi[i] ? r[1] : ik->ctrl_hi[i];
    }
  }
  ik->gripper_act = mj_name2id(m, mjOBJ_ACTUATOR, spec->gripper_act);
  if (ik->gripper_act < 0) {
    snprintf(msg, sizeof(msg),
             "robot '%s': gripper actuator '%s' not in this model",
             spec->tcp_body, spec->gripper_act);
    *why = msg;
    return -1;
  }
  return 0;
}

void ik_dls_tcp(const IkDls* ik, const mjData* d, mjtNum pos[3],
                mjtNum quat[4]) {
  const mjtNum* p = d->xpos + 3*ik->tcp_body;
  const mjtNum* q = d->xquat + 4*ik->tcp_body;
  mjtNum off[3];
  mju_rotVecQuat(off, ik->spec->tcp_offset, q);
  pos[0] = p[0] + off[0];
  pos[1] = p[1] + off[1];
  pos[2] = p[2] + off[2];
  mju_copy(quat, q, 4);
}

void ik_dls_solve(const IkDls* ik, const mjModel* m, const mjData* d,
                  const mjtNum target_pos[3], const mjtNum target_quat[4],
                  mjtNum dq[]) {
  int nv = m->nv;
  const int narm = ik->narm;
  const mjtNum w_rot = ik->spec->w_rot;
  mjtNum jacp[3*IK_MAXNV], jacr[3*IK_MAXNV];
  mjtNum p_tcp[3], q_tcp[4];
  ik_dls_tcp(ik, d, p_tcp, q_tcp);
  mj_jac(m, d, jacp, jacr, p_tcp, ik->tcp_body);

  // 6D task error: position, then local-frame subQuat rotated into world
  mjtNum e[6], e_local[3];
  mju_sub3(e, target_pos, p_tcp);
  mju_subQuat(e_local, target_quat, q_tcp);  // argument order matters
  mju_rotVecQuat(e + 3, e_local, q_tcp);

  // ROTATION WEIGHT. Scaling the bottom three rows of BOTH e and J is what
  // makes the least-squares problem minimise |e_pos|^2 + w^2|e_rot|^2 — the
  // weight has to appear in the same places a change of task units would, or
  // it is not a metric change but an arbitrary bias. w_rot = 1.0 is exactly
  // the identity (x*1.0 is bitwise x), which is why the Franka's golden trace
  // is unmoved by this block existing.
  e[3] *= w_rot;
  e[4] *= w_rot;
  e[5] *= w_rot;

  // J: 6 x narm arm columns of [jacp; w_rot*jacr]
  mjtNum J[6*MXR_MAX_ARM];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < narm; ++c) {
      J[r*narm + c] = jacp[r*nv + ik->dofadr[c]];
      J[(r + 3)*narm + c] = w_rot*jacr[r*nv + ik->dofadr[c]];
    }
  }

  // A = J J' + lambda^2 I6, Cholesky-factored in place
  const mjtNum lambda = ik->spec->lambda;
  mjtNum A[36];
  mju_mulMatMatT(A, J, J, 6, narm, 6);
  for (int i = 0; i < 6; ++i) {
    A[i*6 + i] += lambda*lambda;
  }
  mju_cholFactor(A, 6, 0);

  // task step: dq = J' A^-1 e
  mjtNum y[6];
  mju_cholSolve(y, A, e, 6);
  mju_mulMatTVec(dq, J, y, 6, narm);

  // Nullspace home bias: dq += (I - J^+ J) z, z = k (qhome - q), J^+ = J' A^-1.
  //
  // ONLY MEANINGFUL WHEN THE ARM HAS A NULLSPACE, i.e. narm > 6. What this
  // computes is the DAMPED projector, which is not a projector: it leaks as
  // lambda^2/sigma^2, and on a column-rank-full arm (the SO101, narm = 5)
  // there is no nullspace at all, so every bit of `z` that survives lands on
  // the task command as uncommanded tool motion. That is why the SO101's row
  // sets ns_gain = 0 and why the comment there is longer than this one.
  const mjtNum ns_gain = ik->spec->ns_gain;
  if (ns_gain != 0) {
    mjtNum z[MXR_MAX_ARM], Jz[6], w[6], corr[MXR_MAX_ARM];
    for (int i = 0; i < narm; ++i) {
      z[i] = ns_gain*(ik->qhome[i] - d->qpos[ik->qposadr[i]]);
    }
    mju_mulMatVec(Jz, J, z, 6, narm);
    mju_cholSolve(w, A, Jz, 6);
    mju_mulMatTVec(corr, J, w, 6, narm);
    for (int i = 0; i < narm; ++i) {
      dq[i] += z[i] - corr[i];
    }
  }
}

void ik_dls_write_ctrl(const IkDls* ik, const mjModel* m, mjData* d,
                       const mjtNum dq[]) {
  for (int i = 0; i < ik->narm; ++i) {
    int a = ik->act[i];
    // gravity feedforward: the position servo settles at ctrl - qfrc_bias/kp,
    // so add the sag back to make the held pose track the IK solution
    mjtNum kp = m->actuator_gainprm[mjNGAIN*a];
    mjtNum sag = kp > 0 ? d->qfrc_bias[ik->dofadr[i]]/kp : 0;
    mjtNum c = d->qpos[ik->qposadr[i]] + dq[i] + sag;
    c = c < ik->ctrl_lo[i] ? ik->ctrl_lo[i]
                           : (c > ik->ctrl_hi[i] ? ik->ctrl_hi[i] : c);
    d->ctrl[a] = c;
  }
}
