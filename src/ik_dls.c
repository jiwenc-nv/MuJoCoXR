#include "ik_dls.h"

#include <stdio.h>
#include <string.h>

// Jacobian buffers sized generously above the Franka scene's nv = 9.
#define IK_MAXNV 32

int ik_dls_init(IkDls* ik, const mjModel* m) {
  memset(ik, 0, sizeof(*ik));
  if (m->nv > IK_MAXNV) {
    return -1;
  }
  ik->hand_body = mj_name2id(m, mjOBJ_BODY, "hand");
  int key = mj_name2id(m, mjOBJ_KEY, "home");
  if (ik->hand_body < 0 || key < 0) {
    return -1;
  }
  for (int i = 0; i < IK_NARM; ++i) {
    char name[16];
    snprintf(name, sizeof(name), "joint%d", i + 1);
    int jid = mj_name2id(m, mjOBJ_JOINT, name);
    if (jid < 0) {
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
      return -1;
    }
  }
  // grasp midpoint in the hand frame (doc: TCP anchor, no XML surgery)
  ik->tcp_offset[0] = 0;
  ik->tcp_offset[1] = 0;
  ik->tcp_offset[2] = 0.103;
  ik->lambda = 0.05;
  ik->ns_gain = 0.1;
  return 0;
}

void ik_dls_tcp(const IkDls* ik, const mjData* d, mjtNum pos[3],
                mjtNum quat[4]) {
  const mjtNum* p = d->xpos + 3*ik->hand_body;
  const mjtNum* q = d->xquat + 4*ik->hand_body;
  mjtNum off[3];
  mju_rotVecQuat(off, ik->tcp_offset, q);
  pos[0] = p[0] + off[0];
  pos[1] = p[1] + off[1];
  pos[2] = p[2] + off[2];
  mju_copy(quat, q, 4);
}

void ik_dls_solve(const IkDls* ik, const mjModel* m, const mjData* d,
                  const mjtNum target_pos[3], const mjtNum target_quat[4],
                  mjtNum dq[IK_NARM]) {
  int nv = m->nv;
  mjtNum jacp[3*IK_MAXNV], jacr[3*IK_MAXNV];
  mjtNum p_tcp[3], q_tcp[4];
  ik_dls_tcp(ik, d, p_tcp, q_tcp);
  mj_jac(m, d, jacp, jacr, p_tcp, ik->hand_body);

  // 6D task error: position, then local-frame subQuat rotated into world
  mjtNum e[6], e_local[3];
  mju_sub3(e, target_pos, p_tcp);
  mju_subQuat(e_local, target_quat, q_tcp);  // argument order matters
  mju_rotVecQuat(e + 3, e_local, q_tcp);

  // J: 6 x 7 arm columns of [jacp; jacr]
  mjtNum J[6*IK_NARM];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < IK_NARM; ++c) {
      J[r*IK_NARM + c] = jacp[r*nv + ik->dofadr[c]];
      J[(r + 3)*IK_NARM + c] = jacr[r*nv + ik->dofadr[c]];
    }
  }

  // A = J J' + lambda^2 I6, Cholesky-factored in place
  mjtNum A[36];
  mju_mulMatMatT(A, J, J, 6, IK_NARM, 6);
  for (int i = 0; i < 6; ++i) {
    A[i*6 + i] += ik->lambda*ik->lambda;
  }
  mju_cholFactor(A, 6, 0);

  // task step: dq = J' A^-1 e
  mjtNum y[6];
  mju_cholSolve(y, A, e, 6);
  mju_mulMatTVec(dq, J, y, 6, IK_NARM);

  // nullspace home bias: dq += (I - J^+ J) z, z = k (qhome - q), J^+ = J' A^-1
  if (ik->ns_gain != 0) {
    mjtNum z[IK_NARM], Jz[6], w[6], corr[IK_NARM];
    for (int i = 0; i < IK_NARM; ++i) {
      z[i] = ik->ns_gain*(ik->qhome[i] - d->qpos[ik->qposadr[i]]);
    }
    mju_mulMatVec(Jz, J, z, 6, IK_NARM);
    mju_cholSolve(w, A, Jz, 6);
    mju_mulMatTVec(corr, J, w, 6, IK_NARM);
    for (int i = 0; i < IK_NARM; ++i) {
      dq[i] += z[i] - corr[i];
    }
  }
}

void ik_dls_write_ctrl(const IkDls* ik, const mjModel* m, mjData* d,
                       const mjtNum dq[IK_NARM]) {
  for (int i = 0; i < IK_NARM; ++i) {
    int a = ik->act[i];
    // gravity feedforward: the position servo settles at ctrl - qfrc_bias/kp,
    // so add the sag back to make the held pose track the IK solution
    mjtNum kp = m->actuator_gainprm[mjNGAIN*a];
    mjtNum sag = kp > 0 ? d->qfrc_bias[ik->dofadr[i]]/kp : 0;
    mjtNum c = d->qpos[ik->qposadr[i]] + dq[i] + sag;
    if (m->actuator_ctrllimited[a]) {
      const mjtNum* r = m->actuator_ctrlrange + 2*a;
      c = c < r[0] ? r[0] : (c > r[1] ? r[1] : c);
    }
    d->ctrl[a] = c;
  }
}
