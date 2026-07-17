// Host prototype: scripted-target drive of the DLS solver against the
// unmodified Menagerie Franka scene. IK runs at display-rate (~72 Hz)
// against the 2 ms physics timestep with rate-limited targets, mirroring
// the shape of the on-device teleop loop.
//
// Usage: ik_prototype <path/to/franka_emika_panda/scene.xml>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <mujoco/mujoco.h>

#include "ik_dls.h"
#include "mxr_error.h"
#include "rate_slew.h"

namespace {

constexpr int kIkPeriodSteps = 7;       // IK every 7 steps = 14 ms at 2 ms dt
constexpr double kMaxLinRate = 1.5;     // m/s target rate limit
constexpr double kMaxAngRate = 3.0;     // rad/s target rate limit
constexpr double kPhaseDuration = 1.5;  // s per waypoint hold
constexpr double kPi = 3.14159265358979323846;

// PASS thresholds for a held target after each phase.
constexpr double kMaxPosErrMm = 5.0;
constexpr double kMaxOriErrDeg = 2.0;
constexpr double kMaxHomeDistRad = 0.15;  // final posture vs home, with bias

struct Waypoint {
  mjtNum dpos[3];    // offset from the initial TCP position, world frame
  mjtNum axis[3];    // world rotation axis
  mjtNum angle_deg;  // rotation pre-multiplying the initial TCP quat
};

constexpr Waypoint kWaypoints[] = {
    {{0.00, 0.00, -0.10}, {0, 0, 1}, 0},
    {{0.10, -0.10, -0.10}, {0, 0, 1}, 30},
    {{-0.05, 0.10, 0.05}, {0, 1, 0}, -25},
    {{0.00, 0.00, 0.00}, {0, 0, 1}, 0},
};
constexpr int kNumPhases = sizeof(kWaypoints)/sizeof(kWaypoints[0]);

struct CaseResult {
  double pos_err_mm[kNumPhases];
  double ori_err_deg[kNumPhases];
  double home_dist_linf;  // final arm posture distance to home, rad
};

void WaypointPose(const Waypoint& w, const mjtNum p0[3], const mjtNum q0[4],
                  mjtNum pos[3], mjtNum quat[4]) {
  for (int i = 0; i < 3; ++i) {
    pos[i] = p0[i] + w.dpos[i];
  }
  mjtNum qrot[4];
  mju_axisAngle2Quat(qrot, w.axis, w.angle_deg*kPi/180);
  mju_mulQuat(quat, qrot, q0);
}

// Rate-limit the moving target toward the waypoint (~1.5 m/s, ~3 rad/s).
void SlewTarget(const mjtNum way_pos[3], const mjtNum way_quat[4], double dt,
                mjtNum tgt_pos[3], mjtNum tgt_quat[4]) {
  mxr_slew_target(way_pos, way_quat, kMaxLinRate, kMaxAngRate, dt, tgt_pos,
                  tgt_quat);
}

CaseResult RunCase(const mjModel* m, mjtNum ns_gain) {
  mjData* d = mj_makeData(m);
  int key = mj_name2id(m, mjOBJ_KEY, "home");
  mj_resetDataKeyframe(m, d, key);
  mj_forward(m, d);  // kinematics for the initial TCP pose

  IkDls ik;
  if (ik_dls_init(&ik, m) != 0) {
    fprintf(stderr, "ik_dls_init failed\n");
    exit(1);
  }
  ik.ns_gain = ns_gain;

  mjtNum p0[3], q0[4], tgt_pos[3], tgt_quat[4];
  ik_dls_tcp(&ik, d, p0, q0);
  mju_copy(tgt_pos, p0, 3);
  mju_copy(tgt_quat, q0, 4);

  const double dt = m->opt.timestep;
  const double dt_ik = kIkPeriodSteps*dt;
  const int steps_per_phase = (int)llround(kPhaseDuration/dt);

  CaseResult res = {};
  for (int phase = 0; phase < kNumPhases; ++phase) {
    mjtNum way_pos[3], way_quat[4];
    WaypointPose(kWaypoints[phase], p0, q0, way_pos, way_quat);
    for (int s = 0; s < steps_per_phase; ++s) {
      if (s % kIkPeriodSteps == 0) {
        SlewTarget(way_pos, way_quat, dt_ik, tgt_pos, tgt_quat);
        mjtNum dq[IK_NARM];
        ik_dls_solve(&ik, m, d, tgt_pos, tgt_quat, dq);
        ik_dls_write_ctrl(&ik, m, d, dq);
      }
      mj_step(m, d);
    }
    mjtNum p[3], q[4], dp[3], w[3];
    ik_dls_tcp(&ik, d, p, q);
    mju_sub3(dp, way_pos, p);
    res.pos_err_mm[phase] = 1000*mju_norm3(dp);
    mju_subQuat(w, way_quat, q);
    res.ori_err_deg[phase] = mju_norm3(w)*180/kPi;
  }

  double linf = 0;
  for (int i = 0; i < IK_NARM; ++i) {
    double e = fabs(d->qpos[ik.qposadr[i]] - ik.qhome[i]);
    linf = e > linf ? e : linf;
  }
  res.home_dist_linf = linf;
  mj_deleteData(d);
  return res;
}

}  // namespace

int main(int argc, char** argv) {
  mxr_install_error_hooks();  // before the first MuJoCo call
  if (argc != 2) {
    fprintf(stderr, "usage: %s <scene.xml>\n", argv[0]);
    return 1;
  }
  char err[1024];
  mjModel* m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) {
    fprintf(stderr, "load failed: %s\n", err);
    return 1;
  }
  printf("mujoco_version = %s\n", mj_versionString());

  CaseResult with_ns = RunCase(m, 0.1);
  CaseResult no_ns = RunCase(m, 0.0);
  mj_deleteModel(m);

  bool pass = true;
  for (int i = 0; i < kNumPhases; ++i) {
    bool ok = with_ns.pos_err_mm[i] <= kMaxPosErrMm &&
              with_ns.ori_err_deg[i] <= kMaxOriErrDeg;
    pass = pass && ok;
    printf("phase %d: pos_err = %.3f mm, ori_err = %.3f deg  [%s]\n", i,
           with_ns.pos_err_mm[i], with_ns.ori_err_deg[i], ok ? "ok" : "FAIL");
  }
  bool home_ok = with_ns.home_dist_linf <= kMaxHomeDistRad;
  pass = pass && home_ok;
  printf("home_dist_linf(ns_gain=0.1) = %.4f rad  [%s]\n",
         with_ns.home_dist_linf, home_ok ? "ok" : "FAIL");
  printf("home_dist_linf(ns_gain=0)   = %.4f rad  [info: nullspace contrast]\n",
         no_ns.home_dist_linf);
  printf("ik_prototype: %s\n", pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}
