// Short-horizon (<=2 s) cross-platform invariant baseline: deterministic
// ctrl excitation from `home`, then final qpos + energy. The same binary
// records the reference (host) and checks against it (device) — final qpos
// L-inf plus energy, with a small tolerance for cross-platform libm
// differences.
//
// Usage:
//   baseline <scene.xml>               record: print baseline block to stdout
//   baseline <scene.xml> --ref <file>  also compare against a recorded block

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <mujoco/mujoco.h>

#include "excitation.h"
#include "mxr_error.h"

namespace {

constexpr int kMaxNq = 64;
constexpr double kQposTol = 1e-3;    // L-inf gate, rad/m
constexpr double kEnergyTol = 1e-3;  // absolute floor; also relative factor
constexpr double kMinMotion = 0.1;   // rad: excitation must actually move it

struct Baseline {
  int nq = 0;
  double qpos[kMaxNq] = {};
  double energy[2] = {};
  double moved = -1;
};

bool ReadRef(const char* path, Baseline* ref) {
  FILE* f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "cannot open ref %s\n", path);
    return false;
  }
  char line[256];
  while (fgets(line, sizeof(line), f)) {
    int i;
    double v;
    if (sscanf(line, "qpos[%d] = %lf", &i, &v) == 2 && i >= 0 && i < kMaxNq) {
      ref->qpos[i] = v;
      ref->nq = i + 1 > ref->nq ? i + 1 : ref->nq;
    } else if (sscanf(line, "energy_potential = %lf", &v) == 1) {
      ref->energy[0] = v;
    } else if (sscanf(line, "energy_kinetic = %lf", &v) == 1) {
      ref->energy[1] = v;
    } else if (sscanf(line, "moved_linf_vs_home = %lf", &v) == 1) {
      ref->moved = v;
    }
  }
  fclose(f);
  return ref->nq > 0;
}

}  // namespace

int main(int argc, char** argv) {
  mxr_install_error_hooks();  // before the first MuJoCo call
  const char* ref_path = nullptr;
  if (argc == 4 && !strcmp(argv[2], "--ref")) {
    ref_path = argv[3];
  } else if (argc != 2) {
    fprintf(stderr, "usage: %s <scene.xml> [--ref <baseline.txt>]\n", argv[0]);
    return 1;
  }

  char err[1024];
  mjModel* m = mj_loadXML(argv[1], nullptr, err, sizeof(err));
  if (!m) {
    fprintf(stderr, "load failed: %s\n", err);
    return 1;
  }
  if (m->nq > kMaxNq || m->nu < 8) {
    fprintf(stderr, "unexpected model size (nq=%ld nu=%ld)\n",
            static_cast<long>(m->nq), static_cast<long>(m->nu));
    return 1;
  }
  m->opt.enableflags |= mjENBL_ENERGY;

  mjData* d = mj_makeData(m);
  int key = mj_name2id(m, mjOBJ_KEY, "home");
  if (key < 0) {
    fprintf(stderr, "no `home` keyframe\n");
    return 1;
  }
  mj_resetDataKeyframe(m, d, key);
  const mjtNum* home_ctrl = m->key_ctrl + key*m->nu;
  const mjtNum* home_qpos = m->key_qpos + key*m->nq;

  const int nsteps = (int)llround(kExcitationDuration/m->opt.timestep);
  for (int s = 0; s < nsteps; ++s) {
    excitation_ctrl(s*m->opt.timestep, home_ctrl, m->nu, d->ctrl);
    mj_step(m, d);
  }
  mj_forward(m, d);  // refresh energy/kinematics at the final state

  double moved = 0;
  for (int i = 0; i < m->nq; ++i) {
    double e = fabs(d->qpos[i] - home_qpos[i]);
    moved = e > moved ? e : moved;
  }

  printf("mujoco_version = %s\n", mj_versionString());
  printf("nq = %ld\nnv = %ld\nnu = %ld\n", static_cast<long>(m->nq),
         static_cast<long>(m->nv), static_cast<long>(m->nu));
  printf("timestep = %.17g\n", m->opt.timestep);
  printf("integrator = %d\n", m->opt.integrator);
  printf("nsteps = %d\n", nsteps);
  for (int i = 0; i < m->nq; ++i) {
    printf("qpos[%d] = %.17g\n", i, d->qpos[i]);
  }
  printf("energy_potential = %.17g\n", d->energy[0]);
  printf("energy_kinetic = %.17g\n", d->energy[1]);
  printf("moved_linf_vs_home = %.17g\n", moved);

  int rc = 0;
  if (ref_path) {
    Baseline ref;
    if (!ReadRef(ref_path, &ref) || ref.nq != m->nq) {
      fprintf(stderr, "bad ref block (nq=%d vs model %ld)\n", ref.nq,
              static_cast<long>(m->nq));
      return 1;
    }
    double linf = 0;
    for (int i = 0; i < m->nq; ++i) {
      double e = fabs(d->qpos[i] - ref.qpos[i]);
      linf = e > linf ? e : linf;
    }
    double de[2];
    bool e_ok = true;
    for (int i = 0; i < 2; ++i) {
      de[i] = fabs(d->energy[i] - ref.energy[i]);
      double tol = kEnergyTol > kEnergyTol*fabs(ref.energy[i])
                       ? kEnergyTol
                       : kEnergyTol*fabs(ref.energy[i]);
      e_ok = e_ok && de[i] <= tol;
    }
    bool q_ok = linf <= kQposTol;
    bool m_ok = moved >= kMinMotion;  // dynamics actually excited
    printf("qpos_linf_diff = %.17g  [%s, tol %g]\n", linf,
           q_ok ? "ok" : "FAIL", kQposTol);
    printf("energy_diff = %.17g %.17g  [%s]\n", de[0], de[1],
           e_ok ? "ok" : "FAIL");
    printf("motion_check = %.17g  [%s, min %g]\n", moved,
           m_ok ? "ok" : "FAIL", kMinMotion);
    bool pass = q_ok && e_ok && m_ok;
    printf("invariant_check = %s\n", pass ? "PASS" : "FAIL");
    rc = pass ? 0 : 1;
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return rc;
}
