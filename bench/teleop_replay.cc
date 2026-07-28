// The teleop domain logic's only execution outside a headset, and the one
// artifact that turns "this refactor changed nothing" into a diff.
//
// Drives SimScene through a fixed scripted controller sequence — engage,
// move, release, re-engage, recenter, tracking loss, A-reset, gripper ramp —
// and records the resulting trajectory. Same record-and-check idiom as
// bench/baseline.cc: no argument prints the block, --ref also compares.
//
// It carries four assertions the headers already claim and nothing has ever
// checked, all of which are pure arithmetic and never needed a headset:
// the frame-convention axis map, zero engage jump, slew compliance, and a
// geometry census. Being a HOST executable linked against mxr_core is also
// what enforces the portable tier — a shell symbol reaching into the core
// fails to link here.
//
// The reference file locks ONE ARCHITECTURE against itself across a
// refactor — it records the arch it was taken on and the bitwise trace
// comparison is skipped, loudly, anywhere else. It is deliberately not a
// cross-architecture claim; baselines/host-x86_64.txt is the file that makes
// that one, and it is checked by `baseline`.

#ifdef __FAST_MATH__
#error "-ffast-math breaks the determinism this binary exists to measure"
#endif

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

#include "frames.h"
#include "mesh_buffers.h"
#include "mxr_error.h"
#include "sim_scene.h"

namespace {

constexpr int kFrames = 360;          // 5 s at 72 Hz
constexpr double kFrameDt = 1.0/72.0;
// Non-zero, because a display clock that starts at exactly 0 is the one
// input that distinguishes "first frame" sentinels from each other.
constexpr double kEpoch = 1.0;
constexpr int kTraceEvery = 12;

// Must match src/teleop.cc. Duplicated rather than exported: these are the
// values the assertions below are asserting AGAINST, so reading them from
// the implementation would make the checks tautological. Note the asymmetry
// this buys: *loosening* the limits in teleop.cc fails here loudly, while
// *tightening* them passes vacuously, because SlewWatch only checks an
// upper bound.
constexpr mjtNum kMaxLinRate = 1.5;
constexpr mjtNum kMaxAngRate = 3.0;

// The trace hash is a bitwise lock, so it is only comparable against a
// reference recorded on the SAME architecture. That is the real predicate;
// `#ifdef __EMSCRIPTEN__` was a proxy for it that reads "not host means
// wasm" and is wrong for the third target — teleop_replay sits in the
// common prefix, so an Android configure builds it too, and running it on
// arm64 with --ref would report "trace differs" when the correct answer is
// "this reference does not apply here". Recorded in the baseline, compared
// explicitly.
#if defined(__EMSCRIPTEN__) || defined(__wasm32__)
constexpr const char* kArch = "wasm32";
#elif defined(__x86_64__)
constexpr const char* kArch = "x86_64";
#elif defined(__aarch64__)
constexpr const char* kArch = "aarch64";
#else
constexpr const char* kArch = "unknown";
#endif

uint64_t FnvUpdate(uint64_t h, const void* p, size_t n) {
  const unsigned char* b = static_cast<const unsigned char*>(p);
  for (size_t i = 0; i < n; ++i) {
    h ^= b[i];
    h *= 1099511628211ull;
  }
  return h;
}

// The scripted sequence. Deliberately hardcoded: a parameterised script is a
// second thing to keep in sync with a recorded reference.
//   f20  engage        f120 release      f140 re-engage
//   f180 recenter      f200-219 tracking lost
//   f240 A-reset       f240-300 gripper ramp   f300 release
//
// Two branches this script does NOT cover, stated so the pass count is not
// read as coverage: `a_down` is true for exactly one frame (k == 240), so
// Teleop::Update's `frame_ > 1` first-frame suppression — the only
// behavioural change the A-edge move introduced — is never exercised; and
// `recenter_edge` is true for one frame (k == 180), so the 4-consecutive-
// frame recenter watchdog never fires either. Both would need a second
// script, which is a second reference to keep in sync.
void ScriptFrame(int k, InputState* in) {
  const double kTwoPi = 6.283185307179586476925286766559;
  const double t = k*kFrameDt;
  in->grip_valid = !(k >= 200 && k < 220);
  in->grip_pos[0] = static_cast<float>(0.30*sin(kTwoPi*0.13*t));
  in->grip_pos[1] = static_cast<float>(1.20 + 0.15*sin(kTwoPi*0.21*t));
  in->grip_pos[2] = static_cast<float>(-0.45 + 0.20*sin(kTwoPi*0.17*t));
  const double ang = 0.6*sin(kTwoPi*0.11*t);
  const double ax[3] = {0.2672612419124244, 0.5345224838248488,
                        0.8017837257372732};
  const double sh = sin(0.5*ang), ch = cos(0.5*ang);
  in->grip_quat[0] = static_cast<float>(ax[0]*sh);
  in->grip_quat[1] = static_cast<float>(ax[1]*sh);
  in->grip_quat[2] = static_cast<float>(ax[2]*sh);
  in->grip_quat[3] = static_cast<float>(ch);
  in->squeeze = ((k >= 20 && k < 120) || (k >= 140 && k < 300)) ? 1.0f : 0.0f;
  in->trigger =
      k < 240 ? 0.0f
              : static_cast<float>((k - 240)/60.0 > 1 ? 1 : (k - 240)/60.0);
  in->a_down = (k == 240);
  in->recenter_edge = (k == 180);
}

int g_failures = 0;

// Diagnostics go to stderr so stdout is exactly the recordable block:
// `teleop_replay scene.xml > baselines/...` captures data and nothing else.
void Check(bool ok, const char* what) {
  fprintf(stderr, "%-34s [%s]\n", what, ok ? "ok" : "FAIL");
  if (!ok) {
    ++g_failures;
  }
}

// frames.h states a testable property in prose and nothing has ever run it:
// "a point 1 m in front of the user at eye height h lands at MuJoCo
// (+1, 0, h)". This is the half of Android's handedness gate that never
// needed a device.
void CheckFrameConvention() {
  const mjtNum* t = MXR_T_MJ_FROM_XR;
  const float h = 1.6f;
  const float fwd[3] = {0, h, -1};  // 1 m in front, at eye height
  mjtNum p[3];
  mxr_pos_mj_from_xr(fwd, p);
  bool ok = fabs(p[0] - (1.0 + t[0])) < 1e-12 &&
            fabs(p[1] - (0.0 + t[1])) < 1e-12 &&
            fabs(p[2] - (h + t[2])) < 1e-12;
  Check(ok, "frames: 1 m fwd -> MJ +x");

  // The three axis maps, translation removed: XR -Z -> +x, +Y -> +z, +X -> -y.
  const float axes[3][3] = {{0, 0, -1}, {0, 1, 0}, {1, 0, 0}};
  const mjtNum want[3][3] = {{1, 0, 0}, {0, 0, 1}, {0, -1, 0}};
  ok = true;
  for (int a = 0; a < 3; ++a) {
    mjtNum got[3];
    mxr_pos_mj_from_xr(axes[a], got);
    for (int i = 0; i < 3; ++i) {
      ok = ok && fabs((got[i] - t[i]) - want[a][i]) < 1e-12;
    }
  }
  Check(ok, "frames: XR -Z/+Y/+X axis map");

  // mxr_mat4_xr_from_mj must invert mxr_pos_mj_from_xr, or the renderer and
  // the teleop math disagree about where the robot is.
  float m[16];
  mxr_mat4_xr_from_mj(m);
  mjtNum p_mj[3];
  mxr_pos_mj_from_xr(fwd, p_mj);
  float back[3];
  for (int r = 0; r < 3; ++r) {
    back[r] = m[12 + r];
    for (int c = 0; c < 3; ++c) {
      back[r] += m[c*4 + r]*static_cast<float>(p_mj[c]);
    }
  }
  ok = fabs(back[0] - fwd[0]) < 1e-5 && fabs(back[1] - fwd[1]) < 1e-5 &&
       fabs(back[2] - fwd[2]) < 1e-5;
  Check(ok, "frames: mat4 inverts the point map");
}

// teleop.h claims engage is "zero-jump by construction" and nothing had ever
// checked it. Measured 2026-07-28, and the claim is exact for only one half:
//
//   position    bitwise identical. goal = p_t0 + s*(p_c - p_c0) with
//               p_c == p_c0 is p_t0 exactly, and the slew's n == 0 path
//               adds a literal zero.
//   orientation NOT bitwise, ~2e-16 rad. The cause is NOT double rounding:
//               InputState::grip_quat is float[4], so q_c carries a ~3e-08
//               norm defect, eight orders above double eps. That defect is
//               pure MAGNITUDE and rotationally inert — mju_negQuat is the
//               conjugate, so q (x) conj(q) is |q|^2 times the identity,
//               the same rotation — which is why the angle below stays at
//               ~1e-16 rather than ~1e-08. See src/teleop.h.
//
// The orientation half is measured as an ANGLE, in radians, via mju_subQuat
// — the same sign-invariant metric SlewWatch uses fifteen lines down.
// Comparing raw components instead would not survive the quaternion double
// cover: q and -q are the same rotation, but component-wise they differ by
// up to 1.41, so any future reformulation returning -q_t0 — physically
// identical and genuinely zero-jump — would turn this gate red. A red gate
// on a correct change is how tolerances get widened.
//
// Position stays bitwise: widening it to match the quaternion bound would
// throw away exactness that genuinely is there.
struct EngageWatch {
  bool prev_engaged = false;
  mjtNum prev_pos[3] = {0}, prev_quat[4] = {0};
  bool have_prev = false;
  // Radians, and anchored at both ends rather than picked for headroom:
  //
  //   measured engage jump          ~4e-16 rad
  //   Franka joint encoder step     ~2e-05 rad   (7 orders above this bound)
  //   one frame of full-rate slew   4.17e-02 rad (3 rad/s at 72 Hz)
  //
  // So 1e-12 rad is unreachable by any jump a human or a bug could produce,
  // and ~3.5 orders above the floating-point floor.
  static constexpr double kQuatTolRad = 1e-12;
  int engages = 0;
  bool pos_ok = true;
  bool quat_ok = true;
  double worst_pos = 0, worst_quat_rad = 0;

  void Observe(bool engaged, const mjtNum* pos, const mjtNum* quat) {
    if (engaged && !prev_engaged && have_prev) {
      ++engages;
      for (int i = 0; i < 3; ++i) {
        pos_ok = pos_ok && pos[i] == prev_pos[i];
        const double e = fabs(pos[i] - prev_pos[i]);
        worst_pos = e > worst_pos ? e : worst_pos;
      }
      mjtNum w[3];
      mju_subQuat(w, quat, prev_quat);
      const double ang = mju_norm3(w);
      quat_ok = quat_ok && ang <= kQuatTolRad;
      worst_quat_rad = ang > worst_quat_rad ? ang : worst_quat_rad;
    }
    prev_engaged = engaged;
    memcpy(prev_pos, pos, sizeof(prev_pos));
    memcpy(prev_quat, quat, sizeof(prev_quat));
    have_prev = true;
  }
};

// mxr_slew_target's header promises |dp| <= max_lin*dt every frame. A-reset
// teleports the target by design, so that one frame is excluded explicitly
// rather than by widening the bound.
struct SlewWatch {
  mjtNum prev_pos[3] = {0}, prev_quat[4] = {0};
  bool have_prev = false;
  double worst_lin = 0, worst_ang = 0;
  bool ok = true;

  void Observe(const mjtNum* pos, const mjtNum* quat, double dt, bool reset) {
    if (have_prev && !reset) {
      mjtNum dp[3];
      mju_sub3(dp, pos, prev_pos);
      const double lin = mju_norm3(dp);
      mjtNum w[3];
      mju_subQuat(w, quat, prev_quat);
      const double ang = mju_norm3(w);
      // 1 ulp of slack at the bound, per the header's "hit exactly" claim.
      const double lin_max = kMaxLinRate*dt*(1 + 1e-15) + 1e-15;
      const double ang_max = kMaxAngRate*dt*(1 + 1e-15) + 1e-15;
      worst_lin = lin/(kMaxLinRate*dt) > worst_lin ? lin/(kMaxLinRate*dt)
                                                   : worst_lin;
      worst_ang = ang/(kMaxAngRate*dt) > worst_ang ? ang/(kMaxAngRate*dt)
                                                   : worst_ang;
      ok = ok && lin <= lin_max && ang <= ang_max;
    }
    memcpy(prev_pos, pos, sizeof(prev_pos));
    memcpy(prev_quat, quat, sizeof(prev_quat));
    have_prev = true;
  }
};

// Census over the DEREFERENCED triangle set, never over base_vertex values —
// otherwise this locks in the very index convention it exists to outlive.
uint64_t TriangleChecksum(const MeshBuffers& mb) {
  uint64_t h = 14695981039346656037ull;
  for (size_t i = 0; i < mb.meshes.size(); ++i) {
    const MeshRange& r = mb.meshes[i];
    for (uint32_t k = 0; k < r.index_count; ++k) {
      const size_t v = r.base_vertex + mb.indices[r.first_index + k];
      h = FnvUpdate(h, mb.verts[v].pos, sizeof(mb.verts[v].pos));
      h = FnvUpdate(h, mb.verts[v].normal, sizeof(mb.verts[v].normal));
    }
  }
  return h;
}

struct Ref {
  unsigned long long trace = 0;
  unsigned long long tris = 0;
  long verts = -1, indices = -1, meshes = -1, frames = -1;
  char arch[32] = "";
  bool found = false;
};

bool ReadRef(const char* path, Ref* ref) {
  FILE* f = fopen(path, "r");
  if (!f) {
    fprintf(stderr, "cannot open ref %s\n", path);
    return false;
  }
  char line[512];
  while (fgets(line, sizeof(line), f)) {
    unsigned long long u;
    long v;
    if (sscanf(line, "trace_fnv1a = 0x%llx", &u) == 1) {
      ref->trace = u;
      ref->found = true;
    } else if (sscanf(line, "census_tri_fnv1a = 0x%llx", &u) == 1) {
      ref->tris = u;
    } else if (sscanf(line, "census_vertices = %ld", &v) == 1) {
      ref->verts = v;
    } else if (sscanf(line, "census_indices = %ld", &v) == 1) {
      ref->indices = v;
    } else if (sscanf(line, "census_meshes = %ld", &v) == 1) {
      ref->meshes = v;
    } else if (sscanf(line, "nframes = %ld", &v) == 1) {
      ref->frames = v;
    } else if (sscanf(line, "arch = %31s", ref->arch) == 1) {
      // recorded architecture; gates the bitwise trace comparison only
    }
  }
  fclose(f);
  return ref->found;
}

}  // namespace

int main(int argc, char** argv) {
  mxr_install_error_hooks();  // before the first MuJoCo call
  const char* ref_path = nullptr;
  const char* scene = nullptr;
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--ref") && i + 1 < argc) {
      ref_path = argv[++i];
    } else if (!scene) {
      scene = argv[i];
    } else {
      scene = nullptr;
      break;
    }
  }
  if (!scene) {
    fprintf(stderr, "usage: %s <scene.xml> [--ref <baseline.txt>]\n", argv[0]);
    return 1;
  }

  char err[1024];
  mjModel* model = mj_loadXML(scene, nullptr, err, sizeof(err));
  if (!model) {
    fprintf(stderr, "load failed: %s\n", err);
    return 1;
  }
  mjData* data = mj_makeData(model);

  SimScene sim;
  sim.Init(model, data, "bench/teleop_replay scripted clock");
  if (!sim.teleop_ready()) {
    fprintf(stderr, "teleop init failed\n");
    return 1;
  }

  MeshBuffers mb;
  BuildMeshBuffers(model, &mb);

  printf("# teleop replay: scripted engage/move/release/recenter/reset\n");
  printf("mujoco_version = %s\n", mj_versionString());
  printf("arch = %s\n", kArch);
  printf("nq = %ld\n", static_cast<long>(model->nq));
  printf("nframes = %d\n", kFrames);
  printf("frame_dt = %.17g\n", kFrameDt);
  printf("census_vertices = %zu\n", mb.verts.size());
  printf("census_indices = %zu\n", mb.indices.size());
  printf("census_meshes = %zu\n", mb.meshes.size());
  printf("census_tri_fnv1a = 0x%016llx\n",
         static_cast<unsigned long long>(TriangleChecksum(mb)));

  EngageWatch engage;
  SlewWatch slew;
  uint64_t hash = 14695981039346656037ull;
  const Teleop& teleop = sim.teleop();
  for (int k = 0; k < kFrames; ++k) {
    InputState in;
    ScriptFrame(k, &in);
    const double dt = k == 0 ? 0 : kFrameDt;
    sim.Advance(in, kEpoch + k*kFrameDt);

    const bool engaged = teleop.engaged();
    engage.Observe(engaged, teleop.target_pos(), teleop.target_quat());
    slew.Observe(teleop.target_pos(), teleop.target_quat(), dt, in.a_down);

    const unsigned char e = engaged ? 1u : 0u;
    hash = FnvUpdate(hash, &e, 1);
    hash = FnvUpdate(hash, teleop.target_pos(), 3*sizeof(mjtNum));
    hash = FnvUpdate(hash, teleop.target_quat(), 4*sizeof(mjtNum));
    hash = FnvUpdate(hash, data->qpos, model->nq*sizeof(mjtNum));

    // Sampled for a human reading a failing diff; the checksum above is what
    // actually covers every frame.
    if (k % kTraceEvery == 0) {
      printf("trace %d %d", k, e ? 1 : 0);
      for (int i = 0; i < 3; ++i) {
        printf(" %.17g", teleop.target_pos()[i]);
      }
      for (int i = 0; i < 4; ++i) {
        printf(" %.17g", teleop.target_quat()[i]);
      }
      for (int i = 0; i < model->nq; ++i) {
        printf(" %.17g", data->qpos[i]);
      }
      printf("\n");
    }
  }
  printf("trace_fnv1a = 0x%016llx\n", static_cast<unsigned long long>(hash));

  CheckFrameConvention();
  Check(engage.engages >= 3, "teleop: script reached >= 3 engages");
  Check(engage.pos_ok, "teleop: engage pos jump bitwise 0");
  Check(engage.quat_ok, "teleop: engage quat jump <= 1e-12 rad");
  fprintf(stderr, "  (%d engages, worst |dpos| %.3g m, worst angle %.3g rad)\n",
          engage.engages, engage.worst_pos, engage.worst_quat_rad);
  Check(slew.ok, "teleop: slew within rate limits");
  fprintf(stderr, "  (worst lin %.6f, ang %.6f of the bound)\n",
          slew.worst_lin, slew.worst_ang);

  if (ref_path) {
    Ref ref;
    if (!ReadRef(ref_path, &ref)) {
      fprintf(stderr, "bad ref block\n");
      return 1;
    }
    Check(ref.frames == kFrames, "ref: frame count matches");
    Check(ref.verts == static_cast<long>(mb.verts.size()) &&
              ref.indices == static_cast<long>(mb.indices.size()) &&
              ref.meshes == static_cast<long>(mb.meshes.size()),
          "ref: geometry census matches");
    Check(ref.tris == TriangleChecksum(mb), "ref: triangle set matches");

    // The census above IS a cross-architecture claim — mesh parsing and
    // qhull have to agree everywhere, and they do. The trace hash is not:
    // it is bitwise, and 360 frames of IK and physics amplify the ~1e-19
    // per-step divergence that `baseline` measures and tolerates into a
    // different hash. Comparing it off-arch would assert something untrue
    // and produce a red gate nobody could act on, so it is scoped rather
    // than weakened. Skipped loudly, never silently. A reference with no
    // `arch =` line predates this check and is treated as not comparable.
    if (ref.arch[0] && !strcmp(ref.arch, kArch)) {
      Check(ref.trace == hash, "ref: trace matches");
      if (ref.trace != hash) {
        fprintf(stderr,
                "trace differs: got 0x%016llx want 0x%016llx — diff this "
                "binary's stdout against %s to find the first frame\n",
                static_cast<unsigned long long>(hash), ref.trace, ref_path);
      }
    } else {
      fprintf(stderr,
              "ref: trace comparison SKIPPED (bitwise lock recorded on "
              "arch '%s', this build is '%s'; its trace is 0x%016llx)\n",
              ref.arch[0] ? ref.arch : "(unrecorded)", kArch,
              static_cast<unsigned long long>(hash));
    }
  }

  fprintf(stderr, "replay_check = %s\n", g_failures == 0 ? "PASS" : "FAIL");
  sim.Destroy();
  mj_deleteData(data);
  mj_deleteModel(model);
  return g_failures == 0 ? 0 : 1;
}
