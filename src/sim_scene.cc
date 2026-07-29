#include "sim_scene.h"

#include "mxr_log.h"

namespace {

// World-axes gizmo decor. This lives in the core, not in a shell, because it
// IS the handedness gate: shells with divergent axis->colour mappings
// would make the one cross-target comparison the gate exists for
// meaningless. +x red, +y green, +z blue (REP-103).
void AppendAxesGizmo(mjvScene* scn) {
  const mjtNum sizes[3][3] = {
      {0.4, 0.012, 0.012}, {0.012, 0.4, 0.012}, {0.012, 0.012, 0.4}};
  const mjtNum poses[3][3] = {{0.4, 0, 0.02}, {0, 0.4, 0.02}, {0, 0, 0.42}};
  const float rgba[3][4] = {
      {1, 0.2f, 0.2f, 1}, {0.2f, 1, 0.2f, 1}, {0.25f, 0.45f, 1, 1}};
  for (int i = 0; i < 3 && scn->ngeom < scn->maxgeom; ++i) {
    mjv_initGeom(scn->geoms + scn->ngeom++, mjGEOM_BOX, sizes[i], poses[i],
                 nullptr, rgba[i]);
  }
}

}  // namespace

void SimScene::Init(mjModel* m, mjData* d, const char* clock_source) {
  if (scene_valid_) {
    // Init over a LIVE scene would leak the mjvScene. Refuse, loudly. This is
    // not a refusal to re-initialise — Destroy() -> Init() is a supported
    // cycle and the Android B-press swap drives it — only a refusal to skip
    // the Destroy.
    LOGE("SimScene::Init called twice without Destroy; ignoring the second "
         "call");
    return;
  }
  // Value-initialisation, in one statement that cannot drift from the member
  // list — the same treatment Teleop::Init gives itself, and for the same
  // reason now that both are re-init paths. `SimScene{}` on a non-aggregate
  // zero-initialises and then runs the implicit constructor, so members with
  // a default initialiser get it and scene_/vis_opt_/cam_, which have none,
  // get zero rather than being left alone.
  // Reached only with scene_valid_ == false, i.e. before the first Init or
  // after a Destroy, so there is nothing live to clobber.
  //
  // last_display_s_ is why this is not cosmetic: Destroy() does not clear it,
  // and it was previously harmless ONLY because app/android/main.cc happens
  // to call EndSession() before Destroy(). That made a correctness property
  // of this class depend on the call order of one caller. It no longer does.
  *this = SimScene{};

  model_ = m;
  data_ = d;
  clock_source_ = clock_source;

  int home = mj_name2id(m, mjOBJ_KEY, "home");
  if (home >= 0) {
    mj_resetDataKeyframe(m, d, home);
  }
  mj_forward(m, d);

  mjv_defaultScene(&scene_);
  mjv_defaultOption(&vis_opt_);  // groups 0-2 visible: collision hidden
  mjv_defaultFreeCamera(m, &cam_);
  mjv_makeScene(m, &scene_, 1000);
  scene_valid_ = true;
  teleop_ready_ = teleop_.Init(m, d);

  // Logged identically on every target so the cause checklist in
  // docs/validation-*.md can be followed from either log. A non-floor
  // reference space is ~1.6 m, a bad UA floor estimate ~0.2 m, and a wrong
  // calibration is whatever this line says.
  LOGI("frames: t_mj_from_xr = (%.3f %.3f %.3f) m, clock = %s",
       MXR_T_MJ_FROM_XR[0], MXR_T_MJ_FROM_XR[1], MXR_T_MJ_FROM_XR[2],
       clock_source_);
}

void SimScene::set_clock_source(const char* clock_source) {
  clock_source_ = clock_source;
  LOGI("frames: clock = %s (latched at session entry)", clock_source_);
}

void SimScene::Advance(const InputState& in, double t_display_s) {
  // Frame dt from absolute display times; the first frame steps nothing.
  double dt_frame = 0;
  if (clock_started_) {
    dt_frame = t_display_s - last_display_s_;
    // Written so a NON-FINITE dt lands on 0 and is then named by the
    // watchdog below. The obvious spelling — `dt < 0 ? 0 : (dt > 0.1 ? 0.1
    // : dt)` — passes NaN through untouched, because EVERY NaN comparison
    // is false: this clamp, the `dt_frame == 0` watchdog, and
    // rate_slew.h's `n > max_step && n > 0` all take their else branch. The
    // result is a target that teleports to the goal with the 1.5 m/s and
    // 3 rad/s limits silently bypassed, physics frozen, rendering perfect
    // and nothing in the log. A shell can hand us that for free: WebXR's
    // XRFrame.predictedDisplayTime is optional in practice (WebKit does not
    // expose it), and `undefined * 1e-3` is NaN, which crosses the C ABI as
    // a double without complaint. Guard it HERE so every shell inherits it.
    dt_frame = (dt_frame > 0) ? (dt_frame > 0.1 ? 0.1 : dt_frame) : 0;
  }
  last_display_s_ = t_display_s;
  clock_started_ = true;

  // A stalled clock renders perfectly and steps nothing, with nothing in the
  // log to say so. Name the latched source instead.
  if (dt_frame == 0) {
    if (++zero_dt_run_ == 11) {
      LOGE("dt has been 0 for 11 frames running — clock source '%s' is not "
           "advancing", clock_source_);
    }
  } else {
    zero_dt_run_ = 0;
  }

  if (!teleop_ready_) {
    return;
  }
  teleop_.Update(model_, data_, in, dt_frame);

  // Accumulator-owed fixed steps; catch-up capped at ~2 frames of work.
  sim_accum_ += dt_frame;
  const double timestep = model_->opt.timestep;
  int owed = static_cast<int>(sim_accum_/timestep);
  // A LATENCY BOUND, in absolute seconds: 2*0.014/timestep + 1 = 15 steps at
  // a 2 ms timestep. 0.014 s was one frame at 72 Hz, so at a 90 Hz refresh
  // this silently becomes 2.7 frames rather than 2. Kept verbatim for parity
  // across every target; the domain-correct form is 2*dt_measured.
  const int cap = static_cast<int>(2.0*0.014/timestep) + 1;
  if (owed > cap) {
    // Overflow drops the residual as well as the steps: we are conceding the
    // deficit, not deferring it, so the next frame starts clean instead of
    // immediately owing the same backlog again. Sim time stops tracking
    // display time here, by design.
    LOGW("sim deficit: dropping %d steps", owed - cap);
    owed = cap;
    sim_accum_ = 0;
  } else {
    // The normal branch keeps the sub-timestep residual, so sim time stays
    // locked to display time instead of drifting by up to one timestep per
    // frame.
    sim_accum_ -= owed*timestep;
  }
  for (int s = 0; s < owed; ++s) {
    mj_step(model_, data_);
  }
}

const mjvScene* SimScene::Compose() {
  if (!scene_valid_) {
    return nullptr;
  }
  // Abstract scene extraction; decor appended after so it draws last.
  mjv_updateScene(model_, data_, &vis_opt_, nullptr, &cam_, mjCAT_ALL,
                  &scene_);
  AppendAxesGizmo(&scene_);
  if (teleop_ready_) {
    teleop_.AppendMarker(&scene_);
  }
  return &scene_;
}

void SimScene::EndSession() {
  if (teleop_ready_) {
    teleop_.Disengage();
  }
  sim_accum_ = 0;
  clock_started_ = false;
  zero_dt_run_ = 0;
}

void SimScene::Destroy() {
  if (scene_valid_) {
    mjv_freeScene(&scene_);
    scene_valid_ = false;
  }
  teleop_ready_ = false;
  model_ = nullptr;
  data_ = nullptr;
}
