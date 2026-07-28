// The frame loop, which is the product, extracted out of android_main so a
// second shell cannot re-derive it differently.
//
// It is TWO calls and that is load-bearing, not aesthetic: today
// mjv_updateScene runs only when the runtime says the frame will be
// presented AND the view poses located. A single fused Step() would compose
// ~70 geoms on non-render frames, which is a CPU cost change on a target
// with no hardware to measure it on. Advance is unconditional; Compose is
// only for frames that will actually draw.
//
// It is not an interface and takes no callback: the step loop is a bare
// `for (s) mj_step(...)` and the shell does nothing between any two steps,
// so there is nothing to call back into. Both shells are a two-call
// sequence, and everything platform-shaped stays below that line.
//
// Time crosses as an ABSOLUTE display timestamp in seconds and the core
// derives dt. That is the whole point: each shell converts its runtime's
// units exactly once, right next to the value that produced them (Android
// nanoseconds, WebXR milliseconds), so the unit hazard cannot be got wrong
// twice. dt, a clamp and an accumulator would have been three chances.

#ifndef MUJOCOXR_SRC_SIM_SCENE_H_
#define MUJOCOXR_SRC_SIM_SCENE_H_

#include <mujoco/mujoco.h>

#include "frames.h"
#include "teleop.h"

class SimScene {
 public:
  // Borrows m and d; the shell keeps ownership and outlives this object.
  // `clock_source` names the shell's latched timestamp source and appears in
  // the stalled-clock diagnostic — one source per session, never mixed.
  // Returns nothing: mjv_makeScene is `void` and there is no failure to
  // report. Teleop::Init genuinely can fail, and teleop_ready() is that
  // signal.
  //
  // Destroy() -> Init() IS A SUPPORTED CYCLE and app/android/main.cc drives
  // it once per B-press to switch robots. What is refused is Init over a LIVE
  // scene — that would leak the mjvScene — so the rule is "one Destroy per
  // Init", not "one Init per process". Init value-initialises the whole
  // object, so every member with a default initialiser goes back to it and
  // the three mjv structs, which have none, go to zero. No state survives
  // the cycle, and the reset cannot drift from the member list.
  void Init(mjModel* m, mjData* d, const char* clock_source);

  // Re-names the latched clock, and logs the new name in the same `clock =`
  // format Init uses. For a shell that cannot know its clock until a session
  // exists: WebXR's predictedDisplayTime is a property of XRFrame, and a UA
  // that does not expose it forces the rAF fallback. The stalled-clock
  // diagnostic below is only useful if it names the source actually being
  // read. The string must outlive this object.
  void set_clock_source(const char* clock_source);

  // Once per frame, on every frame, before anything is drawn.
  void Advance(const InputState& in, double t_display_s);

  // Only on frames that will actually draw. Returns null before Init or
  // after Destroy. Decor is appended after the scene so it blends over the
  // robot rather than under it.
  const mjvScene* Compose();

  // Drop the clutch and re-latch the clock, on every session-end transition
  // of BOTH shells: app/web/main.cc's mxr_end_session from the XRSession
  // 'end' event, and app/android/main.cc on the session_running() true ->
  // false edge. Without it a session that ends mid-clutch resumes with
  // engaged_ still true over a stale p_c0_, and with an accumulator holding
  // the entire time spent outside the session.
  void EndSession();

  void Destroy();

  bool teleop_ready() const { return teleop_ready_; }
  const Teleop& teleop() const { return teleop_; }

 private:
  mjModel* model_ = nullptr;
  mjData* data_ = nullptr;
  Teleop teleop_;
  mjvScene scene_;
  mjvOption vis_opt_;
  mjvCamera cam_;
  const char* clock_source_ = "";
  double sim_accum_ = 0;
  double last_display_s_ = 0;
  bool clock_started_ = false;
  bool scene_valid_ = false;
  bool teleop_ready_ = false;
  int64_t zero_dt_run_ = 0;
};

#endif  // MUJOCOXR_SRC_SIM_SCENE_H_
