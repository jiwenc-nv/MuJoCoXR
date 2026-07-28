// The WebXR half of the app, and nothing else: it implements app/web/abi.h
// over the same SimScene the Android shell drives. There is no main loop
// here — the browser owns the loop, shell.js calls in once per
// XRSession.requestAnimationFrame, and everything about what the robot does
// lives in src/sim_scene.cc.
//
// One module instance per page, so the module IS the handle: no MxrApp*, no
// create/destroy pair, no config struct. That is the same reason
// android_main can keep its objects on the stack.

#include "abi.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include <GLES3/gl3.h>
#include <emscripten/html5.h>

#include <mujoco/mujoco.h>

#include "frames.h"
#include "mesh_buffers.h"
#include "mxr_error.h"
#include "mxr_log.h"
#include "robot_spec.h"
#include "scene_renderer.h"
#include "sim_scene.h"

namespace {

// Inline XR gives exactly one view and immersive stereo gives two. Nothing
// in WebXR produces more today — quad-view is XR_VARJO_quad_views, an
// OpenXR extension with no WebXR counterpart — so 4 is slack, not a
// supported configuration. It stays 4 because the number is QUERIED rather
// than agreed: shrinking it to 2 is a one-character edit with zero API
// consequence, and 576 bytes of BSS is not worth the argument. What the
// slack buys is that a runtime reporting three degrades to a clamped draw
// instead of a heap overwrite.
constexpr int kMaxViews = 4;
constexpr int kInputFloats = 9;   // pos[3], quat[4] xyzw, trigger, squeeze
constexpr int kViewFloats = 32;   // proj[16] then view[16]
constexpr int kViewportInts = 4;  // x, y, w, h

float g_input[kInputFloats] = {0};
float g_views[kMaxViews*kViewFloats] = {0};
int32_t g_viewports[kMaxViews*kViewportInts] = {0};
char g_error[1024] = {0};
char g_clock_source[128] = {0};

EMSCRIPTEN_WEBGL_CONTEXT_HANDLE g_ctx = 0;
mjModel* g_model = nullptr;
mjData* g_data = nullptr;
SceneRenderer g_renderer;
SimScene g_sim;
bool g_ready = false;
bool g_warned_nviews = false;
int g_nviews = 0;
// Composed once per frame in mxr_begin_frame and drawn N times. Composing
// per view would re-run mjv_updateScene over ~70 geoms for each eye.
const mjvScene* g_scene = nullptr;

void Fail(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(g_error, sizeof(g_error), fmt, ap);
  va_end(ap);
  LOGE("%s", g_error);
}

}  // namespace

MXR_ABI_EXPORT int mxr_init(void) {
  mxr_install_error_hooks();  // before the first MuJoCo call
  LOGI("MuJoCoXR web starting (MuJoCo %s)", mj_versionString());

  // emscripten_webgl_init_context_attributes memsets to zero and then sets
  // alpha, depth, antialias, premultipliedAlpha, majorVersion and
  // enableExtensionsByDefault to 1 (emsdk system/lib/gl/webgl1.c). Restating
  // any of those at its default value — or assigning EM_FALSE to one of the
  // memset-zero fields — compiles to nothing and reads like a decision, so
  // only three assignments below actually change the request. alpha and
  // depth are the two exceptions, restated because on an AR target they are
  // load-bearing requirements rather than incidental defaults, and a future
  // emsdk changing either would break passthrough silently. XR compatibility
  // is NOT here on purpose: it comes from gl.makeXRCompatible() in shell.js,
  // which must run before any GL resource exists.
  EmscriptenWebGLContextAttributes attr;
  emscripten_webgl_init_context_attributes(&attr);
  attr.majorVersion = 2;  // differs: the default is 1, the renderer is GLES3
  // Passthrough composites this canvas over the camera feed, so the coverage
  // the transparent clear writes has to reach the compositor rather than be
  // flattened to opaque.
  attr.alpha = EM_TRUE;
  attr.depth = EM_TRUE;  // ~70 opaque geoms, drawn unsorted
  // MSAA off: the immersive framebuffer is XRWebGLLayer's, not this context's
  // drawing buffer, so this only pays for the inline/debug canvas.
  attr.antialias = EM_FALSE;
  // Straight alpha, matching scene.frag's unpremultiplied output and the
  // SRC_ALPHA colour blend. Setting this true without ALSO changing the
  // colour blend factor to ONE would double-multiply alpha and darken the
  // translucent marker over the robot, not just over passthrough — two
  // coupled changes, so neither is made.
  attr.premultipliedAlpha = EM_FALSE;

  g_ctx = emscripten_webgl_create_context("#mxr-canvas", &attr);
  if (g_ctx <= 0) {
    Fail("WebGL2 context creation failed (handle %d)", static_cast<int>(g_ctx));
    return 1;
  }
  if (emscripten_webgl_make_context_current(g_ctx) != EMSCRIPTEN_RESULT_SUCCESS) {
    Fail("emscripten_webgl_make_context_current failed");
    return 1;
  }
  return 0;
}

MXR_ABI_EXPORT int mxr_menu_count(void) { return mxr_scene_count(); }

MXR_ABI_EXPORT const char* mxr_menu_id(int i) {
  const MxrScene* s = mxr_scene_at(i);
  return s ? s->id : "";
}

MXR_ABI_EXPORT const char* mxr_menu_label(int i) {
  const MxrScene* s = mxr_scene_at(i);
  return s ? s->label : "";
}

MXR_ABI_EXPORT int mxr_load_model(const char* scene_id) {
  if (g_ready) {
    Fail("mxr_load_model called twice; switching scenes is a page reload");
    return 1;
  }
  // `scene_id` is the `?scene=` query parameter, i.e. user input. Resolve it
  // through the table and then build the path from the TABLE's copy of the
  // id, never from the argument: an unknown id is a named error here rather
  // than an mj_loadXML failure on an attacker-shaped path.
  const MxrScene* scene = mxr_scene_by_id(scene_id);
  if (!scene) {
    Fail("unknown scene '%s' — not in src/robot_spec.c",
         scene_id ? scene_id : "(null)");
    return 1;
  }
  char path[128];
  // <id> IS the mount point: CMakeLists.txt preloads build/<id> at /<id>.
  // A disagreement between that and the table shows up right here, as a
  // named path in mxr_last_error().
  snprintf(path, sizeof(path), "/%s/ar_scene.xml", scene->id);

  char err[1024] = {0};
  // The staged tree is preloaded into MEMFS with its assets/ subdirectory
  // intact, so the robot XML's meshdir="assets" resolves natively and the VFS
  // argument stays null — no loader code at all on this target.
  g_model = mj_loadXML(path, nullptr, err, sizeof(err));
  if (!g_model) {
    Fail("mj_loadXML(%s) failed: %s", path, err);
    return 1;
  }
  LOGI("scene: %s (%s)", scene->label, scene->id);
  LOGI("model loaded: nq=%ld nv=%ld nu=%ld nmesh=%ld",
       static_cast<long>(g_model->nq), static_cast<long>(g_model->nv),
       static_cast<long>(g_model->nu), static_cast<long>(g_model->nmesh));
  g_data = mj_makeData(g_model);
  if (!g_renderer.Create(g_model)) {
    // Unwind rather than returning with g_model live behind g_ready == false.
    // The double-load guard at the top keys on g_ready, so this is the one
    // path that leaves mxr_load_model callable again — and a retry would
    // overwrite both pointers and leak the model plus its mjData. Nulling
    // them keeps "not ready" meaning "nothing allocated".
    if (g_data) {
      mj_deleteData(g_data);
      g_data = nullptr;
    }
    mj_deleteModel(g_model);
    g_model = nullptr;
    Fail("renderer creation failed");
    return 1;
  }
  // The clock named here is the one the shell intends to use; the shell
  // re-names it through mxr_set_clock_source once a session exists and the
  // actual source is known.
  g_sim.Init(g_model, g_data, "XRFrame.predictedDisplayTime");
  g_ready = true;
  return 0;
}

MXR_ABI_EXPORT const char* mxr_last_error(void) { return g_error; }

MXR_ABI_EXPORT void mxr_set_clock_source(const char* name) {
  // cwrap's 'string' argument lives on the wasm stack for the duration of
  // the call only, and SimScene stores the pointer, so copy it here.
  if (!name) {
    return;
  }
  snprintf(g_clock_source, sizeof(g_clock_source), "%s", name);
  g_sim.set_clock_source(g_clock_source);
}

MXR_ABI_EXPORT int mxr_max_views(void) { return kMaxViews; }
MXR_ABI_EXPORT int mxr_input_floats(void) { return kInputFloats; }
MXR_ABI_EXPORT int mxr_view_floats(void) { return kViewFloats; }
MXR_ABI_EXPORT int mxr_viewport_ints(void) { return kViewportInts; }

MXR_ABI_EXPORT float mxr_near_z(void) { return kNearZ; }
MXR_ABI_EXPORT float mxr_far_z(void) { return kFarZ; }

MXR_ABI_EXPORT float* mxr_input_buffer(void) { return g_input; }
MXR_ABI_EXPORT float* mxr_view_buffer(void) { return g_views; }
MXR_ABI_EXPORT int32_t* mxr_viewport_buffer(void) { return g_viewports; }

MXR_ABI_EXPORT void mxr_begin_frame(double t_display_s, int nviews, int fbo,
                                    int grip_valid, int a_down,
                                    int recenter_edge) {
  g_nviews = nviews < 0 ? 0 : (nviews > kMaxViews ? kMaxViews : nviews);
  if (nviews > kMaxViews && !g_warned_nviews) {
    g_warned_nviews = true;  // once: this is per frame, at 72-90 Hz
    LOGW("runtime reported %d views, arena holds %d — drawing the first %d",
         nviews, kMaxViews, kMaxViews);
  }

  // Bind and clear exactly ONCE, before any view is drawn, and BEFORE the
  // not-ready return: a model-load failure must still hand the compositor a
  // cleared (transparent) framebuffer, or the headset shows whatever the
  // swapchain image last held. `fbo` is the opaque XRWebGLLayer framebuffer
  // registered by shell.js, or 0 for the default framebuffer in an inline
  // session.
  glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(fbo));
  glClearColor(0, 0, 0, 0);  // transparent: the camera feed shows behind
  glClearDepthf(1.0f);
  glDepthMask(GL_TRUE);
  glDisable(GL_SCISSOR_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (!g_ready) {
    return;
  }

  // Field by field out of the homogeneous block, in the order abi.h
  // documents. Nothing here reads a computed offset.
  InputState in;
  in.grip_valid = grip_valid != 0;
  in.grip_pos[0] = g_input[0];
  in.grip_pos[1] = g_input[1];
  in.grip_pos[2] = g_input[2];
  in.grip_quat[0] = g_input[3];
  in.grip_quat[1] = g_input[4];
  in.grip_quat[2] = g_input[5];
  in.grip_quat[3] = g_input[6];
  in.trigger = g_input[7];
  in.squeeze = g_input[8];
  in.a_down = a_down != 0;
  in.recenter_edge = recenter_edge != 0;

  g_sim.Advance(in, t_display_s);
  g_scene = g_sim.Compose();
}

MXR_ABI_EXPORT void mxr_draw_view(int view_index) {
  if (!g_ready || view_index < 0 || view_index >= g_nviews) {
    return;
  }
  const int32_t* vp = g_viewports + view_index*kViewportInts;
  glEnable(GL_SCISSOR_TEST);
  glViewport(vp[0], vp[1], vp[2], vp[3]);
  glScissor(vp[0], vp[1], vp[2], vp[3]);
  const float* v = g_views + view_index*kViewFloats;
  g_renderer.SetView(v, v + 16);  // projection[16], then view[16]
  if (g_scene) {
    g_renderer.Draw(g_scene);
  }
}

MXR_ABI_EXPORT void mxr_end_session(void) {
  if (g_ready) {
    g_sim.EndSession();
  }
}
