// The only surface app/web/shell.js may touch. Internal, not a public API,
// versioned by build — wasm and JS come out of one `cmake --build`, so there
// is no skew to guard and no abi_version() here.
//
// One rule generates the whole shape:
//
//   JS NEVER COMPUTES A FIELD OFFSET. Every pointer JS writes through is
//   written at offset 0, and every block is a homogeneous array of one
//   scalar type.
//
// That is what rules out a struct crossing this line. Not "a struct is
// unsafe" — a HETEROGENEOUS one is: it brings padding, alignment,
// sizeof(bool) and wasm32 pointer width, and a size assert catches total
// drift while missing the likelier, silent failure of two fields swapping.
// float[9] has none of those questions; the element index IS the offset.
//
// It also rules out the other direction, a flat scalar signature: two views
// carry ~41 positional arguments, and Emscripten does not type-check direct
// _fn calls in a release build, so `float px` and `float qx` are
// interchangeable to the loader.
//
// Every STRIDE is QUERIED, never agreed. wasm reports each length and JS
// hardcodes none. An assert says you and I must agree; a query says I will
// tell you, and a stranger cannot get a queried number wrong. Field order
// WITHIN a block is the one thing JS does hardcode — against the contract
// written beside each buffer below, which is why the contract is written.
//
// Frame calls come in exactly one order, and the split is load-bearing:
// mxr_begin_frame binds and clears ONCE, mxr_draw_view only draws. The most
// common WebXR stereo bug is clearing inside the per-view loop, which wipes
// the first eye; this shape makes that unrepresentable rather than
// commented against.
//
// THE SEQUENCE. Stated because "exactly one order" is useless without it.
//
//   startup, once per page:
//     mxr_init()                  -> 0, or read mxr_last_error()
//     mxr_menu_count(), mxr_menu_id(i), mxr_menu_label(i)
//                                 -- the scene catalogue, for building the
//                                    picker. Available BEFORE any model
//                                    exists, which is the point: nothing is
//                                    compiled until a scene is chosen.
//
//   startup, ZERO OR ONE TIMES per page — the shell branches here:
//     mxr_load_model(scene_id)    -> 0, or read mxr_last_error()
//                                 -- a page opened with no `?scene=` is a
//                                    working picker and compiles NOTHING, so
//                                    everything below this line is reached
//                                    only once a scene has been chosen.
//
//   after a successful mxr_load_model, once per page:
//     mxr_max_views(), mxr_input_floats(), mxr_view_floats(),
//     mxr_viewport_ints(), mxr_input_buffer(), mxr_view_buffer(),
//     mxr_viewport_buffer(), mxr_near_z(), mxr_far_z()
//                                 -- all constant for the page's life and
//                                    none of them model-dependent; query
//                                    once and cache. (They are in fact
//                                    callable before the load; they are
//                                    listed here because that is where the
//                                    shell reads them.)
//
//   first frame of each session, not session entry:
//     mxr_set_clock_source(name)  -- names the shell's ONE latched clock,
//                                    and only if it is not the one already
//                                    named at load. An XRFrame is the first
//                                    moment predictedDisplayTime can be
//                                    tested for at all, so the source is not
//                                    knowable at requestSession time.
//
//   per frame, in this order and no other:
//     write the input block (offset 0, see below)
//     write view[i] and viewport[i] for i < nviews
//     mxr_begin_frame(t_display_s, nviews, fbo, grip_valid, a_down,
//                     recenter_edge)
//                                 -- steps physics, composes the scene,
//                                    binds `fbo` and clears it ONCE
//     mxr_draw_view(i) for i in [0, nviews)   -- draws only
//
//   session end:
//     mxr_end_session()
//
// The CALLER clamps nothing: mxr_begin_frame clamps nviews to
// mxr_max_views() itself and draws the first N. mxr_draw_view ignores an
// out-of-range index. There is no shutdown call — a page teardown reclaims
// the GL context and the wasm heap, so there is nothing left to free.

#ifndef MUJOCOXR_APP_WEB_ABI_H_
#define MUJOCOXR_APP_WEB_ABI_H_

#include <stdint.h>

#include <emscripten/emscripten.h>

// grep -r MXR_ABI_EXPORT lists the entire export surface. Applied to
// definitions rather than -sEXPORTED_FUNCTIONS so the list cannot drift from
// the code. `extern "C"` makes this a C++-only header despite the .h: its
// only includer is app/web/main.cc, and JS reads the declarations below with
// its eyes, not with a compiler.
#define MXR_ABI_EXPORT EMSCRIPTEN_KEEPALIVE extern "C"

// --- lifecycle ---------------------------------------------------------
// Both return 0 on success; on failure mxr_last_error() explains.
//
// mxr_init creates the WebGL2 context on the canvas whose element id is
// `#mxr-canvas` — a hardcoded contract with app/web/index.html, and the one
// piece of this ABI that is not visible in a signature.
MXR_ABI_EXPORT int mxr_init(void);        // GL context, shaders, GL state

// The scene catalogue, straight out of src/robot_spec.c, so the page carries
// NO per-robot content: app/web/index.html has an empty container and
// shell.js builds one link per entry. Adding a robot therefore cannot
// produce the failure where a scene exists in C and is unreachable from the
// UI, which is what a hand-written list of links would eventually do.
//
// These three are named `menu` rather than `scene` because they are a
// genuinely different interface from src/robot_spec.h's mxr_scene_count/at/
// by_id: a `const MxrScene*` cannot cross this ABI, so these are index-and-
// string-returning and answer out-of-range with "" rather than NULL. The
// naming is forced as well as chosen — the src/ symbols are ordinary linked
// symbols in the same binary, so reusing their names would be a
// duplicate-symbol link error rather than an override.
//
// mxr_menu_count has nothing to marshal — it is `int(void)` and so is
// mxr_scene_count — so the honest question is why it is not just
// MXR_ABI_EXPORT on the original. Because MXR_ABI_EXPORT is defined in THIS
// header, and src/robot_spec.h is in the portable tier: exporting it there would
// put a web-target concern into a file that Android and the host binaries
// also compile. The twin keeps that boundary at the cost of one forwarding
// line, which is the right trade in that direction.
// Index i is valid for [0, mxr_menu_count()); out of range returns "".
MXR_ABI_EXPORT int         mxr_menu_count(void);
MXR_ABI_EXPORT const char* mxr_menu_id(int i);     // the `?scene=` token
MXR_ABI_EXPORT const char* mxr_menu_label(int i);  // shown to a human

// MEMFS -> mj_loadXML -> geometry, for one catalogue entry. CALLED ZERO OR
// ONE TIMES PER PAGE: the page compiles nothing until a scene is picked,
// since compiling a 67-mesh model the user did not ask for is ~11 MB of VBO
// and a second or so of a headset browser's time.
//
// `scene_id` must be an id from mxr_menu_id; anything else is a graceful
// failure naming the id, because this value arrives from the query string and
// is therefore user input.
//
// THE TWO FAILURES ARE NOT DISTINGUISHABLE FROM THE RETURN VALUE, and both
// return 1: "you passed an id that is not in the table" (a programmer bug,
// or a stale bookmark) and "the id is right but its files are not in the
// bundle" (the packaging bug this header warns about at MxrScene::id — a
// staging line that disagrees with the table). Telling them apart today means
// substring-matching mxr_last_error(): the first says `unknown scene '<id>'`,
// the second `mj_loadXML(/<id>/ar_scene.xml) failed`. Both name the id, which
// is enough to act on; a distinct return code would be better and nothing
// needs one yet.
//
// There is no unload and no second call: switching scenes is a page RELOAD
// with a different `?scene=`, which is the only teardown that reclaims the GL
// context, the wasm heap and SimScene's latched clock together. A second call
// is refused rather than leaking the live model.
MXR_ABI_EXPORT int mxr_load_model(const char* scene_id);
// Named `last_error` rather than `error` because src/mxr_error.h is a
// different and near-opposite thing (MuJoCo's mju_user_error hooks, which
// abort); this is the string you read after a graceful non-zero return.
MXR_ABI_EXPORT const char* mxr_last_error(void);  // static buf; UTF8ToString

// Names the shell's latched per-session clock, for the stalled-clock
// diagnostic. Copied into a static buffer, so the JS string need not
// outlive the call. Called on the FIRST FRAME of a session rather than at
// session entry, because WebXR's predictedDisplayTime is a property of
// XRFrame: until one exists there is nothing to test, and a UA that omits it
// forces a different source — which the log line must then name, or the
// diagnostic points at the wrong clock. mxr_load_model already names the
// intended source, so a shell that gets what it expected need not call this
// at all.
MXR_ABI_EXPORT void mxr_set_clock_source(const char* name);

// Drop the clutch and re-latch the clock. Called on XR session end, so
// re-entering does not resume with a latched clutch and an accumulator
// holding the entire time the tab spent outside the session.
MXR_ABI_EXPORT void mxr_end_session(void);

// --- shapes: query these, never assume them ----------------------------
MXR_ABI_EXPORT int mxr_max_views(void);       // capacity of the view arena
MXR_ABI_EXPORT int mxr_input_floats(void);    // floats in the input block
MXR_ABI_EXPORT int mxr_view_floats(void);     // floats PER VIEW
MXR_ABI_EXPORT int mxr_viewport_ints(void);   // int32s PER VIEW

// --- clip planes: query these too ---------------------------------------
// Not shapes — these are scene geometry in metres, and they are up here with
// the strides only because they share the query-once lifetime.
//
// Clip planes, in metres, for XRSession.updateRenderState. Queried for the
// same reason every stride is: src/mesh_buffers.h owns these two numbers for
// every renderer, and WebXR's defaults (0.1 / 1000) clip the gripper at arm's
// length. A JS literal here would be the one value agreed rather than
// queried, and drift would be undetectable because nothing on this target
// reads the C constants.
MXR_ABI_EXPORT float mxr_near_z(void);
MXR_ABI_EXPORT float mxr_far_z(void);

// --- buffers: JS writes each at offset 0 -------------------------------
// input : [0..2] grip_pos, [3..6] grip_quat (xyzw), [7] trigger, [8] squeeze
MXR_ABI_EXPORT float* mxr_input_buffer(void);
// views : per view, projection[16] then WORLD-TO-VIEW[16] — the INVERSE of
// the view pose (WebXR: view.transform.inverse.matrix, never
// view.transform.matrix, which produces a scene that moves with the head).
// Both column-major.
MXR_ABI_EXPORT float* mxr_view_buffer(void);
// viewports : per view, x, y, width, height
MXR_ABI_EXPORT int32_t* mxr_viewport_buffer(void);

// --- the frame ---------------------------------------------------------
// t_display_s is SECONDS as a double and never travels in the float block.
// A millisecond timestamp narrowed to float against a large
// performance.timeOrigin loses resolution catastrophically, and the symptom
// — physics frozen, rendering perfect — is indistinguishable from a units
// bug. The core derives dt, so the real requirement is not that the value
// be absolute but that it be the SAME CLOCK on every frame of a session:
// two timebases interleaved inject an offset the core reads as real motion.
// Name that clock through mxr_set_clock_source. A non-finite value is
// treated as a stalled clock and reported, never propagated.
//
// `fbo` is the integer name of XRWebGLLayer's opaque framebuffer, registered
// by shell.js into Emscripten's GL table. 0 means the default framebuffer,
// which is what an inline session gives.
//
// grip_valid == 0 means the controller pose in the input block is not
// trustworthy this frame — lost tracking, no grip space, or (shell policy)
// the tab is not visible. The core then drops the clutch and does not read
// grip_pos/grip_quat at all, so a shell may leave last frame's bytes there.
//
// a_down is the raw button LEVEL; the core owns the edge. recenter_edge is
// latched by the shell for exactly one frame.
MXR_ABI_EXPORT void mxr_begin_frame(double t_display_s, int nviews, int fbo,
                                    int grip_valid, int a_down,
                                    int recenter_edge);
MXR_ABI_EXPORT void mxr_draw_view(int view_index);

#endif  // MUJOCOXR_APP_WEB_ABI_H_
