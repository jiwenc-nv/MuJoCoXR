// mjvScene -> WebGL2, the twin of app/openxr/scene_renderer.h and
// deliberately the same shape: MESH + BOX only, one program, one hardcoded
// directional light, no textures/shadows/sorting. Geometry and appearance
// constants come from src/mesh_buffers.h so both renderers draw the same
// pixels; only the API below differs. RENDERERS, not targets — there are two
// of these and three targets, because Android and Linux share the Vulkan one.
//
// Three things this twin does NOT mirror, all on purpose:
//
// There is no ProjFromFov. WebXR supplies XRView.projectionMatrix already
// built and already in GL convention, so building one here would be a second
// definition of clip space for the two renderers to disagree about.
//
// Indices are folded to ABSOLUTE at upload. WebGL2 has no base-vertex draw
// call, and the alternative — teaching the shared builder to emit absolute
// indices — would change the bytes in Android's index buffer, on a target
// with no hardware to re-verify. Five lines here instead.
//
// There is no Destroy, and the reason is now a caller rather than an
// argument. The Vulkan twin has one because android_main returns into a live
// process and app/android/main.cc calls SceneRenderer::Destroy() at two
// points: at exit, and on every B-press, because switching scenes there means
// rebuilding the model in-process. (app/linux/main.cc is on THIS side of the
// contrast, not Android's: it loads one scene per process and picks it with
// --scene, so it too has a single teardown at exit.) THIS renderer has no
// such caller and
// cannot acquire one, because app/web/shell.js switches scenes by navigating
// to a new ?scene= — the page teardown reclaims the GL context, the wasm heap
// and this object together, and it is also the only teardown that clears
// SimScene's latched clock. So the targets differ here because their
// scene-switch mechanisms differ, not because one of them forgot.
//
// If you are adding a Destroy() here, the question to answer first is what
// calls it. `beforeunload`/`visibilitychange` is not an answer — that is a
// speculative page lifecycle, and the browser already does the work.

#ifndef MUJOCOXR_APP_WEB_SCENE_RENDERER_H_
#define MUJOCOXR_APP_WEB_SCENE_RENDERER_H_

#include <cstdint>
#include <vector>

#include <GLES3/gl3.h>

#include <mujoco/mujoco.h>

#include "mesh_buffers.h"

class SceneRenderer {
 public:
  bool Create(const mjModel* m);
  // Uploads projection * view * xr_from_mj for the view about to be drawn.
  // Both matrices are column-major, straight from WebXR.
  void SetView(const float proj[16], const float view[16]);
  // Records draws for every renderable geom in scene order, so app-appended
  // decor lands last and blends over the robot.
  void Draw(const mjvScene* scn);

 private:
  bool CreateProgram();

  GLuint program_ = 0;
  GLuint vao_ = 0;
  GLuint vbuf_ = 0;
  GLuint ibuf_ = 0;
  GLint u_viewproj_ = -1;
  GLint u_light_dir_ = -1;
  GLint u_model_ = -1;
  GLint u_ncol0_ = -1;
  GLint u_ncol1_ = -1;
  GLint u_ncol2_ = -1;
  GLint u_color_ = -1;
  MeshRange box_range_;
  std::vector<MeshRange> mesh_ranges_;
  float xr_from_mj_[16] = {0};
};

#endif  // MUJOCOXR_APP_WEB_SCENE_RENDERER_H_
