// mjvScene -> WebGL2, the twin of app/android/scene_renderer.h and
// deliberately the same shape: MESH + BOX only, one program, one hardcoded
// directional light, no textures/shadows/sorting. Geometry and appearance
// constants come from src/mesh_buffers.h so both targets draw the same
// pixels; only the API below differs.
//
// Three things this twin does NOT mirror, all on purpose:
//
// There is no ProjFromFov. WebXR supplies XRView.projectionMatrix already
// built and already in GL convention, so building one here would be a second
// definition of clip space for the two targets to disagree about.
//
// Indices are folded to ABSOLUTE at upload. WebGL2 has no base-vertex draw
// call, and the alternative — teaching the shared builder to emit absolute
// indices — would change the bytes in Android's index buffer, on a target
// with no hardware to re-verify. Five lines here instead.
//
// There is no Destroy. android_main genuinely returns, so the Vulkan twin
// tears down into a live process; a page does not — the tab going away
// reclaims the GL context, the wasm heap and this object together. A
// Destroy() here would have had zero callers, and a reader finding one would
// reasonably wire it to `beforeunload`/`visibilitychange`, which is the
// speculative page lifecycle this design refuses. GL objects here live
// exactly as long as the page does.

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
