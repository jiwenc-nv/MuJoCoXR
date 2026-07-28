#include "scene_renderer.h"

#include <cmath>
#include <cstring>

#include "frames.h"
#include "mxr_log.h"

namespace {

// The shader pair, embedded rather than fetched: they must arrive with the
// wasm module that was compiled against them, and a second network request
// is a second thing that can 404 in a headset browser.
const char* const kVertSrc =
#include "shaders/scene.vert.inc"
    ;
const char* const kFragSrc =
#include "shaders/scene.frag.inc"
    ;

// out = a * b, column-major 4x4. Twin of app/android/scene_renderer.cc.
void Mat4Mul(float out[16], const float a[16], const float b[16]) {
  float r[16];
  for (int c = 0; c < 4; ++c) {
    for (int row = 0; row < 4; ++row) {
      r[c*4 + row] = a[0*4 + row]*b[c*4 + 0] + a[1*4 + row]*b[c*4 + 1] +
                     a[2*4 + row]*b[c*4 + 2] + a[3*4 + row]*b[c*4 + 3];
    }
  }
  memcpy(out, r, sizeof(r));
}

GLuint CompileShader(GLenum stage, const char* src) {
  GLuint s = glCreateShader(stage);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char log[1024] = {0};
    glGetShaderInfoLog(s, sizeof(log) - 1, nullptr, log);
    LOGE("shader compile failed: %s", log);
    glDeleteShader(s);
    return 0;
  }
  return s;
}

}  // namespace

bool SceneRenderer::Create(const mjModel* m) {
  mxr_mat4_xr_from_mj(xr_from_mj_);
  if (!CreateProgram()) {
    return false;
  }

  MeshBuffers mb;
  BuildMeshBuffers(m, &mb);
  box_range_ = mb.box;
  mesh_ranges_ = mb.meshes;

  // WebGL2 has no base-vertex draw, so fold the per-mesh offset in once.
  // The ranges keep their base_vertex values for the census to read; only
  // the index array changes, and only in this renderer's copy.
  std::vector<uint32_t> abs = mb.indices;
  auto fold = [&abs](const MeshRange& r) {
    for (uint32_t k = 0; k < r.index_count; ++k) {
      abs[r.first_index + k] += static_cast<uint32_t>(r.base_vertex);
    }
  };
  fold(mb.box);
  for (const MeshRange& r : mb.meshes) {
    fold(r);
  }

  glGenVertexArrays(1, &vao_);
  glBindVertexArray(vao_);
  glGenBuffers(1, &vbuf_);
  glBindBuffer(GL_ARRAY_BUFFER, vbuf_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(mb.verts.size()*sizeof(Vertex)),
               mb.verts.data(), GL_STATIC_DRAW);
  glGenBuffers(1, &ibuf_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibuf_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(abs.size()*sizeof(uint32_t)),
               abs.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, normal)));
  glBindVertexArray(0);

  // Matches app/android/scene_renderer.cc:319-345 state for state: no
  // culling (mixed winding across the OBJ/STL assets), straight-alpha colour
  // blend with alpha ONE/ZERO, depth LEQUAL with writes, no sorting.
  glDisable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ZERO);
  glBlendEquation(GL_FUNC_ADD);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glDepthMask(GL_TRUE);
  return glGetError() == GL_NO_ERROR;
}

bool SceneRenderer::CreateProgram() {
  GLuint vs = CompileShader(GL_VERTEX_SHADER, kVertSrc);
  GLuint fs = CompileShader(GL_FRAGMENT_SHADER, kFragSrc);
  if (!vs || !fs) {
    return false;
  }
  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glBindAttribLocation(program_, 0, "in_pos");
  glBindAttribLocation(program_, 1, "in_normal");
  glLinkProgram(program_);
  glDeleteShader(vs);
  glDeleteShader(fs);
  GLint ok = 0;
  glGetProgramiv(program_, GL_LINK_STATUS, &ok);
  if (!ok) {
    char log[1024] = {0};
    glGetProgramInfoLog(program_, sizeof(log) - 1, nullptr, log);
    LOGE("program link failed: %s", log);
    return false;
  }
  u_viewproj_ = glGetUniformLocation(program_, "eye_viewproj");
  u_light_dir_ = glGetUniformLocation(program_, "eye_light_dir");
  u_model_ = glGetUniformLocation(program_, "pc_model");
  u_ncol0_ = glGetUniformLocation(program_, "pc_ncol0");
  u_ncol1_ = glGetUniformLocation(program_, "pc_ncol1");
  u_ncol2_ = glGetUniformLocation(program_, "pc_ncol2");
  u_color_ = glGetUniformLocation(program_, "pc_color");
  return true;
}

void SceneRenderer::SetView(const float proj[16], const float view[16]) {
  float pv[16], viewproj[16];
  Mat4Mul(pv, proj, view);
  Mat4Mul(viewproj, pv, xr_from_mj_);
  glUseProgram(program_);
  glUniformMatrix4fv(u_viewproj_, 1, GL_FALSE, viewproj);
  const float len = sqrtf(kLightDirWorld[0]*kLightDirWorld[0] +
                          kLightDirWorld[1]*kLightDirWorld[1] +
                          kLightDirWorld[2]*kLightDirWorld[2]);
  const float light[4] = {kLightDirWorld[0]/len, kLightDirWorld[1]/len,
                          kLightDirWorld[2]/len, 0.0f};
  glUniform4fv(u_light_dir_, 1, light);
}

void SceneRenderer::Draw(const mjvScene* scn) {
  glUseProgram(program_);
  glBindVertexArray(vao_);
  for (int i = 0; i < scn->ngeom; ++i) {
    const mjvGeom* g = scn->geoms + i;
    const MeshRange* range = nullptr;
    float scale[3] = {1, 1, 1};
    switch (g->type) {
      case mjGEOM_PLANE:
        continue;  // AR: no ground plane; passthrough is the background
      case mjGEOM_BOX:
        range = &box_range_;
        scale[0] = g->size[0];
        scale[1] = g->size[1];
        scale[2] = g->size[2];
        break;
      case mjGEOM_MESH: {
        // dataid = 2*meshid (mesh) or 2*meshid+1 (hull): render even only.
        if (g->dataid < 0 || (g->dataid & 1)) {
          continue;
        }
        const int meshid = g->dataid >> 1;
        if (meshid >= static_cast<int>(mesh_ranges_.size())) {
          continue;
        }
        range = &mesh_ranges_[meshid];
        break;
      }
      default:
        continue;  // census: MESH + BOX only
    }

    float model[16];
    // g->mat is row-major; column-major model[c*4+r] = mat[r*3+c] * scale[c].
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        model[c*4 + r] = g->mat[r*3 + c]*scale[c];
      }
      model[c*4 + 3] = 0;
    }
    model[12] = g->pos[0];
    model[13] = g->pos[1];
    model[14] = g->pos[2];
    model[15] = 1;
    // Normal matrix columns: rotation with inverse scale.
    float ncols[3][4];
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        ncols[c][r] = g->mat[r*3 + c]/scale[c];
      }
      ncols[c][3] = 0;
    }
    glUniformMatrix4fv(u_model_, 1, GL_FALSE, model);
    glUniform4fv(u_ncol0_, 1, ncols[0]);
    glUniform4fv(u_ncol1_, 1, ncols[1]);
    glUniform4fv(u_ncol2_, 1, ncols[2]);
    glUniform4fv(u_color_, 1, g->rgba);
    glDrawElements(
        GL_TRIANGLES, static_cast<GLsizei>(range->index_count),
        GL_UNSIGNED_INT,
        reinterpret_cast<void*>(static_cast<uintptr_t>(range->first_index)*
                                sizeof(uint32_t)));
  }
  glBindVertexArray(0);
}
