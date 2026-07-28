// Everything the two renderers must agree on to put the same pixels on
// screen, built once from mjModel and owned by neither of them: the welded
// vertex/index buffers, plus the geometry and appearance constants the C++
// side itself consumes.
//
// That last clause is the line actually drawn, and it is narrower than
// "every constant the two renderers share": constants the C++ code READS
// live here (kNearZ/kFarZ go to updateRenderState and to Android's
// projection; kLightDirWorld is normalized and uploaded as a uniform);
// constants only GLSL reads stay in GLSL, where `diff` over the twin shader
// pair covers them. See the note on `ambient` below.
//
// Meshes are de-indexed at load by welding unique (vertex, normal) index
// pairs, because mesh_facenormal indexes normals separately from mesh_face
// and a GPU vertex must carry both.
//
// Indices stay MESH-LOCAL with a per-mesh base_vertex, rather than absolute.
// That is what vkCmdDrawIndexed's vertexOffset consumes directly, so the
// Android buffer contents are unchanged by this extraction. A renderer with
// no base-vertex draw call folds the offset in at upload instead — five
// lines in the one renderer that needs it, rather than the shared builder
// emitting one consumer's convention for the other to undo. Absolute indices
// would be arithmetically identical (max welded index is bounded by
// 3*nmeshface ~ 410k, far inside uint32), but they would change bytes on a
// target with no hardware to re-verify them on.

#ifndef MUJOCOXR_SRC_MESH_BUFFERS_H_
#define MUJOCOXR_SRC_MESH_BUFFERS_H_

#include <cstdint>
#include <vector>

#include <mujoco/mujoco.h>

// Clip planes. The web shell also hands these to
// XRSession.updateRenderState — WebXR defaults to 0.1/1000, which clips the
// gripper at arm's length and wrecks depth precision. It reads them through
// mxr_near_z()/mxr_far_z() rather than repeating the literals in JS, so
// these two remain the only definition on that target as well.
constexpr float kNearZ = 0.05f;
constexpr float kFarZ = 50.0f;

// The one directional light, in MuJoCo world space, normalized on upload.
// The half-lambert `ambient` term is deliberately NOT here: no C++ code
// reads it, so hoisting it would mean a uniform plus a plumbing path in both
// renderers to share one float. It stays a `const float` inside each
// fragment shader, spelled byte-identically in both, so `diff` over the twin
// pair catches a drift in it.
constexpr float kLightDirWorld[3] = {0.35f, -0.25f, -1.0f};

struct Vertex {
  float pos[3];
  float normal[3];
};

struct MeshRange {
  int32_t base_vertex = 0;
  uint32_t first_index = 0;
  uint32_t index_count = 0;
};

struct MeshBuffers {
  std::vector<Vertex> verts;
  std::vector<uint32_t> indices;  // mesh-local: add base_vertex to deref
  MeshRange box;                  // unit box, half-extent 1
  std::vector<MeshRange> meshes;  // indexed by meshid
};

// Welds every mesh in `m` plus the unit box into one vertex/index pair.
void BuildMeshBuffers(const mjModel* m, MeshBuffers* out);

#endif  // MUJOCOXR_SRC_MESH_BUFFERS_H_
