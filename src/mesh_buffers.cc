#include "mesh_buffers.h"

#include <cstring>
#include <unordered_map>

#include "mxr_log.h"

void BuildMeshBuffers(const mjModel* m, MeshBuffers* out) {
  std::vector<Vertex>& verts = out->verts;
  std::vector<uint32_t>& indices = out->indices;
  verts.clear();
  indices.clear();

  // Unit box: 6 faces x 4 verts, half-extent 1.
  out->box.base_vertex = static_cast<int32_t>(verts.size());
  out->box.first_index = static_cast<uint32_t>(indices.size());
  const float bn[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  const float bu[6][3] = {{0, 1, 0}, {0, 1, 0},  {0, 0, 1},
                          {0, 0, 1}, {1, 0, 0},  {1, 0, 0}};
  uint32_t box_base = 0;
  for (int f = 0; f < 6; ++f) {
    float v[3];  // bv = n x u
    v[0] = bn[f][1]*bu[f][2] - bn[f][2]*bu[f][1];
    v[1] = bn[f][2]*bu[f][0] - bn[f][0]*bu[f][2];
    v[2] = bn[f][0]*bu[f][1] - bn[f][1]*bu[f][0];
    for (int i = 0; i < 4; ++i) {
      const float su = (i == 1 || i == 2) ? 1.0f : -1.0f;
      const float sv = (i >= 2) ? 1.0f : -1.0f;
      Vertex vert;
      for (int k = 0; k < 3; ++k) {
        vert.pos[k] = bn[f][k] + su*bu[f][k] + sv*v[k];
        vert.normal[k] = bn[f][k];
      }
      verts.push_back(vert);
    }
    const uint32_t b = box_base + 4*f;
    indices.insert(indices.end(), {b, b + 1, b + 2, b, b + 2, b + 3});
  }
  out->box.index_count =
      static_cast<uint32_t>(indices.size()) - out->box.first_index;

  // Meshes: weld unique (vertex, normal) index pairs per mesh — normals are
  // indexed separately from vertices (mesh_facenormal vs mesh_face).
  out->meshes.assign(m->nmesh, MeshRange());
  std::unordered_map<uint64_t, uint32_t> weld;
  for (int mesh = 0; mesh < m->nmesh; ++mesh) {
    weld.clear();
    MeshRange& range = out->meshes[mesh];
    range.base_vertex = static_cast<int32_t>(verts.size());
    range.first_index = static_cast<uint32_t>(indices.size());
    const float* mverts = m->mesh_vert + 3*m->mesh_vertadr[mesh];
    const float* mnormals = m->mesh_normal + 3*m->mesh_normaladr[mesh];
    const int faceadr = m->mesh_faceadr[mesh];
    uint32_t local_count = 0;
    for (int f = 0; f < m->mesh_facenum[mesh]; ++f) {
      for (int k = 0; k < 3; ++k) {
        const uint32_t vi = m->mesh_face[3*(faceadr + f) + k];
        const uint32_t ni = m->mesh_facenormal[3*(faceadr + f) + k];
        const uint64_t key = (static_cast<uint64_t>(vi) << 32) | ni;
        auto it = weld.find(key);
        uint32_t idx;
        if (it != weld.end()) {
          idx = it->second;
        } else {
          idx = local_count++;
          weld.emplace(key, idx);
          Vertex v;
          memcpy(v.pos, mverts + 3*vi, sizeof(v.pos));
          memcpy(v.normal, mnormals + 3*ni, sizeof(v.normal));
          verts.push_back(v);
        }
        indices.push_back(idx);
      }
    }
    range.index_count =
        static_cast<uint32_t>(indices.size()) - range.first_index;
  }
  LOGI("geometry: %zu vertices, %zu indices, %ld meshes", verts.size(),
       indices.size(), static_cast<long>(m->nmesh));
}
