#include "scene_renderer.h"

#include <cmath>
#include <cstring>
#include <unordered_map>

#include "frames.h"
#include "mxr_log.h"
#include "vk_context.h"

#include "shaders/scene.frag.spv.h"
#include "shaders/scene.vert.spv.h"

namespace {

constexpr float kNearZ = 0.05f;
constexpr float kFarZ = 50.0f;
constexpr float kInfinitePlaneHalfExtent = 10.0f;  // floor size[0] == 0
const float kLightDirWorld[3] = {0.35f, -0.25f, -1.0f};  // one light, fixed

struct Vertex {
  float pos[3];
  float normal[3];
};

// 128 bytes — the Vulkan-guaranteed push constant budget, used exactly.
struct PushConstants {
  float model[16];  // column-major world-from-local (rotation*scale, pos)
  float ncol0[4];   // normal-matrix columns; ncol0.w > 0.5 => checker
  float ncol1[4];
  float ncol2[4];
  float color[4];
};
static_assert(sizeof(PushConstants) == 128, "push constant budget");

struct EyeUbo {
  float viewproj[16];
  float light_dir[4];
};

// out = a * b, column-major 4x4.
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

// Vulkan-convention projection (y-down clip, depth 0..1) from an XrFovf.
void ProjFromFov(const XrFovf& fov, float out[16]) {
  const float tl = tanf(fov.angleLeft);
  const float tr = tanf(fov.angleRight);
  const float tu = tanf(fov.angleUp);
  const float td = tanf(fov.angleDown);
  memset(out, 0, 16*sizeof(float));
  out[0] = 2.0f/(tr - tl);
  out[8] = (tr + tl)/(tr - tl);
  // (td - tu) < 0 flips y for Vulkan clip space.
  out[5] = 2.0f/(td - tu);
  out[9] = (td + tu)/(td - tu);
  out[10] = kFarZ/(kNearZ - kFarZ);
  out[14] = (kFarZ*kNearZ)/(kNearZ - kFarZ);
  out[11] = -1.0f;
}

// Inverse of a rigid pose (XR view pose = eye-in-stage): V = [R^T | -R^T t].
void ViewFromPose(const XrPosef& pose, float out[16]) {
  const float x = pose.orientation.x, y = pose.orientation.y,
              z = pose.orientation.z, w = pose.orientation.w;
  // Row-major R from quaternion.
  const float R[9] = {
      1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y),
      2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x),
      2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)};
  const float t[3] = {pose.position.x, pose.position.y, pose.position.z};
  // Column-major out: rotation part = R^T -> out[c*4+r] = R^T[r][c] = R[c*3+r]
  memset(out, 0, 16*sizeof(float));
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out[c*4 + r] = R[c*3 + r];
    }
  }
  for (int r = 0; r < 3; ++r) {
    out[12 + r] = -(R[0*3 + r]*t[0] + R[1*3 + r]*t[1] + R[2*3 + r]*t[2]);
  }
  out[15] = 1.0f;
}

VkShaderModule MakeShaderModule(VkDevice dev, const uint32_t* code,
                                size_t size_bytes) {
  VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  info.codeSize = size_bytes;
  info.pCode = code;
  VkShaderModule mod = VK_NULL_HANDLE;
  if (vkCreateShaderModule(dev, &info, nullptr, &mod) != VK_SUCCESS) {
    LOGE("vkCreateShaderModule failed");
  }
  return mod;
}

}  // namespace

bool SceneRenderer::Create(VkContext* vk, const mjModel* m) {
  vk_ = vk;
  mxr_stage_from_world(stage_from_world_);
  if (!UploadGeometry(m)) {
    return false;
  }
  if (!CreatePipeline()) {
    return false;
  }
  // Per-eye UBOs, persistently mapped; descriptor sets bound once.
  VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 2};
  VkDescriptorPoolCreateInfo pool_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
  pool_info.maxSets = 2;
  pool_info.poolSizeCount = 1;
  pool_info.pPoolSizes = &pool_size;
  if (vkCreateDescriptorPool(vk_->device(), &pool_info, nullptr, &pool_) !=
      VK_SUCCESS) {
    return false;
  }
  for (int i = 0; i < 2; ++i) {
    if (!vk_->CreateBuffer(sizeof(EyeUbo),
                           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                               VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                           &ubo_[i], &ubo_mem_[i])) {
      return false;
    }
    vkMapMemory(vk_->device(), ubo_mem_[i], 0, sizeof(EyeUbo), 0,
                &ubo_map_[i]);
    VkDescriptorSetAllocateInfo alloc{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    alloc.descriptorPool = pool_;
    alloc.descriptorSetCount = 1;
    alloc.pSetLayouts = &dsl_;
    if (vkAllocateDescriptorSets(vk_->device(), &alloc, &sets_[i]) !=
        VK_SUCCESS) {
      return false;
    }
    VkDescriptorBufferInfo buf{ubo_[i], 0, sizeof(EyeUbo)};
    VkWriteDescriptorSet write{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    write.dstSet = sets_[i];
    write.dstBinding = 0;
    write.descriptorCount = 1;
    write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    write.pBufferInfo = &buf;
    vkUpdateDescriptorSets(vk_->device(), 1, &write, 0, nullptr);
  }
  return true;
}

bool SceneRenderer::UploadGeometry(const mjModel* m) {
  std::vector<Vertex> verts;
  std::vector<uint32_t> indices;

  // Unit plane: quad in local xy, normal +z.
  plane_range_.base_vertex = 0;
  plane_range_.first_index = 0;
  plane_range_.index_count = 6;
  verts.insert(verts.end(), {{{-1, -1, 0}, {0, 0, 1}},
                             {{1, -1, 0}, {0, 0, 1}},
                             {{1, 1, 0}, {0, 0, 1}},
                             {{-1, 1, 0}, {0, 0, 1}}});
  indices.insert(indices.end(), {0, 1, 2, 0, 2, 3});

  // Unit box: 6 faces x 4 verts, half-extent 1.
  box_range_.base_vertex = static_cast<int32_t>(verts.size());
  box_range_.first_index = static_cast<uint32_t>(indices.size());
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
  box_range_.index_count =
      static_cast<uint32_t>(indices.size()) - box_range_.first_index;

  // Meshes: weld unique (vertex, normal) index pairs per mesh — normals are
  // indexed separately from vertices (mesh_facenormal vs mesh_face).
  mesh_ranges_.resize(m->nmesh);
  std::unordered_map<uint64_t, uint32_t> weld;
  for (int mesh = 0; mesh < m->nmesh; ++mesh) {
    weld.clear();
    auto& range = mesh_ranges_[mesh];
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

  const VkDeviceSize vsize = verts.size()*sizeof(Vertex);
  const VkDeviceSize isize = indices.size()*sizeof(uint32_t);
  const VkMemoryPropertyFlags host_props =
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
  if (!vk_->CreateBuffer(vsize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, host_props,
                         &vbuf_, &vmem_) ||
      !vk_->CreateBuffer(isize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT, host_props,
                         &ibuf_, &imem_)) {
    return false;
  }
  void* map = nullptr;
  vkMapMemory(vk_->device(), vmem_, 0, vsize, 0, &map);
  memcpy(map, verts.data(), vsize);
  vkUnmapMemory(vk_->device(), vmem_);
  vkMapMemory(vk_->device(), imem_, 0, isize, 0, &map);
  memcpy(map, indices.data(), isize);
  vkUnmapMemory(vk_->device(), imem_);
  return true;
}

bool SceneRenderer::CreatePipeline() {
  VkDevice dev = vk_->device();

  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  binding.descriptorCount = 1;
  binding.stageFlags =
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  VkDescriptorSetLayoutCreateInfo dsl_info{
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
  dsl_info.bindingCount = 1;
  dsl_info.pBindings = &binding;
  if (vkCreateDescriptorSetLayout(dev, &dsl_info, nullptr, &dsl_) !=
      VK_SUCCESS) {
    return false;
  }

  VkPushConstantRange pc_range{
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
      sizeof(PushConstants)};
  VkPipelineLayoutCreateInfo pl_info{
      VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  pl_info.setLayoutCount = 1;
  pl_info.pSetLayouts = &dsl_;
  pl_info.pushConstantRangeCount = 1;
  pl_info.pPushConstantRanges = &pc_range;
  if (vkCreatePipelineLayout(dev, &pl_info, nullptr, &layout_) != VK_SUCCESS) {
    return false;
  }

  VkShaderModule vs =
      MakeShaderModule(dev, scene_vert_spv, sizeof(scene_vert_spv));
  VkShaderModule fs =
      MakeShaderModule(dev, scene_frag_spv, sizeof(scene_frag_spv));
  if (vs == VK_NULL_HANDLE || fs == VK_NULL_HANDLE) {
    return false;
  }

  VkPipelineShaderStageCreateInfo stages[2]{};
  stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
  stages[0].module = vs;
  stages[0].pName = "main";
  stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  stages[1].module = fs;
  stages[1].pName = "main";

  VkVertexInputBindingDescription vbind{0, sizeof(Vertex),
                                        VK_VERTEX_INPUT_RATE_VERTEX};
  VkVertexInputAttributeDescription vattrs[2] = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)}};
  VkPipelineVertexInputStateCreateInfo vin{
      VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
  vin.vertexBindingDescriptionCount = 1;
  vin.pVertexBindingDescriptions = &vbind;
  vin.vertexAttributeDescriptionCount = 2;
  vin.pVertexAttributeDescriptions = vattrs;

  VkPipelineInputAssemblyStateCreateInfo ia{
      VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
  ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

  VkPipelineViewportStateCreateInfo vp{
      VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
  vp.viewportCount = 1;
  vp.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo rs{
      VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
  rs.polygonMode = VK_POLYGON_MODE_FILL;
  rs.cullMode = VK_CULL_MODE_NONE;  // POC: mixed winding across OBJ/STL
  rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  rs.lineWidth = 1.0f;

  VkPipelineMultisampleStateCreateInfo ms{
      VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
  ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

  VkPipelineDepthStencilStateCreateInfo ds{
      VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
  ds.depthTestEnable = VK_TRUE;
  ds.depthWriteEnable = VK_TRUE;
  ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

  VkPipelineColorBlendAttachmentState blend{};
  blend.blendEnable = VK_TRUE;  // marker decor is translucent, drawn last
  blend.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
  blend.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  blend.colorBlendOp = VK_BLEND_OP_ADD;
  blend.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
  blend.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
  blend.alphaBlendOp = VK_BLEND_OP_ADD;
  blend.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
  VkPipelineColorBlendStateCreateInfo cb{
      VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
  cb.attachmentCount = 1;
  cb.pAttachments = &blend;

  VkDynamicState dyn_states[2] = {VK_DYNAMIC_STATE_VIEWPORT,
                                  VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dyn{
      VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
  dyn.dynamicStateCount = 2;
  dyn.pDynamicStates = dyn_states;

  VkGraphicsPipelineCreateInfo info{
      VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
  info.stageCount = 2;
  info.pStages = stages;
  info.pVertexInputState = &vin;
  info.pInputAssemblyState = &ia;
  info.pViewportState = &vp;
  info.pRasterizationState = &rs;
  info.pMultisampleState = &ms;
  info.pDepthStencilState = &ds;
  info.pColorBlendState = &cb;
  info.pDynamicState = &dyn;
  info.layout = layout_;
  info.renderPass = vk_->render_pass();
  info.subpass = 0;
  VkResult r = vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &info,
                                         nullptr, &pipeline_);
  vkDestroyShaderModule(dev, vs, nullptr);
  vkDestroyShaderModule(dev, fs, nullptr);
  if (r != VK_SUCCESS) {
    LOGE("vkCreateGraphicsPipelines failed: %d", r);
    return false;
  }
  return true;
}

void SceneRenderer::SetEye(int eye, const XrPosef& view_pose,
                           const XrFovf& fov) {
  float proj[16], view[16], pv[16];
  ProjFromFov(fov, proj);
  ViewFromPose(view_pose, view);
  Mat4Mul(pv, proj, view);
  EyeUbo ubo;
  Mat4Mul(ubo.viewproj, pv, stage_from_world_);
  const float len = sqrtf(kLightDirWorld[0]*kLightDirWorld[0] +
                          kLightDirWorld[1]*kLightDirWorld[1] +
                          kLightDirWorld[2]*kLightDirWorld[2]);
  for (int i = 0; i < 3; ++i) {
    ubo.light_dir[i] = kLightDirWorld[i]/len;
  }
  ubo.light_dir[3] = 0;
  memcpy(ubo_map_[eye], &ubo, sizeof(ubo));
}

void SceneRenderer::Draw(VkCommandBuffer cmd, int eye, const mjvScene* scn) {
  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout_, 0, 1,
                          &sets_[eye], 0, nullptr);
  VkDeviceSize zero = 0;
  vkCmdBindVertexBuffers(cmd, 0, 1, &vbuf_, &zero);
  vkCmdBindIndexBuffer(cmd, ibuf_, 0, VK_INDEX_TYPE_UINT32);

  for (int i = 0; i < scn->ngeom; ++i) {
    const mjvGeom* g = scn->geoms + i;
    const MeshRange* range = nullptr;
    float scale[3] = {1, 1, 1};
    bool checker = false;
    switch (g->type) {
      case mjGEOM_PLANE:
        range = &plane_range_;
        scale[0] = g->size[0] > 0 ? g->size[0] : kInfinitePlaneHalfExtent;
        scale[1] = g->size[1] > 0 ? g->size[1] : kInfinitePlaneHalfExtent;
        checker = true;
        break;
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
        continue;  // census: PLANE + MESH + BOX only
    }

    PushConstants pc;
    // g->mat is row-major; column-major model[c*4+r] = mat[r*3+c] * scale[c].
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        pc.model[c*4 + r] = g->mat[r*3 + c]*scale[c];
      }
      pc.model[c*4 + 3] = 0;
    }
    pc.model[12] = g->pos[0];
    pc.model[13] = g->pos[1];
    pc.model[14] = g->pos[2];
    pc.model[15] = 1;
    // Normal matrix columns: rotation with inverse scale.
    float* ncols[3] = {pc.ncol0, pc.ncol1, pc.ncol2};
    for (int c = 0; c < 3; ++c) {
      for (int r = 0; r < 3; ++r) {
        ncols[c][r] = g->mat[r*3 + c]/scale[c];
      }
      ncols[c][3] = 0;
    }
    pc.ncol0[3] = checker ? 1.0f : 0.0f;
    memcpy(pc.color, g->rgba, sizeof(pc.color));

    vkCmdPushConstants(cmd, layout_,
                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                       0, sizeof(pc), &pc);
    vkCmdDrawIndexed(cmd, range->index_count, 1, range->first_index,
                     range->base_vertex, 0);
  }
}

void SceneRenderer::Destroy() {
  if (!vk_) {
    return;
  }
  VkDevice dev = vk_->device();
  vkDeviceWaitIdle(dev);
  for (int i = 0; i < 2; ++i) {
    if (ubo_map_[i]) {
      vkUnmapMemory(dev, ubo_mem_[i]);
    }
    if (ubo_[i]) {
      vkDestroyBuffer(dev, ubo_[i], nullptr);
    }
    if (ubo_mem_[i]) {
      vkFreeMemory(dev, ubo_mem_[i], nullptr);
    }
  }
  if (vbuf_) {
    vkDestroyBuffer(dev, vbuf_, nullptr);
  }
  if (vmem_) {
    vkFreeMemory(dev, vmem_, nullptr);
  }
  if (ibuf_) {
    vkDestroyBuffer(dev, ibuf_, nullptr);
  }
  if (imem_) {
    vkFreeMemory(dev, imem_, nullptr);
  }
  if (pipeline_) {
    vkDestroyPipeline(dev, pipeline_, nullptr);
  }
  if (layout_) {
    vkDestroyPipelineLayout(dev, layout_, nullptr);
  }
  if (pool_) {
    vkDestroyDescriptorPool(dev, pool_, nullptr);
  }
  if (dsl_) {
    vkDestroyDescriptorSetLayout(dev, dsl_, nullptr);
  }
  vk_ = nullptr;
}
