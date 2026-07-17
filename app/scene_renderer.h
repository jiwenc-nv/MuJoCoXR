// mjvScene -> Vulkan, scene-specific by design:
// PLANE + MESH + BOX only, one pipeline, push constants, one hardcoded
// directional light, procedural checker floor, no textures/shadows/sorting.
// View/projection come exclusively from XrView pose + XrFovf; mjvGLCamera is
// bypassed. Meshes are de-indexed at load by welding unique (vertex, normal)
// index pairs — mesh_facenormal indexes normals separately from vertices.

#ifndef MUJOCOXR_APP_SCENE_RENDERER_H_
#define MUJOCOXR_APP_SCENE_RENDERER_H_

#include <vulkan/vulkan.h>

#include <openxr/openxr.h>

#include <mujoco/mujoco.h>

#include <cstdint>
#include <vector>

class VkContext;

class SceneRenderer {
 public:
  bool Create(VkContext* vk, const mjModel* m);
  // Computes P(fov) * V(pose)^-1 * stage_T_world and writes the eye's UBO.
  void SetEye(int eye, const XrPosef& view_pose, const XrFovf& fov);
  // Records draws for every renderable geom in the scene, in scene order
  // (app-appended decor geoms land last: marker/gizmo over the robot).
  void Draw(VkCommandBuffer cmd, int eye, const mjvScene* scn);
  void Destroy();

 private:
  struct MeshRange {
    int32_t base_vertex = 0;
    uint32_t first_index = 0;
    uint32_t index_count = 0;
  };

  bool CreatePipeline();
  bool UploadGeometry(const mjModel* m);

  VkContext* vk_ = nullptr;
  VkDescriptorSetLayout dsl_ = VK_NULL_HANDLE;
  VkDescriptorPool pool_ = VK_NULL_HANDLE;
  VkPipelineLayout layout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;
  VkDescriptorSet sets_[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
  VkBuffer ubo_[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
  VkDeviceMemory ubo_mem_[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
  void* ubo_map_[2] = {nullptr, nullptr};
  VkBuffer vbuf_ = VK_NULL_HANDLE;
  VkDeviceMemory vmem_ = VK_NULL_HANDLE;
  VkBuffer ibuf_ = VK_NULL_HANDLE;
  VkDeviceMemory imem_ = VK_NULL_HANDLE;

  MeshRange plane_range_;
  MeshRange box_range_;
  std::vector<MeshRange> mesh_ranges_;  // per meshid
  float stage_from_world_[16] = {0};
};

#endif  // MUJOCOXR_APP_SCENE_RENDERER_H_
