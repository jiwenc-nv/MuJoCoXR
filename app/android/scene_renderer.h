// mjvScene -> Vulkan, scene-specific by design:
// MESH + BOX only (planes are skipped — the AR background is passthrough),
// one pipeline, push constants, one hardcoded directional light, no
// textures/shadows/sorting. View/projection come exclusively from XrView
// pose + XrFovf; mjvGLCamera is bypassed. The vertex/index buffers and the
// appearance constants come from src/mesh_buffers.h so this renderer and its
// WebGL2 twin draw the same pixels; everything below is Vulkan-only.

#ifndef MUJOCOXR_APP_ANDROID_SCENE_RENDERER_H_
#define MUJOCOXR_APP_ANDROID_SCENE_RENDERER_H_

#include <vulkan/vulkan.h>

#include <openxr/openxr.h>

#include <mujoco/mujoco.h>

#include <cstdint>
#include <vector>

#include "mesh_buffers.h"

class VkContext;

class SceneRenderer {
 public:
  bool Create(VkContext* vk, const mjModel* m);
  // Computes P(fov) * V(pose)^-1 * xr_from_mj and writes the eye's UBO.
  void SetEye(int eye, const XrPosef& view_pose, const XrFovf& fov);
  // Records draws for every renderable geom in the scene, in scene order
  // (app-appended decor geoms land last: marker/gizmo over the robot).
  void Draw(VkCommandBuffer cmd, int eye, const mjvScene* scn);
  void Destroy();

 private:
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

  MeshRange box_range_;
  std::vector<MeshRange> mesh_ranges_;  // per meshid
  float xr_from_mj_[16] = {0};
};

#endif  // MUJOCOXR_APP_ANDROID_SCENE_RENDERER_H_
