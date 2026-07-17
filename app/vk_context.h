// Vulkan context created through the OpenXR runtime
// (XR_KHR_vulkan_enable2): instance, device and queue come from
// xrCreateVulkanInstanceKHR/xrCreateVulkanDeviceKHR; render targets wrap the
// XR swapchain images. One render pass per eye, no multiview (do-not-build).

#ifndef MUJOCOXR_APP_VK_CONTEXT_H_
#define MUJOCOXR_APP_VK_CONTEXT_H_

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

class XrShell;

struct EyeTarget {
  int32_t width = 0;
  int32_t height = 0;
  VkImage depth_image = VK_NULL_HANDLE;
  VkDeviceMemory depth_memory = VK_NULL_HANDLE;
  VkImageView depth_view = VK_NULL_HANDLE;
  std::vector<VkImageView> color_views;    // one per swapchain image
  std::vector<VkFramebuffer> framebuffers; // one per swapchain image
};

class VkContext {
 public:
  bool Create(XrShell* xr);              // instance/device/queue via OpenXR
  bool InitRenderTargets(XrShell* xr);   // render pass + per-eye framebuffers

  VkDevice device() const { return device_; }
  VkPhysicalDevice physical() const { return physical_; }
  VkInstance instance() const { return instance_; }
  uint32_t queue_family() const { return queue_family_; }
  VkQueue queue() const { return queue_; }
  VkRenderPass render_pass() const { return render_pass_; }
  VkFormat color_format() const { return color_format_; }
  const EyeTarget& eye(int i) const { return eyes_[i]; }

  // Transparent black for passthrough AR; opaque color otherwise.
  void SetClearColor(float r, float g, float b, float a) {
    clear_color_[0] = r;
    clear_color_[1] = g;
    clear_color_[2] = b;
    clear_color_[3] = a;
  }

  // Frame recording: one command buffer per frame, both eyes, one submit.
  VkCommandBuffer BeginFrameCommands();
  void BeginEyePass(VkCommandBuffer cmd, int eye, uint32_t image_index);
  void EndEyePass(VkCommandBuffer cmd);
  bool SubmitAndWait(VkCommandBuffer cmd);

  uint32_t FindMemoryType(uint32_t type_bits, VkMemoryPropertyFlags props);
  bool CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VkMemoryPropertyFlags props, VkBuffer* buffer,
                    VkDeviceMemory* memory);

  void Destroy();

 private:
  VkInstance instance_ = VK_NULL_HANDLE;
  VkPhysicalDevice physical_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  uint32_t queue_family_ = 0;
  VkQueue queue_ = VK_NULL_HANDLE;
  VkCommandPool command_pool_ = VK_NULL_HANDLE;
  VkCommandBuffer command_buffer_ = VK_NULL_HANDLE;
  VkFence frame_fence_ = VK_NULL_HANDLE;
  VkRenderPass render_pass_ = VK_NULL_HANDLE;
  VkFormat color_format_ = VK_FORMAT_UNDEFINED;
  VkFormat depth_format_ = VK_FORMAT_UNDEFINED;
  float clear_color_[4] = {0.10f, 0.12f, 0.16f, 1.0f};
  std::vector<EyeTarget> eyes_;
};

#endif  // MUJOCOXR_APP_VK_CONTEXT_H_
