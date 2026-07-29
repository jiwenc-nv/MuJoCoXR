// Vulkan context created through the OpenXR runtime
// (XR_KHR_vulkan_enable2): instance, device and queue come from
// xrCreateVulkanInstanceKHR/xrCreateVulkanDeviceKHR; render targets wrap the
// XR swapchain images. One render pass per eye, no multiview (do-not-build).

#ifndef MUJOCOXR_SRC_OPENXR_VK_CONTEXT_H_
#define MUJOCOXR_SRC_OPENXR_VK_CONTEXT_H_

#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

class XrShell;

struct EyeTarget {
  int32_t width = 0;
  int32_t height = 0;
  // App-owned depth, and ONLY when the runtime has no depth swapchain to give
  // us. Not dead code and not a legacy path: it is what every runtime without
  // XR_KHR_composition_layer_depth uses, Android included.
  VkImage depth_image = VK_NULL_HANDLE;
  VkDeviceMemory depth_memory = VK_NULL_HANDLE;

  std::vector<VkImageView> color_views;  // N, from the colour swapchain
  // M views: the depth swapchain's images, or exactly one app-owned image.
  std::vector<VkImageView> depth_views;
  // N*M, indexed [color_index*M + depth_index] — see FramebufferAt().
  //
  // THE PRODUCT IS NOT PARANOIA. Colour and depth are two independent
  // swapchains with their own image counts, and the runtime guarantees
  // NOTHING about the two acquired indices agreeing on any frame. A table
  // indexed by colour alone silently renders depth into the wrong image the
  // first time they diverge. With no depth layer M is 1 and this degenerates
  // to exactly the N framebuffers that were here before.
  std::vector<VkFramebuffer> framebuffers;
};

class VkContext {
 public:
  bool Create(XrShell* xr);              // instance/device/queue via OpenXR
  bool InitRenderTargets(XrShell* xr);   // render pass + per-eye framebuffers

  VkDevice device() const { return device_; }
  VkPhysicalDevice physical() const { return physical_; }
  VkInstance instance() const { return instance_; }
  uint32_t queue_family() const { return queue_family_; }
  VkRenderPass render_pass() const { return render_pass_; }

  // Transparent black for passthrough AR; opaque color otherwise.
  void SetClearColor(float r, float g, float b, float a) {
    clear_color_[0] = r;
    clear_color_[1] = g;
    clear_color_[2] = b;
    clear_color_[3] = a;
  }

  // Frame recording: one command buffer per frame, both eyes, one submit.
  VkCommandBuffer BeginFrameCommands();
  void BeginEyePass(VkCommandBuffer cmd, int eye, uint32_t color_index,
                    uint32_t depth_index);
  void EndEyePass(VkCommandBuffer cmd);
  // Submits and does NOT wait — the wait is the vkWaitForFences at the top
  // of the next BeginFrameCommands. It was called SubmitAndWait, which put
  // one method that says it waits and does not beside WaitIdle(), which
  // says it waits and does.
  bool Submit(VkCommandBuffer cmd);

  uint32_t FindMemoryType(uint32_t type_bits, VkMemoryPropertyFlags props);
  bool CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VkMemoryPropertyFlags props, VkBuffer* buffer,
                    VkDeviceMemory* memory);

  // Block until the GPU is idle. Exposed separately from Destroy() because
  // teardown has to interleave with the XR side: the runtime's swapchain
  // images live on this device, so xrDestroySwapchain must run BEFORE
  // vkDestroyDevice — and the last submitted frame must be finished before
  // either. Shells call WaitIdle(), then XrShell::Destroy(), then Destroy().
  void WaitIdle();

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

#endif  // MUJOCOXR_SRC_OPENXR_VK_CONTEXT_H_
