// Raw OpenXR shell for the MuJoCoXR NativeActivity app
// (raw OpenXR, no game engine): Khronos loader,
// XR_KHR_android_create_instance, XR_KHR_vulkan_enable2 handshake,
// LOCAL_FLOOR reference space (fallback STAGE -> LOCAL), one action set on
// the right Touch controller (grip pose, trigger, squeeze, A; B unbound).

#ifndef MUJOCOXR_APP_XR_SHELL_H_
#define MUJOCOXR_APP_XR_SHELL_H_

#include <jni.h>
#include <vulkan/vulkan.h>

#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdint>
#include <vector>

struct android_app;

// Controller state sampled once per frame via xrSyncActions.
struct XrInputState {
  bool grip_valid = false;
  XrPosef grip = {{0, 0, 0, 1}, {0, 0, 0}};  // in the app reference space
  float trigger = 0;
  float squeeze = 0;
  bool a_click = false;    // rising edge this frame
  bool recenter = false;   // XrEventDataReferenceSpaceChangePending this frame
};

struct SwapchainInfo {
  XrSwapchain handle = XR_NULL_HANDLE;
  int32_t width = 0;
  int32_t height = 0;
  std::vector<XrSwapchainImageVulkanKHR> images;
};

class XrShell {
 public:
  // Loader init + instance + system. Requires the activity for
  // XR_KHR_android_create_instance.
  bool CreateInstance(android_app* app);

  // XR_KHR_vulkan_enable2 handshake, in call order.
  bool GetVulkanRequirements(XrGraphicsRequirementsVulkanKHR* reqs);
  bool CreateVulkanInstance(const VkInstanceCreateInfo* vk_info,
                            VkInstance* vk_instance);
  bool GetVulkanPhysicalDevice(VkInstance vk_instance,
                               VkPhysicalDevice* vk_physical);
  bool CreateVulkanDevice(VkPhysicalDevice vk_physical,
                          const VkDeviceCreateInfo* vk_info, VkDevice* vk_dev);

  bool CreateSession(VkInstance vk_instance, VkPhysicalDevice vk_physical,
                     VkDevice vk_device, uint32_t queue_family);
  // Picks the first supported format from `preferred`; swapchain per view.
  bool CreateSwapchains(const std::vector<int64_t>& preferred);
  bool CreateActions();

  // Session-state machine; sets *exit_render_loop / *request_restart.
  void PollEvents(bool* exit_render_loop, XrInputState* input);

  bool session_running() const { return session_running_; }
  XrInstance instance() const { return instance_; }
  XrSession session() const { return session_; }
  XrSpace app_space() const { return app_space_; }
  int64_t swapchain_format() const { return swapchain_format_; }
  const std::vector<SwapchainInfo>& swapchains() const { return swapchains_; }

  // Frame flow.
  bool WaitBeginFrame(XrFrameState* frame_state);
  bool LocateViews(XrTime time, std::vector<XrView>* views);
  bool AcquireSwapchainImage(int view, uint32_t* image_index);
  bool ReleaseSwapchainImage(int view);
  bool EndFrame(const XrFrameState& frame_state,
                const std::vector<XrCompositionLayerProjectionView>& proj_views);
  void SyncInput(XrTime time, XrInputState* input);

  void Destroy();

 private:
  bool CreateAppSpace();
  void HandleSessionStateChange(const XrEventDataSessionStateChanged& ev,
                                bool* exit_render_loop);

  XrInstance instance_ = XR_NULL_HANDLE;
  XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
  XrSession session_ = XR_NULL_HANDLE;
  XrSpace app_space_ = XR_NULL_HANDLE;
  bool local_floor_available_ = false;
  bool session_running_ = false;
  XrSessionState session_state_ = XR_SESSION_STATE_UNKNOWN;
  int64_t swapchain_format_ = 0;
  std::vector<XrViewConfigurationView> config_views_;
  std::vector<SwapchainInfo> swapchains_;

  void LogInteractionProfiles();

  XrActionSet action_set_ = XR_NULL_HANDLE;
  XrAction grip_action_ = XR_NULL_HANDLE;
  XrAction trigger_action_ = XR_NULL_HANDLE;
  XrAction squeeze_action_ = XR_NULL_HANDLE;
  XrAction a_action_ = XR_NULL_HANDLE;
  // Both hands bound; per frame the first hand with a valid grip pose wins
  // (index 0 = right, 1 = left).
  XrPath hand_paths_[2] = {XR_NULL_PATH, XR_NULL_PATH};
  XrSpace grip_spaces_[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
  int64_t sync_count_ = 0;
};

#endif  // MUJOCOXR_APP_XR_SHELL_H_
