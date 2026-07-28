// Raw OpenXR shell for the MuJoCoXR NativeActivity app
// (raw OpenXR, no game engine): Khronos loader,
// XR_KHR_android_create_instance, XR_KHR_vulkan_enable2 handshake,
// LOCAL_FLOOR reference space (fallback STAGE -> LOCAL), one action set bound
// on BOTH hands across every supported interaction profile (grip pose,
// trigger, squeeze, A/X, B/Y).

#ifndef MUJOCOXR_APP_ANDROID_XR_SHELL_H_
#define MUJOCOXR_APP_ANDROID_XR_SHELL_H_

#include <jni.h>
#include <vulkan/vulkan.h>

#define XR_USE_PLATFORM_ANDROID
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdint>
#include <vector>

#include "frames.h"

struct android_app;

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

  // Session-state machine; sets *exit_render_loop. Also latches
  // input->recenter_edge from XrEventDataReferenceSpaceChangePending, which
  // is why it takes the input struct at all — that field is the one piece of
  // InputState that does not come from xrSyncActions.
  void PollEvents(bool* exit_render_loop, InputState* input);

  // B/Y as a RAW LEVEL, updated by SyncInput. Deliberately NOT a field of
  // InputState: InputState is the shell->core struct and the core does
  // nothing with B — switching scenes destroys and rebuilds SimScene, so it
  // cannot be a method on it. The WebXR shell keeps its B binding local for
  // the same reason (app/web/shell.js reads buttons[5] and ends the session).
  // The caller owns the edge; see the b_prev latch in android_main.
  bool b_down() const { return b_down_; }

  bool session_running() const { return session_running_; }
  XrInstance instance() const { return instance_; }
  XrSession session() const { return session_; }
  XrSpace app_space() const { return app_space_; }
  int64_t swapchain_format() const { return swapchain_format_; }
  const std::vector<SwapchainInfo>& swapchains() const { return swapchains_; }
  // ALPHA_BLEND (passthrough AR) when the runtime offers it, else OPAQUE.
  bool passthrough() const {
    return blend_mode_ == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
  }

  // Frame flow.
  bool WaitBeginFrame(XrFrameState* frame_state);
  bool LocateViews(XrTime time, std::vector<XrView>* views);
  bool AcquireSwapchainImage(int view, uint32_t* image_index);
  bool ReleaseSwapchainImage(int view);
  bool EndFrame(const XrFrameState& frame_state,
                const std::vector<XrCompositionLayerProjectionView>& proj_views);
  // Fills every InputState field except recenter_edge (see PollEvents).
  void SyncInput(XrTime time, InputState* input);

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
  bool bd_controllers_available_ = false;  // XR_BD_controller_interaction
  XrEnvironmentBlendMode blend_mode_ = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
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
  XrAction b_action_ = XR_NULL_HANDLE;
  bool b_down_ = false;
  // Both hands bound; per frame the first hand with a valid grip pose wins
  // (index 0 = right, 1 = left).
  XrPath hand_paths_[2] = {XR_NULL_PATH, XR_NULL_PATH};
  XrSpace grip_spaces_[2] = {XR_NULL_HANDLE, XR_NULL_HANDLE};
  int64_t sync_count_ = 0;
};

#endif  // MUJOCOXR_APP_ANDROID_XR_SHELL_H_
