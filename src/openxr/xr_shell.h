// Raw OpenXR shell (no game engine): Khronos loader, XR_KHR_vulkan_enable2
// handshake, LOCAL_FLOOR reference space (fallback STAGE -> LOCAL), one action
// set bound on BOTH hands across every supported interaction profile (grip
// pose, trigger, squeeze, A/X, B/Y).
//
// src/openxr/ IS A TIER, AND ITS RULE IS THE INVERSE OF THE ONE ABOVE IT.
// src/ holds two tiers: its TOP LEVEL is the portable core and may not name a
// graphics or runtime API, while this directory exists to name exactly two of
// them — Vulkan and OpenXR — and in exchange may not name a PLATFORM: no JNI,
// no NDK or Emscripten headers, no platform-selection macro for the OpenXR
// headers, no compiler platform predefine. THE EXACT ALTERNATION IS IN
// CMakeLists.txt, not restated here — a configure-time scan is the rule, and
// this file is inside what it scans, so spelling the forbidden tokens in this
// comment would fail the build that the comment describes. Both scans sit in
// the CMake common prefix, so they run in every configuration, including the
// plain host build where this tier is not compiled at all — and between them
// they cover every first-party file under src/ exactly once, which is why the
// top-level scan globs `src/*` and NOT `src/**`.
//
// That is why CreateInstance takes its platform chain as an opaque pointer:
// Android's loader init and its create-info struct live in
// app/android/xr_platform.cc, the one file in the tree that selects the
// Android OpenXR platform. A desktop mirror window would break the same rule
// from the other side — it belongs in app/linux/ with its own surface and
// present loop, NEVER inside src/openxr/vk_context.cc, which is deliberately
// surfaceless.

#ifndef MUJOCOXR_SRC_OPENXR_XR_SHELL_H_
#define MUJOCOXR_SRC_OPENXR_XR_SHELL_H_

#include <csignal>

#include <vulkan/vulkan.h>

#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <cstdint>
#include <vector>

#include "frames.h"

struct SwapchainInfo {
  XrSwapchain handle = XR_NULL_HANDLE;
  int32_t width = 0;
  int32_t height = 0;
  std::vector<XrSwapchainImageVulkanKHR> images;
};

class XrShell {
 public:
  // Instance + system.
  //
  // `platform_next` is a platform-specific struct chained into
  // XrInstanceCreateInfo::next and `platform_ext` the instance extension that
  // struct requires — on Android, XrInstanceCreateInfoAndroidKHR and
  // XR_KHR_android_create_instance, both produced by
  // app/android/xr_platform.h. Both are nullptr on a platform that needs
  // neither, which is every platform except Android. Opaque rather than typed
  // because naming the type here would make this tier Android-aware; see the
  // tier rule at the top of this file.
  //
  // `api_version` is a VALUE, not a build-time branch, because it changes
  // which reference spaces exist and which interaction profiles are legal —
  // things a reader has to be able to trace from the call site. 1.1 on Linux,
  // 1.0 on Android; a runtime that rejects 1.1 is retried at 1.0 with a
  // warning; the granted version is what the runtime log line reports.
  //
  // `system_wait_s` bounds a retry of xrGetSystem, and ONLY for
  // XR_ERROR_FORM_FACTOR_UNAVAILABLE — a streaming runtime answers that until
  // a headset attaches. <= 0 makes exactly one attempt. Returning true with no
  // system is a legitimate outcome; the caller reads have_system().
  bool CreateInstance(const void* platform_next, const char* platform_ext,
                      XrVersion api_version, int system_wait_s);

  // False when the instance came up but no system is available. Split from
  // CreateInstance's return so --probe can report it without judging it,
  // while every other caller treats it as fatal.
  bool have_system() const { return system_id_ != XR_NULL_SYSTEM_ID; }
  // The last xrCreateInstance result. Exists so a shell can say what a
  // failure MEANS on its own platform without this tier having to know which
  // runtimes exist — see the RUNTIME_UNAVAILABLE branch in app/linux/main.cc.
  XrResult instance_result() const { return instance_result_; }
  // The last xrGetSystem result, so a caller reports what it SAW rather than
  // naming a literal for a branch it has never executed.
  XrResult system_result() const { return system_result_; }
  // XrSystemProperties::systemName, or "" with no system. Worth printing
  // because a streaming runtime answers xrGetSystem from a virtual device
  // long before a headset is worn, so "found a system" is not "ready".
  const char* system_name() const { return system_name_; }

  // A flag a shell's signal handler sets, polled by the only long wait in this
  // class (the xrGetSystem retry). It exists because a shell MUST install its
  // handlers before CreateInstance — a Ctrl-C during a 120 s wait would
  // otherwise kill the process with a live XrInstance against a host-singleton
  // runtime — and doing that without this would instead make Ctrl-C do NOTHING
  // for those 120 s. nullptr, the default and what Android passes, means the
  // wait is uninterruptible, which is right there: Android passes 0 and never
  // waits.
  // The POINTEE MUST OUTLIVE THIS OBJECT. In practice it is a file-scope
  // sig_atomic_t in the shell's translation unit, which outlives everything;
  // nothing here copies or owns it.
  void set_abort_flag(const volatile sig_atomic_t* flag) { abort_flag_ = flag; }

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
  // Instance-scope: action set, actions, and one binding suggestion per
  // supported interaction profile. Needs no session and therefore no headset.
  bool CreateActions();
  // Session-scope: attach the set and create the two grip action spaces.
  // Must follow CreateActions() and a live session.
  bool AttachActions();
  // How many interaction profiles ACCEPTED their suggestions — the runtime's
  // side of the transaction, which is why it is not called `suggested`.
  // Reported by --probe, the only check of that work that runs without
  // hardware. The count alone cannot distinguish "rejected" from "never
  // offered", so CreateActions() logs one line per profile with all three
  // outcomes and this is only the summary.
  int profiles_accepted() const { return profiles_accepted_; }

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
  // The caller owns the edge; see the b_prev latch in app/android/main.cc.
  // app/linux has no B binding at all — it selects a scene with --scene.
  bool b_down() const { return b_down_; }

  bool session_running() const { return session_running_; }

  // Consume-once: true exactly once per real session end (STOPPING / EXITING /
  // LOSS_PENDING from a running session), so a shell can hand the transition
  // to SimScene::EndSession. On XrShell rather than re-derived per shell
  // because BOTH failure modes are silent — miss it and an HMD sleep/wake
  // mid-clutch resumes still engaged over a stale anchor; fire it spuriously
  // and you disengage a clutch nobody released. Neither prints anything.
  //
  // The b_prev edge deliberately does NOT move here; see b_down() above for
  // why that one belongs to the caller.
  bool TakeSessionEndEdge() {
    const bool e = session_end_edge_;
    session_end_edge_ = false;
    return e;
  }

  // Absolute display time in seconds, from an epoch latched on the first call.
  //
  // THE SUBTRACTION HAPPENS IN INT64, BEFORE THE CONVERSION, and that is the
  // whole reason this is a method rather than a line in each shell:
  // predictedDisplayTime is ~1e18 ns, and a*1e-9 - b*1e-9 is not the same
  // double as (a-b)*1e-9. This makes the dt the core derives accurate, not
  // exact — (t3-e)*1e-9 - (t2-e)*1e-9 is still a difference of two rounded
  // doubles, ~1e-13 s once a session has been up for hours. That is 7 orders
  // below one 72 Hz frame and does not accumulate, because SimScene's
  // accumulator keeps its sub-timestep residual.
  double display_time_s(XrTime t) {
    if (time_epoch_ == 0) {
      time_epoch_ = t;
    }
    return static_cast<double>(t - time_epoch_)*1e-9;
  }
  int64_t swapchain_format() const { return swapchain_format_; }
  const std::vector<SwapchainInfo>& swapchains() const { return swapchains_; }

  // DEPTH SUBMISSION, and the reason it is worth the second swapchain: a
  // streaming runtime reprojects the last rendered frame to the head pose it
  // has NOW, and without a depth buffer it can only rotate a flat image —
  // which is why the symptom of not submitting depth is world-swim under head
  // rotation that gets worse the faster you turn. Gated on the extension, NOT
  // on the platform: Quest advertises it and reprojects too, so a Linux-only
  // gate would be an artificial divergence inside this tier.
  //
  // False when the runtime does not advertise XR_KHR_composition_layer_depth.
  // Everything below then behaves as it did before this existed, and
  // VkContext falls back to an app-owned depth image — which is not dead code,
  // it is the path any runtime without the extension takes, Android included.
  bool depth_layer() const { return !depth_swapchains_.empty(); }
  const std::vector<SwapchainInfo>& depth_swapchains() const {
    return depth_swapchains_;
  }
  int64_t depth_swapchain_format() const { return depth_swapchain_format_; }
  // ALPHA_BLEND (passthrough AR) when the runtime offers it, else OPAQUE.
  bool passthrough() const {
    return blend_mode_ == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
  }

  // Frame flow.
  bool WaitBeginFrame(XrFrameState* frame_state);
  bool LocateViews(XrTime time, std::vector<XrView>* views);
  bool AcquireSwapchainImage(int view, uint32_t* image_index);
  bool ReleaseSwapchainImage(int view);
  // The depth swapchain cycles INDEPENDENTLY of the colour one — the runtime
  // guarantees no relationship between the two acquired indices — so this is
  // a separate acquire whose result must be carried separately. Assuming
  // depth_index == color_index is the bug this API shape exists to prevent.
  // No-ops returning true when depth_layer() is false.
  bool AcquireDepthImage(int view, uint32_t* image_index);
  bool ReleaseDepthImage(int view);

  // Chains one XrCompositionLayerDepthInfoKHR per view into proj_views[i].next.
  //
  // THE LIFETIME IS THE WHOLE POINT OF THIS SIGNATURE. The chain is by
  // POINTER and the runtime dereferences it inside xrEndFrame, so the depth
  // infos must outlive that call. Filling a local in the per-view loop
  // dangles and usually appears to work. Taking the vector by pointer forces
  // the caller to own it in the same scope as proj_views, and the sizing
  // happens here, once, before any address is taken.
  // A no-op when depth_layer() is false.
  // Takes no image index on purpose: XrSwapchainSubImage has none, because
  // the runtime composites the most recently RELEASED image of the swapchain.
  // The acquired depth index selects a FRAMEBUFFER and is a Vulkan-side
  // concern only.
  void ChainDepthInfo(std::vector<XrCompositionLayerProjectionView>* proj_views,
                      std::vector<XrCompositionLayerDepthInfoKHR>* depth_infos);
  bool EndFrame(const XrFrameState& frame_state,
                const std::vector<XrCompositionLayerProjectionView>& proj_views);
  // Fills every InputState field except recenter_edge (see PollEvents).
  void SyncInput(XrTime time, InputState* input);

  void Destroy();

 private:
  bool CreateAppSpace();
  static bool AcquireFrom(const SwapchainInfo& sc, uint32_t* image_index);
  static bool ReleaseFrom(const SwapchainInfo& sc);
  bool WaitForSystem(int system_wait_s);
  void HandleSessionStateChange(const XrEventDataSessionStateChanged& ev,
                                bool* exit_render_loop);

  XrInstance instance_ = XR_NULL_HANDLE;
  XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
  XrSession session_ = XR_NULL_HANDLE;
  XrSpace app_space_ = XR_NULL_HANDLE;
  XrVersion api_version_ = 0;
  XrResult instance_result_ = XR_ERROR_RUNTIME_FAILURE;
  XrResult system_result_ = XR_ERROR_FORM_FACTOR_UNAVAILABLE;
  char system_name_[XR_MAX_SYSTEM_NAME_SIZE] = {0};
  const volatile sig_atomic_t* abort_flag_ = nullptr;
  // "the XR_EXT_local_floor extension string was advertised", which after the
  // 1.1 work is NOT the same question as "is a local-floor space available" —
  // at 1.1 it is core and the extension is absent. Named for what it holds.
  bool local_floor_ext_ = false;
  bool bd_controllers_available_ = false;  // XR_BD_controller_interaction
  bool generic_controller_available_ = false;  // XR_KHR_generic_controller
  int profiles_accepted_ = 0;
  XrEnvironmentBlendMode blend_mode_ = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  bool session_running_ = false;
  bool session_end_edge_ = false;
  XrTime time_epoch_ = 0;
  XrSessionState session_state_ = XR_SESSION_STATE_UNKNOWN;
  int64_t swapchain_format_ = 0;
  int64_t depth_swapchain_format_ = 0;
  bool depth_layer_ext_ = false;  // XR_KHR_composition_layer_depth
  std::vector<XrViewConfigurationView> config_views_;
  std::vector<SwapchainInfo> swapchains_;
  std::vector<SwapchainInfo> depth_swapchains_;

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
  // Rising-edge latch + lifetime total for the stale-pose window.
  int not_tracked_run_ = 0;
  int64_t not_tracked_frames_ = 0;
};

#endif  // MUJOCOXR_SRC_OPENXR_XR_SHELL_H_
