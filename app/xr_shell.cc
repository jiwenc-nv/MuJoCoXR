#include "xr_shell.h"

#include <android_native_app_glue.h>

#include <cstring>

#include "mxr_log.h"

namespace {

bool XrOk(XrResult r, const char* what) {
  if (XR_SUCCEEDED(r)) {
    return true;
  }
  LOGE("%s failed: %d", what, r);
  return false;
}

// Extension functions are not exported by the loader; fetch via GIPA.
template <typename Fn>
bool LoadXrFn(XrInstance instance, const char* name, Fn* fn) {
  XrResult r = xrGetInstanceProcAddr(instance, name,
                                     reinterpret_cast<PFN_xrVoidFunction*>(fn));
  return XrOk(r, name);
}

}  // namespace

bool XrShell::CreateInstance(android_app* app) {
  // Android mandates explicit loader initialization before any other call.
  PFN_xrInitializeLoaderKHR init_loader = nullptr;
  if (!LoadXrFn(XR_NULL_HANDLE, "xrInitializeLoaderKHR", &init_loader)) {
    return false;
  }
  XrLoaderInitInfoAndroidKHR loader_info{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
  loader_info.applicationVM = app->activity->vm;
  loader_info.applicationContext = app->activity->clazz;
  if (!XrOk(init_loader(reinterpret_cast<XrLoaderInitInfoBaseHeaderKHR*>(
                &loader_info)),
            "xrInitializeLoaderKHR")) {
    return false;
  }

  // XR_EXT_local_floor is required by the design (fallback chain at space
  // creation); probe for it so a missing runtime extension degrades cleanly.
  uint32_t ext_count = 0;
  xrEnumerateInstanceExtensionProperties(nullptr, 0, &ext_count, nullptr);
  std::vector<XrExtensionProperties> exts(ext_count,
                                          {XR_TYPE_EXTENSION_PROPERTIES});
  xrEnumerateInstanceExtensionProperties(nullptr, ext_count, &ext_count,
                                         exts.data());
  for (const auto& e : exts) {
    if (!strcmp(e.extensionName, XR_EXT_LOCAL_FLOOR_EXTENSION_NAME)) {
      local_floor_available_ = true;
    }
    // Pico controllers use ByteDance's own interaction profiles.
    if (!strcmp(e.extensionName, "XR_BD_controller_interaction")) {
      bd_controllers_available_ = true;
    }
  }

  std::vector<const char*> enabled = {
      XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME,
      XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME,
  };
  if (local_floor_available_) {
    enabled.push_back(XR_EXT_LOCAL_FLOOR_EXTENSION_NAME);
  }
  if (bd_controllers_available_) {
    enabled.push_back("XR_BD_controller_interaction");
  }

  XrInstanceCreateInfoAndroidKHR android_info{
      XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
  android_info.applicationVM = app->activity->vm;
  android_info.applicationActivity = app->activity->clazz;

  XrInstanceCreateInfo info{XR_TYPE_INSTANCE_CREATE_INFO};
  info.next = &android_info;
  strncpy(info.applicationInfo.applicationName, "MuJoCoXR",
          XR_MAX_APPLICATION_NAME_SIZE - 1);
  info.applicationInfo.applicationVersion = 1;
  strncpy(info.applicationInfo.engineName, "none",
          XR_MAX_ENGINE_NAME_SIZE - 1);
  info.applicationInfo.apiVersion = XR_API_VERSION_1_0;
  info.enabledExtensionCount = static_cast<uint32_t>(enabled.size());
  info.enabledExtensionNames = enabled.data();
  if (!XrOk(xrCreateInstance(&info, &instance_), "xrCreateInstance")) {
    return false;
  }

  XrSystemGetInfo sys{XR_TYPE_SYSTEM_GET_INFO};
  sys.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
  if (!XrOk(xrGetSystem(instance_, &sys, &system_id_), "xrGetSystem")) {
    return false;
  }

  XrInstanceProperties props{XR_TYPE_INSTANCE_PROPERTIES};
  xrGetInstanceProperties(instance_, &props);
  LOGI("OpenXR runtime: %s %u.%u.%u (local_floor=%d)", props.runtimeName,
       XR_VERSION_MAJOR(props.runtimeVersion),
       XR_VERSION_MINOR(props.runtimeVersion),
       XR_VERSION_PATCH(props.runtimeVersion), local_floor_available_);
  return true;
}

bool XrShell::GetVulkanRequirements(XrGraphicsRequirementsVulkanKHR* reqs) {
  PFN_xrGetVulkanGraphicsRequirements2KHR fn = nullptr;
  if (!LoadXrFn(instance_, "xrGetVulkanGraphicsRequirements2KHR", &fn)) {
    return false;
  }
  reqs->type = XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR;
  reqs->next = nullptr;
  return XrOk(fn(instance_, system_id_, reqs),
              "xrGetVulkanGraphicsRequirements2KHR");
}

bool XrShell::CreateVulkanInstance(const VkInstanceCreateInfo* vk_info,
                                   VkInstance* vk_instance) {
  PFN_xrCreateVulkanInstanceKHR fn = nullptr;
  if (!LoadXrFn(instance_, "xrCreateVulkanInstanceKHR", &fn)) {
    return false;
  }
  XrVulkanInstanceCreateInfoKHR info{XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR};
  info.systemId = system_id_;
  info.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
  info.vulkanCreateInfo = vk_info;
  VkResult vk_result = VK_SUCCESS;
  if (!XrOk(fn(instance_, &info, vk_instance, &vk_result),
            "xrCreateVulkanInstanceKHR") ||
      vk_result != VK_SUCCESS) {
    LOGE("vkCreateInstance (via runtime): %d", vk_result);
    return false;
  }
  return true;
}

bool XrShell::GetVulkanPhysicalDevice(VkInstance vk_instance,
                                      VkPhysicalDevice* vk_physical) {
  PFN_xrGetVulkanGraphicsDevice2KHR fn = nullptr;
  if (!LoadXrFn(instance_, "xrGetVulkanGraphicsDevice2KHR", &fn)) {
    return false;
  }
  XrVulkanGraphicsDeviceGetInfoKHR info{
      XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR};
  info.systemId = system_id_;
  info.vulkanInstance = vk_instance;
  return XrOk(fn(instance_, &info, vk_physical),
              "xrGetVulkanGraphicsDevice2KHR");
}

bool XrShell::CreateVulkanDevice(VkPhysicalDevice vk_physical,
                                 const VkDeviceCreateInfo* vk_info,
                                 VkDevice* vk_dev) {
  PFN_xrCreateVulkanDeviceKHR fn = nullptr;
  if (!LoadXrFn(instance_, "xrCreateVulkanDeviceKHR", &fn)) {
    return false;
  }
  XrVulkanDeviceCreateInfoKHR info{XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR};
  info.systemId = system_id_;
  info.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
  info.vulkanPhysicalDevice = vk_physical;
  info.vulkanCreateInfo = vk_info;
  VkResult vk_result = VK_SUCCESS;
  if (!XrOk(fn(instance_, &info, vk_dev, &vk_result),
            "xrCreateVulkanDeviceKHR") ||
      vk_result != VK_SUCCESS) {
    LOGE("vkCreateDevice (via runtime): %d", vk_result);
    return false;
  }
  return true;
}

bool XrShell::CreateSession(VkInstance vk_instance,
                            VkPhysicalDevice vk_physical, VkDevice vk_device,
                            uint32_t queue_family) {
  XrGraphicsBindingVulkanKHR binding{XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR};
  binding.instance = vk_instance;
  binding.physicalDevice = vk_physical;
  binding.device = vk_device;
  binding.queueFamilyIndex = queue_family;
  binding.queueIndex = 0;

  XrSessionCreateInfo info{XR_TYPE_SESSION_CREATE_INFO};
  info.next = &binding;
  info.systemId = system_id_;
  if (!XrOk(xrCreateSession(instance_, &info, &session_), "xrCreateSession")) {
    return false;
  }
  return CreateAppSpace();
}

bool XrShell::CreateAppSpace() {
  // LOCAL_FLOOR -> STAGE -> LOCAL, most concrete first.
  XrReferenceSpaceType order[] = {XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR_EXT,
                                  XR_REFERENCE_SPACE_TYPE_STAGE,
                                  XR_REFERENCE_SPACE_TYPE_LOCAL};
  const char* names[] = {"LOCAL_FLOOR", "STAGE", "LOCAL"};
  for (int i = local_floor_available_ ? 0 : 1; i < 3; ++i) {
    XrReferenceSpaceCreateInfo info{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    info.referenceSpaceType = order[i];
    info.poseInReferenceSpace = {{0, 0, 0, 1}, {0, 0, 0}};
    if (XR_SUCCEEDED(xrCreateReferenceSpace(session_, &info, &app_space_))) {
      LOGI("reference space: %s", names[i]);
      return true;
    }
  }
  LOGE("no usable reference space");
  return false;
}

bool XrShell::CreateSwapchains(const std::vector<int64_t>& preferred) {
  uint32_t count = 0;
  xrEnumerateViewConfigurationViews(instance_, system_id_,
                                    XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                    0, &count, nullptr);
  config_views_.assign(count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
  if (!XrOk(xrEnumerateViewConfigurationViews(
                instance_, system_id_,
                XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, count, &count,
                config_views_.data()),
            "xrEnumerateViewConfigurationViews")) {
    return false;
  }

  uint32_t nfmt = 0;
  xrEnumerateSwapchainFormats(session_, 0, &nfmt, nullptr);
  std::vector<int64_t> formats(nfmt);
  xrEnumerateSwapchainFormats(session_, nfmt, &nfmt, formats.data());
  swapchain_format_ = 0;
  for (int64_t want : preferred) {
    for (int64_t have : formats) {
      if (want == have) {
        swapchain_format_ = want;
        break;
      }
    }
    if (swapchain_format_) {
      break;
    }
  }
  if (!swapchain_format_) {
    LOGE("no preferred swapchain format available");
    return false;
  }

  swapchains_.resize(config_views_.size());
  for (size_t i = 0; i < config_views_.size(); ++i) {
    const auto& cv = config_views_[i];
    XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT |
                      XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    info.format = swapchain_format_;
    info.sampleCount = 1;
    info.width = cv.recommendedImageRectWidth;
    info.height = cv.recommendedImageRectHeight;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    auto& sc = swapchains_[i];
    if (!XrOk(xrCreateSwapchain(session_, &info, &sc.handle),
              "xrCreateSwapchain")) {
      return false;
    }
    sc.width = info.width;
    sc.height = info.height;
    uint32_t nimg = 0;
    xrEnumerateSwapchainImages(sc.handle, 0, &nimg, nullptr);
    sc.images.assign(nimg, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN_KHR});
    if (!XrOk(xrEnumerateSwapchainImages(
                  sc.handle, nimg, &nimg,
                  reinterpret_cast<XrSwapchainImageBaseHeader*>(
                      sc.images.data())),
              "xrEnumerateSwapchainImages")) {
      return false;
    }
    LOGI("swapchain %zu: %dx%d, %u images, format %lld", i, sc.width,
         sc.height, nimg, static_cast<long long>(swapchain_format_));
  }
  return true;
}

bool XrShell::CreateActions() {
  XrActionSetCreateInfo set_info{XR_TYPE_ACTION_SET_CREATE_INFO};
  strcpy(set_info.actionSetName, "teleop");
  strcpy(set_info.localizedActionSetName, "Teleop");
  if (!XrOk(xrCreateActionSet(instance_, &set_info, &action_set_),
            "xrCreateActionSet")) {
    return false;
  }
  xrStringToPath(instance_, "/user/hand/right", &hand_paths_[0]);
  xrStringToPath(instance_, "/user/hand/left", &hand_paths_[1]);

  struct {
    const char* name;
    XrActionType type;
    XrAction* action;
  } actions[] = {
      {"grip_pose", XR_ACTION_TYPE_POSE_INPUT, &grip_action_},
      {"trigger", XR_ACTION_TYPE_FLOAT_INPUT, &trigger_action_},
      {"squeeze", XR_ACTION_TYPE_FLOAT_INPUT, &squeeze_action_},
      {"reset_click", XR_ACTION_TYPE_BOOLEAN_INPUT, &a_action_},
  };
  for (auto& a : actions) {
    XrActionCreateInfo info{XR_TYPE_ACTION_CREATE_INFO};
    strcpy(info.actionName, a.name);
    strcpy(info.localizedActionName, a.name);
    info.actionType = a.type;
    info.countSubactionPaths = 2;
    info.subactionPaths = hand_paths_;
    if (!XrOk(xrCreateAction(action_set_, &info, a.action),
              "xrCreateAction")) {
      return false;
    }
  }

  // Per-profile suggestions, one call each. Same semantics everywhere:
  // squeeze = clutch, trigger = analog gripper, A/X = reset; B stays
  // unbound (squeeze re-engage already re-anchors by construction).
  auto path = [&](const char* p) {
    XrPath out;
    xrStringToPath(instance_, p, &out);
    return out;
  };
  auto suggest = [&](const char* profile_path,
                     std::vector<XrActionSuggestedBinding> b) {
    XrInteractionProfileSuggestedBinding s{
        XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    s.interactionProfile = path(profile_path);
    s.suggestedBindings = b.data();
    s.countSuggestedBindings = static_cast<uint32_t>(b.size());
    XrResult r = xrSuggestInteractionProfileBindings(instance_, &s);
    if (XR_FAILED(r)) {
      LOGW("suggest bindings %s failed: %d", profile_path, r);
      return false;
    }
    LOGI("suggested bindings: %s", profile_path);
    return true;
  };

  if (!suggest("/interaction_profiles/oculus/touch_controller",
               {{grip_action_, path("/user/hand/right/input/grip/pose")},
                {trigger_action_, path("/user/hand/right/input/trigger/value")},
                {squeeze_action_, path("/user/hand/right/input/squeeze/value")},
                {a_action_, path("/user/hand/right/input/a/click")},
                {grip_action_, path("/user/hand/left/input/grip/pose")},
                {trigger_action_, path("/user/hand/left/input/trigger/value")},
                {squeeze_action_, path("/user/hand/left/input/squeeze/value")},
                {a_action_, path("/user/hand/left/input/x/click")}})) {
    return false;  // core profile: failure means the setup itself is broken
  }

  // Pico (ByteDance) native profiles — Pico 4 and Pico 4 Ultra/4S. Grip is
  // bound to both squeeze/value and squeeze/click (floats combine by max;
  // click arrives as 0/1 where the analog source is absent).
  if (bd_controllers_available_) {
    for (const char* p :
         {"/interaction_profiles/bytedance/pico4_controller",
          "/interaction_profiles/bytedance/pico4s_controller"}) {
      suggest(
          p,
          {{grip_action_, path("/user/hand/right/input/grip/pose")},
           {trigger_action_, path("/user/hand/right/input/trigger/value")},
           {squeeze_action_, path("/user/hand/right/input/squeeze/value")},
           {squeeze_action_, path("/user/hand/right/input/squeeze/click")},
           {a_action_, path("/user/hand/right/input/a/click")},
           {grip_action_, path("/user/hand/left/input/grip/pose")},
           {trigger_action_, path("/user/hand/left/input/trigger/value")},
           {squeeze_action_, path("/user/hand/left/input/squeeze/value")},
           {squeeze_action_, path("/user/hand/left/input/squeeze/click")},
           {a_action_, path("/user/hand/left/input/x/click")}});
    }
  }

  // Last-resort fallback: khr/simple_controller — select (0/1 float
  // conversion) drives the clutch, menu resets; no analog gripper exists.
  suggest("/interaction_profiles/khr/simple_controller",
          {{grip_action_, path("/user/hand/right/input/grip/pose")},
           {squeeze_action_, path("/user/hand/right/input/select/click")},
           {a_action_, path("/user/hand/right/input/menu/click")},
           {grip_action_, path("/user/hand/left/input/grip/pose")},
           {squeeze_action_, path("/user/hand/left/input/select/click")},
           {a_action_, path("/user/hand/left/input/menu/click")}});

  XrSessionActionSetsAttachInfo attach{XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
  attach.countActionSets = 1;
  attach.actionSets = &action_set_;
  if (!XrOk(xrAttachSessionActionSets(session_, &attach),
            "xrAttachSessionActionSets")) {
    return false;
  }

  for (int h = 0; h < 2; ++h) {
    XrActionSpaceCreateInfo space_info{XR_TYPE_ACTION_SPACE_CREATE_INFO};
    space_info.action = grip_action_;
    space_info.subactionPath = hand_paths_[h];
    space_info.poseInActionSpace = {{0, 0, 0, 1}, {0, 0, 0}};
    if (!XrOk(xrCreateActionSpace(session_, &space_info, &grip_spaces_[h]),
              "xrCreateActionSpace")) {
      return false;
    }
  }
  return true;
}

void XrShell::LogInteractionProfiles() {
  const char* hands[2] = {"right", "left"};
  for (int h = 0; h < 2; ++h) {
    XrInteractionProfileState st{XR_TYPE_INTERACTION_PROFILE_STATE};
    if (XR_FAILED(
            xrGetCurrentInteractionProfile(session_, hand_paths_[h], &st))) {
      continue;
    }
    if (st.interactionProfile == XR_NULL_PATH) {
      LOGW("interaction profile (%s): NONE bound", hands[h]);
    } else {
      char buf[XR_MAX_PATH_LENGTH];
      uint32_t len = 0;
      xrPathToString(instance_, st.interactionProfile, sizeof(buf), &len, buf);
      LOGI("interaction profile (%s): %s", hands[h], buf);
    }
  }
}

namespace {
const char* SessionStateName(XrSessionState s) {
  switch (s) {
    case XR_SESSION_STATE_IDLE: return "IDLE";
    case XR_SESSION_STATE_READY: return "READY";
    case XR_SESSION_STATE_SYNCHRONIZED: return "SYNCHRONIZED";
    case XR_SESSION_STATE_VISIBLE: return "VISIBLE";
    case XR_SESSION_STATE_FOCUSED: return "FOCUSED";
    case XR_SESSION_STATE_STOPPING: return "STOPPING";
    case XR_SESSION_STATE_LOSS_PENDING: return "LOSS_PENDING";
    case XR_SESSION_STATE_EXITING: return "EXITING";
    default: return "?";
  }
}
}  // namespace

void XrShell::HandleSessionStateChange(
    const XrEventDataSessionStateChanged& ev, bool* exit_render_loop) {
  session_state_ = ev.state;
  // Input only flows in FOCUSED; if this never appears, no controller data.
  LOGI("session state -> %s", SessionStateName(ev.state));
  switch (ev.state) {
    case XR_SESSION_STATE_READY: {
      XrSessionBeginInfo info{XR_TYPE_SESSION_BEGIN_INFO};
      info.primaryViewConfigurationType =
          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
      if (XrOk(xrBeginSession(session_, &info), "xrBeginSession")) {
        session_running_ = true;
        LOGI("session running");
      }
      break;
    }
    case XR_SESSION_STATE_STOPPING:
      session_running_ = false;
      XrOk(xrEndSession(session_), "xrEndSession");
      LOGI("session stopped");
      break;
    case XR_SESSION_STATE_FOCUSED:
      LogInteractionProfiles();
      break;
    case XR_SESSION_STATE_EXITING:
    case XR_SESSION_STATE_LOSS_PENDING:
      session_running_ = false;
      *exit_render_loop = true;
      break;
    default:
      break;
  }
}

void XrShell::PollEvents(bool* exit_render_loop, XrInputState* input) {
  while (true) {
    XrEventDataBuffer ev{XR_TYPE_EVENT_DATA_BUFFER};
    if (xrPollEvent(instance_, &ev) != XR_SUCCESS) {
      break;
    }
    switch (ev.type) {
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED:
        HandleSessionStateChange(
            *reinterpret_cast<XrEventDataSessionStateChanged*>(&ev),
            exit_render_loop);
        break;
      case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING:
        *exit_render_loop = true;
        break;
      case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING:
        // Recentering mid-clutch causes a controller-pose jump; teleop
        // auto-disengages on this flag.
        if (input) {
          input->recenter = true;
        }
        LOGI("reference space change pending (recenter)");
        break;
      case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED:
        LogInteractionProfiles();
        break;
      default:
        break;
    }
  }
}

bool XrShell::WaitBeginFrame(XrFrameState* frame_state) {
  *frame_state = {XR_TYPE_FRAME_STATE};
  XrFrameWaitInfo wait{XR_TYPE_FRAME_WAIT_INFO};
  if (!XrOk(xrWaitFrame(session_, &wait, frame_state), "xrWaitFrame")) {
    return false;
  }
  XrFrameBeginInfo begin{XR_TYPE_FRAME_BEGIN_INFO};
  return XrOk(xrBeginFrame(session_, &begin), "xrBeginFrame");
}

bool XrShell::LocateViews(XrTime time, std::vector<XrView>* views) {
  views->assign(config_views_.size(), {XR_TYPE_VIEW});
  XrViewLocateInfo info{XR_TYPE_VIEW_LOCATE_INFO};
  info.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  info.displayTime = time;
  info.space = app_space_;
  XrViewState state{XR_TYPE_VIEW_STATE};
  uint32_t count = 0;
  if (!XrOk(xrLocateViews(session_, &info, &state,
                          static_cast<uint32_t>(views->size()), &count,
                          views->data()),
            "xrLocateViews")) {
    return false;
  }
  return (state.viewStateFlags & XR_VIEW_STATE_POSITION_VALID_BIT) &&
         (state.viewStateFlags & XR_VIEW_STATE_ORIENTATION_VALID_BIT);
}

bool XrShell::AcquireSwapchainImage(int view, uint32_t* image_index) {
  XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
  if (!XrOk(xrAcquireSwapchainImage(swapchains_[view].handle, &acq,
                                    image_index),
            "xrAcquireSwapchainImage")) {
    return false;
  }
  XrSwapchainImageWaitInfo wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
  wait.timeout = XR_INFINITE_DURATION;
  return XrOk(xrWaitSwapchainImage(swapchains_[view].handle, &wait),
              "xrWaitSwapchainImage");
}

bool XrShell::ReleaseSwapchainImage(int view) {
  XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
  return XrOk(xrReleaseSwapchainImage(swapchains_[view].handle, &rel),
              "xrReleaseSwapchainImage");
}

bool XrShell::EndFrame(
    const XrFrameState& frame_state,
    const std::vector<XrCompositionLayerProjectionView>& proj_views) {
  XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
  layer.space = app_space_;
  layer.viewCount = static_cast<uint32_t>(proj_views.size());
  layer.views = proj_views.data();
  const XrCompositionLayerBaseHeader* layers[] = {
      reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer)};

  XrFrameEndInfo info{XR_TYPE_FRAME_END_INFO};
  info.displayTime = frame_state.predictedDisplayTime;
  info.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  info.layerCount = proj_views.empty() ? 0 : 1;
  info.layers = proj_views.empty() ? nullptr : layers;
  return XrOk(xrEndFrame(session_, &info), "xrEndFrame");
}

void XrShell::SyncInput(XrTime time, XrInputState* input) {
  input->grip_valid = false;
  input->a_click = false;
  ++sync_count_;

  XrActiveActionSet active{action_set_, XR_NULL_PATH};
  XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
  sync.countActiveActionSets = 1;
  sync.activeActionSets = &active;
  XrResult sr = xrSyncActions(session_, &sync);
  if (XR_FAILED(sr)) {
    XrOk(sr, "xrSyncActions");
    return;
  }
  if (sr == XR_SESSION_NOT_FOCUSED && sync_count_ % 90 == 0) {
    LOGW("input: session not focused — runtime is withholding controller "
         "data (state %s)", SessionStateName(session_state_));
  }

  // First hand with a valid grip pose wins (0 = right, 1 = left).
  int hand = -1;
  for (int h = 0; h < 2 && hand < 0; ++h) {
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_SUCCEEDED(xrLocateSpace(grip_spaces_[h], app_space_, time, &loc))) {
      constexpr XrSpaceLocationFlags kValid =
          XR_SPACE_LOCATION_POSITION_VALID_BIT |
          XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
      if ((loc.locationFlags & kValid) == kValid) {
        input->grip_valid = true;
        input->grip = loc.pose;
        hand = h;
      }
    }
  }

  XrActionStateGetInfo get{XR_TYPE_ACTION_STATE_GET_INFO};
  get.subactionPath = hand < 0 ? hand_paths_[0] : hand_paths_[hand];
  bool trig_active = false, sqz_active = false;

  get.action = trigger_action_;
  XrActionStateFloat f{XR_TYPE_ACTION_STATE_FLOAT};
  if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &get, &f)) && f.isActive) {
    input->trigger = f.currentState;
    trig_active = true;
  }
  get.action = squeeze_action_;
  f = {XR_TYPE_ACTION_STATE_FLOAT};
  if (XR_SUCCEEDED(xrGetActionStateFloat(session_, &get, &f)) && f.isActive) {
    input->squeeze = f.currentState;
    sqz_active = true;
  }
  get.action = a_action_;
  XrActionStateBoolean b{XR_TYPE_ACTION_STATE_BOOLEAN};
  if (XR_SUCCEEDED(xrGetActionStateBoolean(session_, &get, &b)) &&
      b.isActive) {
    input->a_click = b.currentState && b.changedSinceLastSync;
  }

  if (sync_count_ % 90 == 0) {
    LOGI("input: hand=%s grip_valid=%d sqz=%.2f(active=%d) trig=%.2f(active=%d)",
         hand < 0 ? "none" : (hand == 0 ? "right" : "left"),
         input->grip_valid, input->squeeze, sqz_active, input->trigger,
         trig_active);
  }
}

void XrShell::Destroy() {
  for (auto& sc : swapchains_) {
    if (sc.handle != XR_NULL_HANDLE) {
      xrDestroySwapchain(sc.handle);
    }
  }
  swapchains_.clear();
  for (int h = 0; h < 2; ++h) {
    if (grip_spaces_[h] != XR_NULL_HANDLE) {
      xrDestroySpace(grip_spaces_[h]);
    }
  }
  if (action_set_ != XR_NULL_HANDLE) {
    xrDestroyActionSet(action_set_);
  }
  if (app_space_ != XR_NULL_HANDLE) {
    xrDestroySpace(app_space_);
  }
  if (session_ != XR_NULL_HANDLE) {
    xrDestroySession(session_);
  }
  if (instance_ != XR_NULL_HANDLE) {
    xrDestroyInstance(instance_);
  }
}
