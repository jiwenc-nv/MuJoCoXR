#include "xr_shell.h"

#include <ctime>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "mesh_buffers.h"  // kNearZ / kFarZ, the projection this depth was rendered with
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

// xrGetSystem, with a bounded retry for the one XrResult that means "come
// back later". `system_wait_s` <= 0 makes exactly one attempt, which is what
// Android passes and why its behaviour here is unchanged.
//
// Returns false only on a HARD failure. A deadline that expires leaves
// system_id_ == XR_NULL_SYSTEM_ID and returns true, so the caller can decide.
bool XrShell::WaitForSystem(int system_wait_s) {
  XrSystemGetInfo sys{XR_TYPE_SYSTEM_GET_INFO};
  sys.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;

  const int64_t deadline_ms =
      system_wait_s > 0 ? static_cast<int64_t>(system_wait_s)*1000 : 0;
  // MEASURED, not counted. Accumulating a nominal 200 per sleep is wrong on
  // two counts: nanosleep returns early on a signal, and a shell that installs
  // handlers before this call (which it must — see set_abort_flag) makes that
  // the normal case rather than a curiosity. A monotonic origin is right
  // whatever the sleep actually did.
  struct timespec origin;
  clock_gettime(CLOCK_MONOTONIC, &origin);
  auto elapsed_ms = [&origin]() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<int64_t>(now.tv_sec - origin.tv_sec)*1000 +
           (now.tv_nsec - origin.tv_nsec)/1000000;
  };

  int64_t last_log_ms = -3000;
  while (true) {
    system_result_ = xrGetSystem(instance_, &sys, &system_id_);
    if (XR_SUCCEEDED(system_result_)) {
      const int64_t waited = elapsed_ms();
      if (waited >= 1000) {
        LOGI("system available after %lld.%llds",
             static_cast<long long>(waited/1000),
             static_cast<long long>((waited%1000)/100));
      }
      // Reported rather than judged: a streaming runtime answers this from a
      // virtual device with nothing worn, so the NAME is what tells a reader
      // which of those two situations they are in.
      XrSystemProperties props{XR_TYPE_SYSTEM_PROPERTIES};
      if (XR_SUCCEEDED(xrGetSystemProperties(instance_, system_id_, &props))) {
        // snprintf, not strncpy: both buffers are
        // XR_MAX_SYSTEM_NAME_SIZE, so a 255-char name makes strncpy copy
        // exactly the buffer minus one with no terminator in the source
        // range, which -Wstringop-truncation flags and is right to.
        snprintf(system_name_, sizeof(system_name_), "%s",
                 props.systemName);
      }
      return true;
    }
    // THE OTHER MEASURED FAILURE. Unlike RUNTIME_UNAVAILABLE this one is
    // transient by construction on a streaming runtime: the service is up and
    // answering, and it will start reporting a system the moment a headset
    // attaches. Retrying anything else would hide a real fault.
    if (system_result_ != XR_ERROR_FORM_FACTOR_UNAVAILABLE) {
      if (system_result_ == XR_ERROR_FORM_FACTOR_UNSUPPORTED) {
        LOGE("xrGetSystem: this runtime does not support a head-mounted "
             "display form factor at all (%d) — not a missing headset",
             system_result_);
      } else {
        LOGE("xrGetSystem failed: %d", system_result_);
      }
      system_id_ = XR_NULL_SYSTEM_ID;
      return false;
    }
    if (abort_flag_ && *abort_flag_) {
      LOGI("interrupted while waiting for a system");
      system_id_ = XR_NULL_SYSTEM_ID;
      return false;
    }
    const int64_t waited_ms = elapsed_ms();
    if (waited_ms >= deadline_ms) {
      LOGE("xrGetSystem: XR_ERROR_FORM_FACTOR_UNAVAILABLE (%d) — the runtime "
           "is running but no headset has connected%s", system_result_,
           deadline_ms > 0 ? " within the timeout" : "");
      system_id_ = XR_NULL_SYSTEM_ID;
      return true;
    }
    if (waited_ms - last_log_ms >= 3000) {
      LOGI("waiting for a headset to connect... (%llds of %ds)",
           static_cast<long long>(waited_ms/1000), system_wait_s);
      last_log_ms = waited_ms;
    }
    struct timespec ts = {0, 200*1000*1000};  // 200 ms
    nanosleep(&ts, nullptr);
  }
}

bool XrShell::CreateInstance(const void* platform_next,
                             const char* platform_ext, XrVersion api_version,
                             int system_wait_s) {
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
      local_floor_ext_ = true;
    }
    // Pico controllers use ByteDance's own interaction profiles.
    if (!strcmp(e.extensionName, "XR_BD_controller_interaction")) {
      bd_controllers_available_ = true;
    }
    // khr/generic_controller — the fallback that actually works. Unlike
    // khr/simple_controller it carries an analog trigger AND an analog
    // squeeze, so the gripper and the clutch both function on a runtime that
    // binds nothing more specific. Probed rather than assumed for the same
    // reason as the two above: xrCreateInstance returns
    // XR_ERROR_EXTENSION_NOT_PRESENT for ANY unadvertised name in the enabled
    // list, so an unconditional entry here is a hard startup failure on every
    // runtime that lacks it — including, for all anyone here can check, Quest.
    //
    // A string literal because it postdates the pinned OpenXR-SDK 1.1.38:
    // there is no XR_KHR_GENERIC_CONTROLLER_EXTENSION_NAME macro to use, and
    // it is absent from that SDK's registry. Same precedent as the line above.
    if (!strcmp(e.extensionName, "XR_KHR_generic_controller")) {
      generic_controller_available_ = true;
    }
    // Submitting depth is what lets a runtime REPROJECT rather than merely
    // rotate the last frame. Probed like the two above and for the same
    // reason: an unadvertised name in the enabled list is
    // XR_ERROR_EXTENSION_NOT_PRESENT, i.e. a hard startup failure.
    if (!strcmp(e.extensionName,
                XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME)) {
      depth_layer_ext_ = true;
    }
  }

  // platform_ext FIRST, so the enabled list is byte-for-byte what it was when
  // XR_KHR_android_create_instance was spelled here literally.
  std::vector<const char*> enabled;
  if (platform_ext) {
    enabled.push_back(platform_ext);
  }
  enabled.push_back(XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME);
  if (local_floor_ext_) {
    enabled.push_back(XR_EXT_LOCAL_FLOOR_EXTENSION_NAME);
  }
  if (bd_controllers_available_) {
    enabled.push_back("XR_BD_controller_interaction");
  }
  if (generic_controller_available_) {
    enabled.push_back("XR_KHR_generic_controller");
  }
  if (depth_layer_ext_) {
    enabled.push_back(XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
  }

  XrInstanceCreateInfo info{XR_TYPE_INSTANCE_CREATE_INFO};
  info.next = platform_next;
  strncpy(info.applicationInfo.applicationName, "MuJoCoXR",
          XR_MAX_APPLICATION_NAME_SIZE - 1);
  info.applicationInfo.applicationVersion = 1;
  strncpy(info.applicationInfo.engineName, "none",
          XR_MAX_ENGINE_NAME_SIZE - 1);
  info.applicationInfo.apiVersion = api_version;
  info.enabledExtensionCount = static_cast<uint32_t>(enabled.size());
  info.enabledExtensionNames = enabled.data();
  XrResult r = xrCreateInstance(&info, &instance_);

  // THE 1.1 REQUEST IS NOT COSMETIC AND NEITHER IS THIS FALLBACK. At 1.1
  // LOCAL_FLOOR is a core reference space needing no extension, which on a
  // runtime that does not advertise XR_EXT_local_floor is the difference
  // between a floor-referenced app space and the STAGE fallback. So a
  // downgrade is a DEGRADED CLIENT, not a formality, and it says so.
  if (r == XR_ERROR_API_VERSION_UNSUPPORTED &&
      api_version != XR_API_VERSION_1_0) {
    LOGW("runtime rejected OpenXR %u.%u; retrying at 1.0. LOCAL_FLOOR is core "
         "only at 1.1, so this client will fall back to STAGE and the floor "
         "datum may be wrong — read the `reference space:` line below",
         XR_VERSION_MAJOR(api_version), XR_VERSION_MINOR(api_version));
    api_version = XR_API_VERSION_1_0;
    info.applicationInfo.apiVersion = api_version;
    r = xrCreateInstance(&info, &instance_);
  }
  instance_result_ = r;
  if (XR_FAILED(r)) {
    // TWO FAILURES THAT LOOK THE SAME AND ARE NOT, and only one of them is
    // this tier's business. RUNTIME_UNAVAILABLE means the runtime named by
    // XR_RUNTIME_JSON did not come up — which is true of every OpenXR runtime
    // and so belongs here — but WHY it did not is entirely runtime-specific,
    // and this tier has no business knowing that any particular one exists.
    // The shell that does know reads instance_result() and says so; see
    // app/linux/main.cc. What stays here is the one thing a shell cannot
    // recover after the fact: the value the LOADER actually used.
    if (r == XR_ERROR_RUNTIME_UNAVAILABLE) {
      const char* manifest = getenv("XR_RUNTIME_JSON");
      LOGE("xrCreateInstance: XR_ERROR_RUNTIME_UNAVAILABLE (%d) — a runtime "
           "manifest was found but the runtime behind it did not come up", r);
      LOGE("  XR_RUNTIME_JSON = %s",
           manifest ? manifest : "(unset — the loader searched its default "
                                 "paths)");
    } else {
      LOGE("xrCreateInstance failed: %d", r);
    }
    return false;
  }
  // THE GRANTED VERSION, not the requested one — the fallback above may have
  // moved it, and the reference-space predicate reads this member.
  api_version_ = api_version;

  XrInstanceProperties props{XR_TYPE_INSTANCE_PROPERTIES};
  xrGetInstanceProperties(instance_, &props);
  LOGI("OpenXR runtime: %s %u.%u.%u, API %u.%u granted (local_floor_ext=%d, "
       "depth_layer_ext=%d)",
       props.runtimeName,
       XR_VERSION_MAJOR(props.runtimeVersion),
       XR_VERSION_MINOR(props.runtimeVersion),
       XR_VERSION_PATCH(props.runtimeVersion),
       XR_VERSION_MAJOR(api_version_), XR_VERSION_MINOR(api_version_),
       local_floor_ext_, depth_layer_ext_);

  if (!WaitForSystem(system_wait_s)) {
    return false;
  }
  // The caller decides whether a missing system is fatal: --probe reports it,
  // everything else refuses to continue. See have_system().
  if (system_id_ == XR_NULL_SYSTEM_ID) {
    return true;
  }

  // AR: prefer alpha-blend (video passthrough behind rendered alpha).
  uint32_t nmodes = 0;
  xrEnumerateEnvironmentBlendModes(instance_, system_id_,
                                   XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                   0, &nmodes, nullptr);
  std::vector<XrEnvironmentBlendMode> modes(nmodes);
  xrEnumerateEnvironmentBlendModes(instance_, system_id_,
                                   XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
                                   nmodes, &nmodes, modes.data());
  for (XrEnvironmentBlendMode m : modes) {
    if (m == XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) {
      blend_mode_ = m;
      break;
    }
  }
  LOGI("environment blend mode: %s",
       passthrough() ? "ALPHA_BLEND (passthrough)" : "OPAQUE");
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
  // TWO WAYS TO BE ALLOWED TO ASK FOR LOCAL_FLOOR, and this used to know only
  // one. XR_EXT_local_floor was promoted into OpenXR 1.1, where the space is
  // core and no extension is advertised for it — so on a 1.1 instance the
  // extension probe reports false and the loop would have skipped straight to
  // STAGE, concluding the floor is unavailable on a runtime that offers it.
  // That is not hypothetical: it is exactly the CloudXR case this shell was
  // extended for. XR_REFERENCE_SPACE_TYPE_LOCAL_FLOOR_EXT below is the same
  // enum value as the 1.1 core name, so the array needs no second entry.
  // MAJOR.MINOR ONLY. XrVersion packs a patch field, and api_version_ holds
  // whatever was granted, so comparing the whole word is right only while the
  // caller happens to pass the same patch this header was built with. A
  // XR_MAKE_VERSION(1,1,0) would silently compare LOW and drop to STAGE with
  // no log line — the exact silent-wrong-floor failure this predicate exists
  // to prevent. Monado compares major.minor for the same reason.
  const bool api_at_least_1_1 =
      XR_VERSION_MAJOR(api_version_) > 1 ||
      (XR_VERSION_MAJOR(api_version_) == 1 &&
       XR_VERSION_MINOR(api_version_) >= 1);
  const bool local_floor_ok = local_floor_ext_ || api_at_least_1_1;
  for (int i = local_floor_ok ? 0 : 1; i < 3; ++i) {
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

  // ONE ENUMERATION, TWO PICKS. The runtime returns colour and depth formats
  // in a single mixed list, in its own order of preference — so a depth format
  // is chosen by intersecting THIS list, not by asking the physical device
  // what it can do. Those are different questions: a device that supports
  // D32_SFLOAT as an attachment says nothing about whether the runtime will
  // accept a swapchain of it, and a swapchain the runtime rejects is a
  // startup failure rather than a fallback.
  uint32_t nfmt = 0;
  xrEnumerateSwapchainFormats(session_, 0, &nfmt, nullptr);
  std::vector<int64_t> formats(nfmt);
  xrEnumerateSwapchainFormats(session_, nfmt, &nfmt, formats.data());
  auto pick = [&formats](const std::vector<int64_t>& want) {
    for (int64_t w : want) {
      for (int64_t have : formats) {
        if (w == have) {
          return w;
        }
      }
    }
    return static_cast<int64_t>(0);
  };

  swapchain_format_ = pick(preferred);
  if (!swapchain_format_) {
    LOGE("no preferred swapchain format available");
    return false;
  }

  // The depth preference is NOT a parameter the way `preferred` is. Both
  // shells want the same thing and neither has an opinion to express, so a
  // second argument would be the same list written twice in two main.cc files
  // to vary nothing.
  if (depth_layer_ext_) {
    depth_swapchain_format_ = pick({VK_FORMAT_D32_SFLOAT,
                                    VK_FORMAT_D24_UNORM_S8_UINT,
                                    VK_FORMAT_D16_UNORM});
    if (!depth_swapchain_format_) {
      // Advertised the extension, offers no depth format. Degrade to the
      // app-owned depth image rather than fail: the picture is still correct,
      // only the reprojection quality is lost.
      LOGW("runtime advertises %s but offers no depth swapchain format — "
           "falling back to an app-owned depth image, so head-rotation "
           "reprojection will be unavailable",
           XR_KHR_COMPOSITION_LAYER_DEPTH_EXTENSION_NAME);
    }
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

  if (!depth_swapchain_format_) {
    return true;
  }
  // Same geometry and sample count as the colour swapchain by construction —
  // both are built from the same XrViewConfigurationView, and a mismatch
  // would make the depth image describe a different framebuffer than the one
  // it was rendered with.
  depth_swapchains_.resize(config_views_.size());
  for (size_t i = 0; i < config_views_.size(); ++i) {
    const auto& cv = config_views_[i];
    XrSwapchainCreateInfo info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    info.usageFlags = XR_SWAPCHAIN_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    info.format = depth_swapchain_format_;
    info.sampleCount = 1;
    info.width = cv.recommendedImageRectWidth;
    info.height = cv.recommendedImageRectHeight;
    info.faceCount = 1;
    info.arraySize = 1;
    info.mipCount = 1;
    auto& sc = depth_swapchains_[i];
    if (!XrOk(xrCreateSwapchain(session_, &info, &sc.handle),
              "xrCreateSwapchain(depth)")) {
      // Leave nothing half-built: depth_layer() is "the vector is non-empty",
      // so a partial vector would claim a capability that does not exist.
      depth_swapchains_.clear();
      depth_swapchain_format_ = 0;
      LOGW("depth swapchain creation failed — continuing without depth "
           "submission; reprojection will be unavailable");
      return true;
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
              "xrEnumerateSwapchainImages(depth)")) {
      return false;
    }
    LOGI("depth swapchain %zu: %dx%d, %u images, format %lld", i, sc.width,
         sc.height, nimg, static_cast<long long>(depth_swapchain_format_));
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
      {"scene_click", XR_ACTION_TYPE_BOOLEAN_INPUT, &b_action_},
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
  // squeeze = clutch, trigger = analog gripper, A/X = reset, B/Y = cycle to
  // the next scene. B is the one action that does not become an InputState
  // field — see b_down() in xr_shell.h for why the core cannot own it.
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
      LOGW("interaction profile %s: REJECTED (%d)", profile_path, r);
      return false;
    }
    ++profiles_accepted_;
    LOGI("interaction profile %s: accepted", profile_path);
    return true;
  };
  // THE THIRD OUTCOME, and the reason it gets a line of its own: a count of
  // accepted profiles cannot distinguish "the runtime refused it" from "we
  // never offered it", and the ones we skip are skipped precisely because
  // their defining extension is absent — which is the interesting case.
  auto not_offered = [](const char* profile_path, const char* ext) {
    LOGI("interaction profile %s: not offered (%s not advertised)",
         profile_path, ext);
  };

  // NO SINGLE PROFILE IS MANDATORY, and this used to say the opposite: a
  // failed oculus/touch_controller suggestion returned false and killed
  // bring-up, on the reasoning that a core profile failing means the setup is
  // broken. Two things make that the wrong bet now. The profile a runtime
  // ACCEPTS SUGGESTIONS FOR and the profile it BINDS TO A DEVICE are different
  // questions, so a rejection here says nothing about whether a controller
  // works; and this shell now runs against a streaming runtime whose accepted
  // set is a build-time property of somebody else's binary. The check that
  // survives is the one that means something: at least one profile took.
  //
  // Which profile actually bound is a separate, louder line —
  // LogInteractionProfiles() at FOCUSED. Read that one, not these.
  suggest("/interaction_profiles/oculus/touch_controller",
          {{grip_action_, path("/user/hand/right/input/grip/pose")},
           {trigger_action_, path("/user/hand/right/input/trigger/value")},
           {squeeze_action_, path("/user/hand/right/input/squeeze/value")},
           {a_action_, path("/user/hand/right/input/a/click")},
           {b_action_, path("/user/hand/right/input/b/click")},
           {grip_action_, path("/user/hand/left/input/grip/pose")},
           {trigger_action_, path("/user/hand/left/input/trigger/value")},
           {squeeze_action_, path("/user/hand/left/input/squeeze/value")},
           {a_action_, path("/user/hand/left/input/x/click")},
           {b_action_, path("/user/hand/left/input/y/click")}});

  // Pico (ByteDance) native profiles — Pico 4 and Pico 4 Ultra/4S. Grip is
  // bound to both squeeze/value and squeeze/click (floats combine by max;
  // click arrives as 0/1 where the analog source is absent).
  if (!bd_controllers_available_) {
    not_offered("/interaction_profiles/bytedance/pico4*_controller",
                "XR_BD_controller_interaction");
  } else {
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
           {b_action_, path("/user/hand/right/input/b/click")},
           {grip_action_, path("/user/hand/left/input/grip/pose")},
           {trigger_action_, path("/user/hand/left/input/trigger/value")},
           {squeeze_action_, path("/user/hand/left/input/squeeze/value")},
           {squeeze_action_, path("/user/hand/left/input/squeeze/click")},
           {a_action_, path("/user/hand/left/input/x/click")},
           {b_action_, path("/user/hand/left/input/y/click")}});
    }
  }

  // khr/generic_controller — THE FALLBACK THAT KEEPS EVERY ACTION WORKING,
  // and the reason it is worth enabling an extension for. Component paths read
  // off this runtime's own binding table rather than guessed: `primary` and
  // `secondary` are its two buttons, NOT a/b and NOT menu, so A (reset) and B
  // (scene cycle) both land — which khr/simple_controller below cannot do,
  // having no second button and no analog axes at all.
  if (generic_controller_available_) {
    suggest("/interaction_profiles/khr/generic_controller",
            {{grip_action_, path("/user/hand/right/input/grip/pose")},
             {trigger_action_, path("/user/hand/right/input/trigger/value")},
             {squeeze_action_, path("/user/hand/right/input/squeeze/value")},
             {a_action_, path("/user/hand/right/input/primary/click")},
             {b_action_, path("/user/hand/right/input/secondary/click")},
             {grip_action_, path("/user/hand/left/input/grip/pose")},
             {trigger_action_, path("/user/hand/left/input/trigger/value")},
             {squeeze_action_, path("/user/hand/left/input/squeeze/value")},
             {a_action_, path("/user/hand/left/input/primary/click")},
             {b_action_, path("/user/hand/left/input/secondary/click")}});
  } else {
    not_offered("/interaction_profiles/khr/generic_controller",
                "XR_KHR_generic_controller");
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

  // The one binding check that survives. khr/simple_controller is mandatory
  // for every conformant runtime, so zero here is not "an unusual headset" —
  // it is a broken instance or a broken action set, and continuing would
  // produce a session that renders and never moves.
  if (profiles_accepted_ == 0) {
    LOGE("no interaction profile accepted a single suggestion — controller "
         "input cannot work; refusing to continue");
    return false;
  }
  LOGI("interaction profiles accepted: %d", profiles_accepted_);
  return true;
}

// The session-scope half of what used to be one function. THE SPLIT IS THE
// SPEC'S OWN: xrCreateActionSet, xrCreateAction and
// xrSuggestInteractionProfileBindings all take an XrInstance, while the two
// calls below take an XrSession — so everything above runs with no headset
// attached, and that is the only reason the interaction-profile work in this
// file can be checked at all before hardware exists (app/linux --probe).
//
// Android still calls CreateActions() then AttachActions() back to back after
// CreateSession(), so ITS order of OpenXR calls is unchanged. app/linux does
// not, and that is the point rather than an inconsistency: it calls
// CreateActions() before there is a session at all, which is what lets --probe
// report the whole interaction-profile decision on a machine with no headset.
bool XrShell::AttachActions() {
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
      // THE GUARD IS NOT DECORATION. Both sites below used to write
      // session_running_ = false unconditionally, and the shells absorbed the
      // difference in a `session_was_running` local. Latching the edge here
      // instead means it has to fire on a TRANSITION — LOSS_PENDING arriving
      // when the session never began must not manufacture a session end.
      if (session_running_) {
        session_running_ = false;
        session_end_edge_ = true;
      }
      XrOk(xrEndSession(session_), "xrEndSession");
      LOGI("session stopped");
      break;
    case XR_SESSION_STATE_FOCUSED:
      LogInteractionProfiles();
      break;
    case XR_SESSION_STATE_EXITING:
    case XR_SESSION_STATE_LOSS_PENDING:
      if (session_running_) {
        session_running_ = false;
        session_end_edge_ = true;
      }
      *exit_render_loop = true;
      break;
    default:
      break;
  }
}

void XrShell::PollEvents(bool* exit_render_loop, InputState* input) {
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
          input->recenter_edge = true;
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

bool XrShell::AcquireFrom(const SwapchainInfo& sc, uint32_t* image_index) {
  XrSwapchainImageAcquireInfo acq{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
  if (!XrOk(xrAcquireSwapchainImage(sc.handle, &acq, image_index),
            "xrAcquireSwapchainImage")) {
    return false;
  }
  XrSwapchainImageWaitInfo wait{XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
  wait.timeout = XR_INFINITE_DURATION;
  return XrOk(xrWaitSwapchainImage(sc.handle, &wait), "xrWaitSwapchainImage");
}

bool XrShell::ReleaseFrom(const SwapchainInfo& sc) {
  XrSwapchainImageReleaseInfo rel{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
  return XrOk(xrReleaseSwapchainImage(sc.handle, &rel),
              "xrReleaseSwapchainImage");
}

bool XrShell::AcquireSwapchainImage(int view, uint32_t* image_index) {
  return AcquireFrom(swapchains_[view], image_index);
}

bool XrShell::ReleaseSwapchainImage(int view) {
  return ReleaseFrom(swapchains_[view]);
}

bool XrShell::AcquireDepthImage(int view, uint32_t* image_index) {
  // *image_index stays 0 with no depth layer, which is what makes the
  // framebuffer table below degenerate to exactly the pre-depth one.
  *image_index = 0;
  if (!depth_layer()) {
    return true;
  }
  return AcquireFrom(depth_swapchains_[view], image_index);
}

bool XrShell::ReleaseDepthImage(int view) {
  if (!depth_layer()) {
    return true;
  }
  return ReleaseFrom(depth_swapchains_[view]);
}

void XrShell::ChainDepthInfo(
    std::vector<XrCompositionLayerProjectionView>* proj_views,
    std::vector<XrCompositionLayerDepthInfoKHR>* depth_infos) {
  depth_infos->clear();
  if (!depth_layer() || proj_views->empty()) {
    return;
  }
  // SIZED ONCE, THEN ADDRESSED. resize() may reallocate, so every address
  // taken below must be taken after the last one — hence a single resize
  // before the loop rather than push_back inside it.
  depth_infos->resize(proj_views->size(),
                      {XR_TYPE_COMPOSITION_LAYER_DEPTH_INFO_KHR});
  for (size_t i = 0; i < proj_views->size(); ++i) {
    const auto& sc = depth_swapchains_[i];
    auto& d = (*depth_infos)[i];
    d.subImage.swapchain = sc.handle;
    d.subImage.imageRect = {{0, 0}, {sc.width, sc.height}};
    d.subImage.imageArrayIndex = 0;
    // VERIFIED AGAINST ProjFromFov IN scene_renderer.cc RATHER THAN ASSUMED,
    // because getting it backwards is silently wrong rather than loud: that
    // projection is STANDARD Z, mapping near->0 and far->1 (m22 = f/(n-f),
    // m23 = f*n/(n-f), m32 = -1, so at z=-n the ndc z is 0 and at z=-f it is
    // 1). If it is ever changed to reverse-Z, minDepth/maxDepth stay 0/1 and
    // nearZ/farZ SWAP — the spec signals reversal by nearZ > farZ.
    d.minDepth = 0.0f;
    d.maxDepth = 1.0f;
    d.nearZ = kNearZ;
    d.farZ = kFarZ;
    (*proj_views)[i].next = &d;
  }
}

bool XrShell::EndFrame(
    const XrFrameState& frame_state,
    const std::vector<XrCompositionLayerProjectionView>& proj_views) {
  XrCompositionLayerProjection layer{XR_TYPE_COMPOSITION_LAYER_PROJECTION};
  layer.space = app_space_;
  layer.viewCount = static_cast<uint32_t>(proj_views.size());
  layer.views = proj_views.data();
  if (passthrough()) {
    // Straight-alpha layer blended over the camera feed.
    layer.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                       XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
  }
  const XrCompositionLayerBaseHeader* layers[] = {
      reinterpret_cast<const XrCompositionLayerBaseHeader*>(&layer)};

  XrFrameEndInfo info{XR_TYPE_FRAME_END_INFO};
  info.displayTime = frame_state.predictedDisplayTime;
  info.environmentBlendMode = blend_mode_;
  info.layerCount = proj_views.empty() ? 0 : 1;
  info.layers = proj_views.empty() ? nullptr : layers;
  return XrOk(xrEndFrame(session_, &info), "xrEndFrame");
}

void XrShell::SyncInput(XrTime time, InputState* input) {
  input->grip_valid = false;
  input->a_down = false;
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
  constexpr XrSpaceLocationFlags kValid =
      XR_SPACE_LOCATION_POSITION_VALID_BIT |
      XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
  constexpr XrSpaceLocationFlags kTracked =
      XR_SPACE_LOCATION_POSITION_TRACKED_BIT |
      XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
  // Carried out of the loop only so the diagnostic at the bottom can read it.
  XrSpaceLocationFlags grip_flags = 0;
  int hand = -1;
  for (int h = 0; h < 2 && hand < 0; ++h) {
    XrSpaceLocation loc{XR_TYPE_SPACE_LOCATION};
    if (XR_SUCCEEDED(xrLocateSpace(grip_spaces_[h], app_space_, time, &loc))) {
      if ((loc.locationFlags & kValid) == kValid) {
        grip_flags = loc.locationFlags;
        input->grip_valid = true;
        // Field by field, never memcpy: XrPosef is {orientation, position}
        // — orientation FIRST — while InputState leads with position. A
        // memcpy compiles and is silently wrong.
        input->grip_quat[0] = loc.pose.orientation.x;
        input->grip_quat[1] = loc.pose.orientation.y;
        input->grip_quat[2] = loc.pose.orientation.z;
        input->grip_quat[3] = loc.pose.orientation.w;
        input->grip_pos[0] = loc.pose.position.x;
        input->grip_pos[1] = loc.pose.position.y;
        input->grip_pos[2] = loc.pose.position.z;
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
    // Raw level; Teleop derives the edge. Equivalent to the previous
    // `currentState && changedSinceLastSync` ON THIS SIDE of the ABI,
    // because xrSyncActions runs exactly once per frame, so `changed` was
    // already a level diff against the previous frame.
    //
    // Composed with the core, ONE frame differs and does so deliberately:
    // with A already held at session start the old code fired a reset on
    // frame 1, and Teleop::Update's `frame_ > 1` now suppresses it. Better,
    // but not "bit-identical" — say which, rather than claim identity.
    input->a_down = b.currentState;
  }

  // B/Y, the scene-cycle button. Read the same way as A but parked on the
  // shell rather than in InputState — see XrShell::b_down(). Left false when
  // the action is inactive (khr/simple_controller has no B at all), so a
  // fallback controller simply cannot switch scenes rather than switching
  // them at random.
  b_down_ = false;
  get.action = b_action_;
  XrActionStateBoolean bb{XR_TYPE_ACTION_STATE_BOOLEAN};
  if (XR_SUCCEEDED(xrGetActionStateBoolean(session_, &get, &bb)) &&
      bb.isActive) {
    b_down_ = bb.currentState;
  }

  // EVERY FRAME, not once in 90 — the same shape as SimScene's zero_dt_run_
  // and Teleop's recenter_run_, and for the same reason. A pose that is VALID
  // but NOT TRACKED is a stale pose presented as live, and the mask above
  // deliberately does not reject it: CloudXR ages a controller measurement out
  // in two stages — TRACKED clears at 100 ms, VALID at 200 ms — so between
  // those the runtime keeps handing back the last pose it received while the
  // operator's hand has moved on. Held through a clutch, the target sits still
  // and then jumps on recovery.
  //
  // Sampling this inside the 1.25 s periodic block would have caught a 100 ms
  // window roughly one time in ten, which is worse than not looking: it is
  // supposed to be the TRIGGER for adding the TRACKED bits to kValid, and it
  // is what someone greps for after seeing the target jump. A trigger that
  // misses nine events in ten is not a trigger.
  //
  // This LOOKS rather than fixes, on purpose. Widening kValid is the
  // spec-correct change, but it is shared-tier behaviour that also ships to
  // Android, where nobody can re-verify it.
  if (input->grip_valid && (grip_flags & kTracked) != kTracked) {
    ++not_tracked_frames_;
    if (++not_tracked_run_ == 1) {
      LOGW("input: grip pose VALID but NOT TRACKED (flags 0x%llx) — the "
           "runtime is serving a stale pose; treat target motion as suspect",
           static_cast<unsigned long long>(grip_flags));
    }
  } else {
    not_tracked_run_ = 0;
  }

  if (sync_count_ % 90 == 0) {
    LOGI("input: hand=%s grip_valid=%d sqz=%.2f(active=%d) trig=%.2f(active=%d)"
         " not_tracked_total=%lld",
         hand < 0 ? "none" : (hand == 0 ? "right" : "left"),
         input->grip_valid, input->squeeze, sqz_active, input->trigger,
         trig_active, static_cast<long long>(not_tracked_frames_));
  }
}

void XrShell::Destroy() {
  for (auto& sc : swapchains_) {
    if (sc.handle != XR_NULL_HANDLE) {
      xrDestroySwapchain(sc.handle);
    }
  }
  swapchains_.clear();
  for (auto& sc : depth_swapchains_) {
    if (sc.handle != XR_NULL_HANDLE) {
      xrDestroySwapchain(sc.handle);
    }
  }
  depth_swapchains_.clear();
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
