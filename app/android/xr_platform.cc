#include "xr_platform.h"

#include <jni.h>

// Platform only. XR_USE_GRAPHICS_API_VULKAN is deliberately NOT defined here:
// nothing in this file touches a graphics binding, and defining it would drag
// <vulkan/vulkan.h> in as a prerequisite of openxr_platform.h for no reason.
#define XR_USE_PLATFORM_ANDROID
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <android_native_app_glue.h>

#include "mxr_log.h"

namespace {

// The one struct xrCreateInstance reads through `next`. Static rather than a
// local because XrShell::CreateInstance is a separate call and the pointer has
// to still be valid when it runs.
XrInstanceCreateInfoAndroidKHR g_android_info{
    XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};

}  // namespace

const void* mxr_android_xr_init(android_app* app) {
  // Android mandates explicit loader initialization before any other call.
  // The loader does not export extension entry points, so this one comes
  // through xrGetInstanceProcAddr with a null instance — the one call for
  // which that is legal.
  PFN_xrInitializeLoaderKHR init_loader = nullptr;
  XrResult r = xrGetInstanceProcAddr(
      XR_NULL_HANDLE, "xrInitializeLoaderKHR",
      reinterpret_cast<PFN_xrVoidFunction*>(&init_loader));
  if (XR_FAILED(r) || !init_loader) {
    LOGE("xrGetInstanceProcAddr(xrInitializeLoaderKHR) failed: %d", r);
    return nullptr;
  }

  XrLoaderInitInfoAndroidKHR loader_info{XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
  loader_info.applicationVM = app->activity->vm;
  loader_info.applicationContext = app->activity->clazz;
  r = init_loader(
      reinterpret_cast<XrLoaderInitInfoBaseHeaderKHR*>(&loader_info));
  if (XR_FAILED(r)) {
    LOGE("xrInitializeLoaderKHR failed: %d", r);
    return nullptr;
  }

  g_android_info.applicationVM = app->activity->vm;
  g_android_info.applicationActivity = app->activity->clazz;
  return &g_android_info;
}

const char* mxr_android_xr_extension(void) {
  return XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME;
}
