// The Android half of OpenXR instance creation, and the ONLY file in this tree
// that defines XR_USE_PLATFORM_ANDROID.
//
// It exists because app/openxr/ may not name a platform (see the tier rule at
// the top of app/openxr/xr_shell.h, enforced by a configure-time scan). Two
// things are genuinely Android-only about bringing up an OpenXR instance —
// the mandatory xrInitializeLoaderKHR call and the
// XrInstanceCreateInfoAndroidKHR carrying the VM and activity — and both live
// here, behind an interface that mentions neither OpenXR nor JNI types. That
// is deliberate: it means app/android/main.cc needs no OpenXR platform header
// either, so there is no "which header got included first" ordering hazard
// anywhere in the Android target.

#ifndef MUJOCOXR_APP_ANDROID_XR_PLATFORM_H_
#define MUJOCOXR_APP_ANDROID_XR_PLATFORM_H_

struct android_app;

// Runs Android's mandatory loader initialization, then returns the
// XrInstanceCreateInfoAndroidKHR to chain into XrShell::CreateInstance's
// `platform_next`, or nullptr if loader init failed (already logged).
//
// The storage is owned by this module and lives for the process, which is what
// xrCreateInstance requires of a `next` chain it reads during the call.
const void* mxr_android_xr_init(android_app* app);

// The instance extension that chain requires — XrShell::CreateInstance's
// `platform_ext`. Spelled here so the constant travels with the struct that
// needs it rather than being restated at the call site.
const char* mxr_android_xr_extension(void);

#endif  // MUJOCOXR_APP_ANDROID_XR_PLATFORM_H_
