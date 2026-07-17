// Route MuJoCo errors/warnings to stderr (host) and logcat (device).
// Must be installed before the first MuJoCo call.

#ifndef MUJOCOXR_SRC_MXR_ERROR_H_
#define MUJOCOXR_SRC_MXR_ERROR_H_

#include <stdio.h>
#include <stdlib.h>

#include <mujoco/mujoco.h>

#ifdef __ANDROID__
#include <android/log.h>
#endif

static inline void mxr_handle_error(const char* msg) {
#ifdef __ANDROID__
  __android_log_print(ANDROID_LOG_ERROR, "mujocoxr", "MuJoCo error: %s", msg);
#endif
  fprintf(stderr, "MuJoCo error: %s\n", msg);
  abort();  // mju_user_error must not return
}

static inline void mxr_handle_warning(const char* msg) {
#ifdef __ANDROID__
  __android_log_print(ANDROID_LOG_WARN, "mujocoxr", "MuJoCo warning: %s", msg);
#endif
  fprintf(stderr, "MuJoCo warning: %s\n", msg);
}

static inline void mxr_install_error_hooks(void) {
  mju_user_error = mxr_handle_error;
  mju_user_warning = mxr_handle_warning;
}

#endif  // MUJOCOXR_SRC_MXR_ERROR_H_
