// Route MuJoCo's own errors/warnings into the app's log. Must be installed
// before the first MuJoCo call.
//
// Goes through mxr_log.h rather than carrying its own __ANDROID__ split:
// that header claims to be the one logging surface for every target, and a
// second copy of the target split plus a second copy of the tag string, one
// file over, is exactly what would falsify it.

#ifndef MUJOCOXR_SRC_MXR_ERROR_H_
#define MUJOCOXR_SRC_MXR_ERROR_H_

#include <stdlib.h>

#include <mujoco/mujoco.h>

#include "mxr_log.h"

static inline void mxr_handle_error(const char* msg) {
  LOGE("MuJoCo error: %s", msg);
  abort();  // mju_user_error must not return
}

static inline void mxr_handle_warning(const char* msg) {
  LOGW("MuJoCo warning: %s", msg);
}

static inline void mxr_install_error_hooks(void) {
  mju_user_error = mxr_handle_error;
  mju_user_warning = mxr_handle_warning;
}

#endif  // MUJOCOXR_SRC_MXR_ERROR_H_
