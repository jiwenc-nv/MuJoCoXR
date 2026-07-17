// Logcat helpers for the MuJoCoXR app.

#ifndef MUJOCOXR_APP_MXR_LOG_H_
#define MUJOCOXR_APP_MXR_LOG_H_

#include <android/log.h>

#define MXR_TAG "mujocoxr"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, MXR_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, MXR_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, MXR_TAG, __VA_ARGS__)

#endif  // MUJOCOXR_APP_MXR_LOG_H_
