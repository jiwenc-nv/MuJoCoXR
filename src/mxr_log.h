// One logging surface for every target, because the portable tier is shared
// and a target-specific logger inside it would make the tier target-specific.
// Two backends, not three: under Emscripten stderr is already routed to
// console.error, so the browser needs none of its own. Every C logging site
// in the tree routes through here, mxr_error.h included, so the tag and the
// target split exist once.
//
// The FORMAT STRING is identical across both backends, so a doc line
// quoting a log line is right on all of them. The prefix is not: logcat
// carries the tag and level out of band, while the stderr backend writes
// them inline as `[mujocoxr I|W|E]`. app/web/shell.js is a third writer to
// the same console and tags its own lines `[mujocoxr]` with no level, which
// is how you tell a shell line from a core line; one filter on "mujocoxr"
// catches all of them.

#ifndef MUJOCOXR_SRC_MXR_LOG_H_
#define MUJOCOXR_SRC_MXR_LOG_H_

#define MXR_TAG "mujocoxr"

#ifdef __ANDROID__

#include <android/log.h>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, MXR_TAG, __VA_ARGS__)
#define LOGW(...) __android_log_print(ANDROID_LOG_WARN, MXR_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, MXR_TAG, __VA_ARGS__)

#else

#include <stdio.h>

#define MXR_LOG_AT(level, ...)                    \
  do {                                            \
    fprintf(stderr, "[" MXR_TAG " " level "] ");  \
    fprintf(stderr, __VA_ARGS__);                 \
    fputc('\n', stderr);                          \
  } while (0)

#define LOGI(...) MXR_LOG_AT("I", __VA_ARGS__)
#define LOGW(...) MXR_LOG_AT("W", __VA_ARGS__)
#define LOGE(...) MXR_LOG_AT("E", __VA_ARGS__)

#endif  // __ANDROID__

#endif  // MUJOCOXR_SRC_MXR_LOG_H_
