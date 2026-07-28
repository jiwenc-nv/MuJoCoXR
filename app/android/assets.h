// APK assets -> mjVFS -> mjModel:
// AAssetManager buffers registered via mj_addBufferVFS (primary path). VFS
// keys are basename-stripped and lowercased, so each flattened assets/<id>/
// dir maps 1:1. Basenames only need to be unique WITHIN one scene, because
// one call builds one VFS from one directory — which is why both scenes can
// contain an ar_scene.xml and a LICENSE.

#ifndef MUJOCOXR_APP_ANDROID_ASSETS_H_
#define MUJOCOXR_APP_ANDROID_ASSETS_H_

#include <android/asset_manager.h>

#include <mujoco/mujoco.h>

// Loads every file under assets/<scene_id>/ into a VFS and compiles that
// directory's ar_scene.xml. `scene_id` is MxrScene::id — the SAME string that
// names the CMake staging dir and the wasm mount point, which is what keeps
// the shells free of per-robot literals. Returns nullptr on failure (details
// on logcat).
mjModel* mxr_load_model_from_assets(AAssetManager* am, const char* scene_id);

#endif  // MUJOCOXR_APP_ANDROID_ASSETS_H_
