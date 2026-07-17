// APK assets -> mjVFS -> mjModel:
// AAssetManager buffers registered via mj_addBufferVFS (primary path). VFS
// keys are basename-stripped and lowercased, so the flattened assets/franka/
// dir maps 1:1; the 67 mesh files have unique case-insensitive basenames.

#ifndef MUJOCOXR_APP_ASSETS_H_
#define MUJOCOXR_APP_ASSETS_H_

#include <android/asset_manager.h>

#include <mujoco/mujoco.h>

// Loads every file under assets/franka/ into a VFS and compiles scene.xml.
// Returns nullptr on failure (details on logcat).
mjModel* mxr_load_model_from_assets(AAssetManager* am);

#endif  // MUJOCOXR_APP_ASSETS_H_
