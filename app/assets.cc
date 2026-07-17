#include "assets.h"

#include <cstdlib>
#include <cstring>
#include <vector>

#include "mxr_log.h"

mjModel* mxr_load_model_from_assets(AAssetManager* am) {
  AAssetDir* dir = AAssetManager_openDir(am, "franka");
  if (!dir) {
    LOGE("assets/franka not found in APK");
    return nullptr;
  }

  mjVFS* vfs = static_cast<mjVFS*>(malloc(sizeof(mjVFS)));
  mj_defaultVFS(vfs);

  int nfiles = 0;
  const char* name;
  while ((name = AAssetDir_getNextFileName(dir)) != nullptr) {
    char path[256];
    snprintf(path, sizeof(path), "franka/%s", name);
    AAsset* asset = AAssetManager_open(am, path, AASSET_MODE_BUFFER);
    if (!asset) {
      LOGE("cannot open asset %s", path);
      continue;
    }
    const void* buf = AAsset_getBuffer(asset);
    off_t len = AAsset_getLength(asset);
    // mj_addBufferVFS copies the buffer; keys are basenames (lowercased).
    if (mj_addBufferVFS(vfs, name, buf, static_cast<int>(len)) != 0) {
      LOGE("mj_addBufferVFS failed for %s", name);
    } else {
      ++nfiles;
    }
    AAsset_close(asset);
  }
  AAssetDir_close(dir);
  LOGI("VFS: %d files registered", nfiles);

  char err[1024] = {0};
  mjModel* m = mj_loadXML("scene.xml", vfs, err, sizeof(err));
  if (!m) {
    LOGE("mj_loadXML failed: %s", err);
  } else {
    LOGI("model loaded: nq=%ld nv=%ld nu=%ld nmesh=%ld",
         static_cast<long>(m->nq), static_cast<long>(m->nv),
         static_cast<long>(m->nu), static_cast<long>(m->nmesh));
  }
  mj_deleteVFS(vfs);
  free(vfs);
  return m;
}
