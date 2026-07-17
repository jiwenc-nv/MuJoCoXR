// MuJoCoXR NativeActivity entry point: raw OpenXR + Vulkan, unmodified
// Menagerie Franka scene, clutched DLS teleop from the right Touch
// controller. One xrWaitFrame-paced loop (single-threaded by decision):
// sync actions -> locate grip -> clutch/IK -> accumulator-owed mj_steps
// (catch-up capped at ~2 frames, deficit logged) -> mjv_updateScene +
// decor -> two eye passes. Sim time tracks display time; no interpolation.

#include <android_native_app_glue.h>

#include <vector>

#include <mujoco/mujoco.h>

#include "assets.h"
#include "mxr_error.h"
#include "mxr_log.h"
#include "scene_renderer.h"
#include "teleop.h"
#include "vk_context.h"
#include "xr_shell.h"

namespace {

struct AppState {
  bool resumed = false;
};

// World-axes gizmo decor: the handedness gate checks each axis on-device (+x red,
// +y green, +z blue) before any teleop math is trusted.
void AppendAxesGizmo(mjvScene* scn) {
  const mjtNum sizes[3][3] = {
      {0.4, 0.012, 0.012}, {0.012, 0.4, 0.012}, {0.012, 0.012, 0.4}};
  const mjtNum poses[3][3] = {{0.4, 0, 0.02}, {0, 0.4, 0.02}, {0, 0, 0.42}};
  const float rgba[3][4] = {
      {1, 0.2f, 0.2f, 1}, {0.2f, 1, 0.2f, 1}, {0.25f, 0.45f, 1, 1}};
  for (int i = 0; i < 3 && scn->ngeom < scn->maxgeom; ++i) {
    mjv_initGeom(scn->geoms + scn->ngeom++, mjGEOM_BOX, sizes[i], poses[i],
                 nullptr, rgba[i]);
  }
}

void OnAppCmd(android_app* app, int32_t cmd) {
  auto* state = static_cast<AppState*>(app->userData);
  switch (cmd) {
    case APP_CMD_RESUME:
      state->resumed = true;
      break;
    case APP_CMD_PAUSE:
      state->resumed = false;
      break;
    default:
      break;
  }
}

}  // namespace

void android_main(android_app* app) {
  // Route MuJoCo diagnostics to logcat before the first MuJoCo call.
  mxr_install_error_hooks();
  LOGI("MuJoCoXR starting (MuJoCo %s)", mj_versionString());

  AppState state;
  app->userData = &state;
  app->onAppCmd = OnAppCmd;

  XrShell xr;
  VkContext vk;
  if (!xr.CreateInstance(app)) {
    LOGE("OpenXR instance creation failed");
    return;
  }
  bool ok = vk.Create(&xr) &&
            xr.CreateSession(vk.instance(), vk.physical(), vk.device(),
                             vk.queue_family()) &&
            xr.CreateSwapchains({VK_FORMAT_R8G8B8A8_SRGB,
                                 VK_FORMAT_B8G8R8A8_SRGB,
                                 VK_FORMAT_R8G8B8A8_UNORM}) &&
            xr.CreateActions() && vk.InitRenderTargets(&xr);
  if (!ok) {
    LOGE("XR/Vulkan bring-up failed");
    xr.Destroy();
    return;
  }
  if (xr.passthrough()) {
    vk.SetClearColor(0, 0, 0, 0);  // transparent: camera feed behind
  }

  // Model + scene extraction + renderer. A load failure degrades to a
  // clear-color loop so the failure is visible (and logged) in-headset.
  mjModel* model = mxr_load_model_from_assets(app->activity->assetManager);
  mjData* data = nullptr;
  mjvScene scene;
  mjv_defaultScene(&scene);
  mjvOption vis_opt;
  mjvCamera cam;
  SceneRenderer renderer;
  bool scene_ready = false;
  if (model) {
    data = mj_makeData(model);
    int home = mj_name2id(model, mjOBJ_KEY, "home");
    if (home >= 0) {
      mj_resetDataKeyframe(model, data, home);
    }
    mj_forward(model, data);
    mjv_defaultOption(&vis_opt);  // groups 0-2 visible: collision hidden
    mjv_defaultFreeCamera(model, &cam);
    mjv_makeScene(model, &scene, 1000);
    scene_ready = renderer.Create(&vk, model);
    if (!scene_ready) {
      LOGE("renderer creation failed; clear-color only");
    }
  }
  Teleop teleop;
  bool teleop_ready = scene_ready && teleop.Init(model, data);
  double sim_accum = 0;
  XrTime last_display_time = 0;

  bool exit_loop = false;
  int64_t frame_count = 0;
  while (!app->destroyRequested && !exit_loop) {
    // Drain Android lifecycle events; block when idle (no session).
    int events;
    android_poll_source* source;
    int timeout = (state.resumed || xr.session_running()) ? 0 : 250;
    while (ALooper_pollOnce(timeout, nullptr, &events,
                            reinterpret_cast<void**>(&source)) >= 0) {
      if (source) {
        source->process(app, source);
      }
      if (app->destroyRequested) {
        break;
      }
      timeout = 0;
    }

    XrInputState input;
    xr.PollEvents(&exit_loop, &input);
    if (!xr.session_running()) {
      continue;
    }

    XrFrameState frame_state;
    if (!xr.WaitBeginFrame(&frame_state)) {
      continue;
    }
    xr.SyncInput(frame_state.predictedDisplayTime, &input);

    // Frame dt from predicted display times; first frame steps nothing.
    double dt_frame = 0;
    if (last_display_time != 0) {
      dt_frame = (frame_state.predictedDisplayTime - last_display_time)*1e-9;
      dt_frame = dt_frame < 0 ? 0 : (dt_frame > 0.1 ? 0.1 : dt_frame);
    }
    last_display_time = frame_state.predictedDisplayTime;

    if (teleop_ready) {
      teleop.Update(model, data, input, dt_frame);
      // Accumulator-owed fixed steps; catch-up capped at ~2 frames of work.
      sim_accum += dt_frame;
      const double timestep = model->opt.timestep;
      int owed = static_cast<int>(sim_accum/timestep);
      const int cap = static_cast<int>(2.0*0.014/timestep) + 1;
      if (owed > cap) {
        LOGW("sim deficit: dropping %d steps", owed - cap);
        owed = cap;
        sim_accum = 0;
      } else {
        sim_accum -= owed*timestep;
      }
      for (int s = 0; s < owed; ++s) {
        mj_step(model, data);
      }
    }

    if (++frame_count % 72 == 0 && input.grip_valid) {
      LOGI("grip p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) trig=%.2f "
           "sqz=%.2f",
           input.grip.position.x, input.grip.position.y,
           input.grip.position.z, input.grip.orientation.x,
           input.grip.orientation.y, input.grip.orientation.z,
           input.grip.orientation.w, input.trigger, input.squeeze);
    }

    std::vector<XrCompositionLayerProjectionView> proj_views;
    if (frame_state.shouldRender) {
      std::vector<XrView> views;
      if (xr.LocateViews(frame_state.predictedDisplayTime, &views)) {
        if (scene_ready) {
          // Abstract scene extraction; decor appended after (drawn last).
          mjv_updateScene(model, data, &vis_opt, nullptr, &cam, mjCAT_ALL,
                          &scene);
          AppendAxesGizmo(&scene);
          if (teleop_ready) {
            teleop.AppendMarker(&scene);
          }
        }
        VkCommandBuffer cmd = vk.BeginFrameCommands();
        proj_views.resize(views.size());
        std::vector<uint32_t> image_indices(views.size());
        for (size_t i = 0; i < views.size(); ++i) {
          if (!xr.AcquireSwapchainImage(static_cast<int>(i),
                                        &image_indices[i])) {
            proj_views.clear();
            break;
          }
          const auto& sc = xr.swapchains()[i];
          proj_views[i] = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
          proj_views[i].pose = views[i].pose;
          proj_views[i].fov = views[i].fov;
          proj_views[i].subImage.swapchain = sc.handle;
          proj_views[i].subImage.imageRect = {{0, 0}, {sc.width, sc.height}};

          if (scene_ready) {
            renderer.SetEye(static_cast<int>(i), views[i].pose, views[i].fov);
          }
          vk.BeginEyePass(cmd, static_cast<int>(i), image_indices[i]);
          if (scene_ready) {
            renderer.Draw(cmd, static_cast<int>(i), &scene);
          }
          vk.EndEyePass(cmd);
        }
        if (!proj_views.empty()) {
          vk.SubmitAndWait(cmd);
          for (size_t i = 0; i < views.size(); ++i) {
            xr.ReleaseSwapchainImage(static_cast<int>(i));
          }
        }
      }
    }
    xr.EndFrame(frame_state, proj_views);
  }

  renderer.Destroy();
  mjv_freeScene(&scene);
  if (data) {
    mj_deleteData(data);
  }
  if (model) {
    mj_deleteModel(model);
  }
  vk.Destroy();
  xr.Destroy();
  LOGI("MuJoCoXR exiting");
}
