// MuJoCoXR NativeActivity entry point: the OpenXR + Vulkan half of the app,
// and nothing else. One xrWaitFrame-paced loop (single-threaded by
// decision): drain lifecycle events -> poll/sync input into an InputState ->
// SimScene::Advance -> if the runtime will present this frame,
// SimScene::Compose and two eye passes.
//
// What the robot does lives in src/sim_scene.cc, shared with the WebXR
// shell. The only physics-adjacent decision left here is the one the core
// cannot see: predictedDisplayTime is this shell's latched clock, and the
// nanosecond -> second conversion happens exactly once, below.

#include <android_native_app_glue.h>

#include <vector>

#include <mujoco/mujoco.h>

#include "assets.h"
#include "mxr_error.h"
#include "mxr_log.h"
#include "scene_renderer.h"
#include "sim_scene.h"
#include "vk_context.h"
#include "xr_shell.h"

namespace {

struct AppState {
  bool resumed = false;
};

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
  SceneRenderer renderer;
  SimScene sim;
  bool scene_ready = false;
  if (model) {
    data = mj_makeData(model);
    scene_ready = renderer.Create(&vk, model);
    if (scene_ready) {
      // TODO: physics is gated on renderer creation, so a Vulkan failure
      // stops the simulation as well as the picture. Pre-existing (this was
      // `teleop_ready = scene_ready && teleop.Init(...)`); preserved
      // verbatim rather than fixed inside a refactor.
      sim.Init(model, data, "XrFrameState::predictedDisplayTime");
    } else {
      LOGE("renderer creation failed; clear-color only");
    }
  }
  // Absolute display time in nanoseconds, converted to seconds exactly once,
  // right here. The int64 epoch is subtracted BEFORE the conversion:
  // predictedDisplayTime is ~1e18 ns, and a*1e-9 - b*1e-9 is not the same
  // double as (a-b)*1e-9. This makes the dt the core derives accurate, not
  // exact — (t3-e)*1e-9 - (t2-e)*1e-9 is still a difference of two rounded
  // doubles, ~1e-13 s once a session has been up for hours. That is 7 orders
  // below one 72 Hz frame and does not accumulate, because the accumulator
  // keeps its sub-timestep residual.
  XrTime time_epoch = 0;

  bool exit_loop = false;
  bool session_was_running = false;
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

    InputState input;
    xr.PollEvents(&exit_loop, &input);
    if (!xr.session_running()) {
      // The session-end transition, which the core owns. Without this the
      // loop merely `continue`d, so an HMD sleep/wake mid-clutch resumed
      // with engaged_ still true over a p_c0_ from before the sleep, and
      // with last_display_s_ from before it too — one clamped 0.1 s dt and
      // a `sim deficit` line on the first resumed frame. session_running_
      // is cleared only on STOPPING / EXITING / LOSS_PENDING
      // (xr_shell.cc:475-486), never by VISIBLE/SYNCHRONIZED, so this edge
      // fires once per real session end and not on focus changes.
      if (session_was_running) {
        session_was_running = false;
        sim.EndSession();
      }
      continue;
    }
    session_was_running = true;

    XrFrameState frame_state;
    if (!xr.WaitBeginFrame(&frame_state)) {
      continue;
    }
    xr.SyncInput(frame_state.predictedDisplayTime, &input);

    if (time_epoch == 0) {
      time_epoch = frame_state.predictedDisplayTime;
    }
    sim.Advance(input,
                (frame_state.predictedDisplayTime - time_epoch)*1e-9);

    if (++frame_count % 72 == 0 && input.grip_valid) {
      LOGI("grip p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) trig=%.2f "
           "sqz=%.2f",
           input.grip_pos[0], input.grip_pos[1], input.grip_pos[2],
           input.grip_quat[0], input.grip_quat[1], input.grip_quat[2],
           input.grip_quat[3], input.trigger, input.squeeze);
    }

    std::vector<XrCompositionLayerProjectionView> proj_views;
    if (frame_state.shouldRender) {
      std::vector<XrView> views;
      if (xr.LocateViews(frame_state.predictedDisplayTime, &views)) {
        const mjvScene* scene = scene_ready ? sim.Compose() : nullptr;
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
          if (scene) {
            renderer.Draw(cmd, static_cast<int>(i), scene);
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
  sim.Destroy();
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
