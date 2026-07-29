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
#include "robot_spec.h"
#include "scene_renderer.h"
#include "sim_scene.h"
#include "vk_context.h"
#include "xr_platform.h"
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
  // Loader init and the Android create-info chain are the whole of what is
  // platform-specific about instance creation; app/android/xr_platform.cc owns
  // both so src/openxr/ can stay platform-free.
  const void* platform_next = mxr_android_xr_init(app);
  // 1.0 and a zero wait keep this exactly the call it always was: one
  // xrGetSystem attempt, no retry. The `have_system()` term is the other half
  // of that — CreateInstance now returns true with no system so the Linux
  // client's --probe can report it, and this restores the old verdict here.
  if (!platform_next ||
      !xr.CreateInstance(platform_next, mxr_android_xr_extension(),
                         XR_API_VERSION_1_0, 0) ||
      !xr.have_system()) {
    LOGE("OpenXR instance creation failed");
    return;
  }
  bool ok = vk.Create(&xr) &&
            xr.CreateSession(vk.instance(), vk.physical(), vk.device(),
                             vk.queue_family()) &&
            xr.CreateSwapchains({VK_FORMAT_R8G8B8A8_SRGB,
                                 VK_FORMAT_B8G8R8A8_SRGB,
                                 VK_FORMAT_R8G8B8A8_UNORM}) &&
            xr.CreateActions() && xr.AttachActions() &&
            vk.InitRenderTargets(&xr);
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
  mjModel* model = nullptr;
  mjData* data = nullptr;
  SceneRenderer renderer;
  SimScene sim;
  bool scene_ready = false;
  int scene_index = 0;

  // THE SCENE SWAP, in process. Used once at startup and again on every B
  // press. There is no in-headset menu and there cannot be one: this stack
  // has no text rendering, so there is nothing to draw a menu WITH. The app
  // starts on the first scene in src/robot_spec.c and B cycles to the next.
  //
  // The teardown order is EndSession -> SimScene::Destroy ->
  // SceneRenderer::Destroy -> mj_deleteData -> mj_deleteModel.
  //
  // BE HONEST ABOUT WHY: as written today, none of the first three
  // dereferences the mjModel, so none of them is a use-after-free waiting to
  // happen. EndSession reaches only Teleop::Disengage(), which is
  // `engaged_ = false`; mjv_freeScene frees the mjvScene's own arrays and
  // never reads the model it was sized from; SceneRenderer::Destroy touches
  // only Vulkan handles. The order is DISCIPLINE, not repair — it keeps
  // "every step runs while the scene it belongs to still exists" true by
  // construction, so that the day EndSession grows a settle step or Destroy
  // grows a log of the final pose, this code is already right. Do not read
  // the order as evidence that a use-after-free was found here.
  //
  // THE ONE STEP THAT IS LOAD-BEARING TODAY is inside SceneRenderer::Destroy,
  // and it is a GPU synchronisation rather than a CPU ordering. See the
  // comment above that function: VkContext::Submit does not wait
  // despite its name, so the previous frame's command buffer may still be
  // executing when B is pressed.
  //
  // Chosen over shipping two launcher icons, which is MEASURED DEAD: two
  // <activity android:name="android.app.NativeActivity"> entries link and
  // install, but they collapse to one ComponentName, so only one is ever
  // resolved. Distinguishing them needs a NativeActivity subclass, which
  // needs Java, which ends android:hasCode="false". The tiebreaker is the
  // failure shape: the launcher approach fails at INSTALL, with no path to
  // any robot, while this one fails only if you press B.
  auto load_scene = [&](int index) {
    // The ONLY range check on the scene index — the B-cycle above wraps
    // through mxr_scene_at rather than carrying its own bound.
    const MxrScene* scene = mxr_scene_at(index);
    if (!scene) {
      LOGE("no scene at index %d", index);
      return;
    }
    sim.EndSession();
    sim.Destroy();
    renderer.Destroy();
    if (data) {
      mj_deleteData(data);
      data = nullptr;
    }
    if (model) {
      mj_deleteModel(model);
      model = nullptr;
    }
    scene_ready = false;

    // The one line that answers "which robot am I on" from `adb logcat`.
    // With no font and no 2D surface anywhere in this tree, recognising the
    // arm by sight is otherwise the only readout, and that assumes the
    // reader already knows both robots.
    LOGI("scene: %s (%s), %d of %d", scene->label, scene->id, index + 1,
         mxr_scene_count());
    model = mxr_load_model_from_assets(app->activity->assetManager, scene->id);
    if (!model) {
      return;  // clear-color loop; assets.cc has already named the directory
    }
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
  };
  load_scene(scene_index);

  // B is a raw level; the edge is derived here. Starts true so a B already
  // held as the session comes up cannot cycle the scene on frame one.
  bool b_prev = true;

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

    InputState input;
    xr.PollEvents(&exit_loop, &input);
    if (!xr.session_running()) {
      // The session-end transition, which the core owns. Without this the
      // loop merely `continue`d, so an HMD sleep/wake mid-clutch resumed
      // with engaged_ still true over a p_c0_ from before the sleep, and
      // with last_display_s_ from before it too — one clamped 0.1 s dt and
      // a `sim deficit` line on the first resumed frame. The edge is latched
      // in XrShell on the STOPPING / EXITING / LOSS_PENDING transitions and
      // never by VISIBLE/SYNCHRONIZED, so it fires once per real session end
      // and not on focus changes.
      if (xr.TakeSessionEndEdge()) {
        sim.EndSession();
      }
      continue;
    }

    XrFrameState frame_state;
    if (!xr.WaitBeginFrame(&frame_state)) {
      continue;
    }
    xr.SyncInput(frame_state.predictedDisplayTime, &input);

    // Scene cycle, before Advance so the new model is the one stepped this
    // frame rather than the old one being stepped and then thrown away.
    const bool b_now = xr.b_down();
    if (b_now && !b_prev) {
      // Wrap by ASKING THE TABLE rather than by `% mxr_scene_count()`: the
      // modulo is undefined on an empty table, which made load_scene's
      // range guard the only one of the two that could not fire. Now there
      // is one bound, mxr_scene_at's documented NULL, and an empty table
      // degrades to that guard's log line instead of a divide.
      scene_index = mxr_scene_at(scene_index + 1) ? scene_index + 1 : 0;
      load_scene(scene_index);
    }
    b_prev = b_now;

    sim.Advance(input, xr.display_time_s(frame_state.predictedDisplayTime));

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
          vk.Submit(cmd);
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
  // XR BEFORE VULKAN, and this order is a fix rather than a preference: it
  // used to be the other way round. The XR swapchain images are VkImages the
  // RUNTIME created on the device below, so xrDestroySwapchain and
  // xrDestroySession dereference that device — destroying it first is a
  // use-after-free inside the runtime. Nothing here ever ran, so nothing ever
  // caught it; the Linux shell ran, and took SIGSEGV on this exact sequence on
  // its first Ctrl-C. WaitIdle first, because the last frame may still be in
  // flight. See app/linux/main.cc for the same three lines.
  vk.WaitIdle();
  xr.Destroy();
  vk.Destroy();
  LOGI("MuJoCoXR exiting");
}
