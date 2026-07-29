// MuJoCoXR Linux entry point: the OpenXR + Vulkan half of the app against a
// desktop OpenXR runtime — in practice NVIDIA CloudXR, which streams to a
// headset over the network. The name of this directory is the PLATFORM, not
// the runtime: XR_RUNTIME_JSON picks the runtime at run time and the same
// binary runs against Monado or SteamVR.
//
// One xrWaitFrame-paced loop (single-threaded by decision): poll/sync input
// into an InputState -> SimScene::Advance -> if the runtime will present this
// frame, SimScene::Compose and two eye passes. What the robot does lives in
// src/sim_scene.cc, shared with the Android and WebXR shells; the OpenXR and
// Vulkan machinery lives in app/openxr/, shared with Android.
//
// THREE THINGS ARE DIFFERENT HERE AND EACH IS A CONSEQUENCE OF THE PLATFORM,
// not a preference:
//
//   1. ONE SCENE PER PROCESS, selected by --scene. Android cycles scenes on B
//      because it has no other input device and no way to draw a menu; a
//      desktop has a command line, so the in-process scene swap — and the
//      teardown ordering it needs (see app/android/main.cc) — simply does not
//      exist here. Restoring a B-cycle would be re-adding the most delicate
//      code in the app to buy what an argument already buys.
//   2. SIGINT/SIGTERM ARE THE EXIT PATH. Android has app->destroyRequested.
//      Here the runtime is a HOST-LEVEL SINGLETON that another process may own,
//      and a client that dies without xrDestroySession can leave it wedged, so
//      Ctrl-C has to unwind rather than kill.
//   3. THE SCENES ARE REAL FILES. Android reads its APK through a VFS; CMake
//      already stages every scene as a real directory beside this binary, which
//      is the same thing bench/teleop_replay loads.
//
// Usage is in kUsage below — one string, printed by --help, not restated here.

#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "mxr_error.h"
#include "mxr_log.h"
#include "robot_spec.h"
#include "scene_renderer.h"
#include "sim_scene.h"
#include "vk_context.h"
#include "xr_shell.h"

namespace {

const char kUsage[] =
    "usage: mujocoxr [--scene <id>] [--timeout <s>] [--probe]\n"
    "\n"
    "  --scene <id>   scene to load; default is the first in the catalogue.\n"
    "                 An id, never a path — the id is the only name a scene\n"
    "                 has (see src/robot_spec.c). Legal ids are listed below.\n"
    "  --timeout <s>  seconds to wait for a headset before giving up; 0 does\n"
    "                 not wait. Default 120. The wait ends as soon as the\n"
    "                 runtime reports a system — and a streaming runtime\n"
    "                 reports one from a VIRTUAL device before any headset\n"
    "                 connects, so against those this flag never delays\n"
    "                 anything and a successful start is not a connected\n"
    "                 headset.\n"
    "  --probe        bring the instance up, report what the runtime offers\n"
    "                 and which interaction profiles it accepted, then exit\n"
    "                 without creating a session. Needs no headset.\n"
    "  --help, -h     this text.\n"
    "\n"
    "This program never starts the runtime — it is a host-level singleton\n"
    "another process may own. Starting it, and the four distinct environment\n"
    "faults that all report XR_ERROR_RUNTIME_UNAVAILABLE (-51), are in\n"
    "docs/validation-linux.md.\n";

// SIGINT/SIGTERM -> unwind the frame loop. sig_atomic_t and nothing else: the
// handler may not allocate, log or take a lock.
volatile sig_atomic_t g_quit = 0;
void OnSignal(int sig) {
  if (g_quit) {
    // SECOND Ctrl-C: the user has decided the unwind is taking too long, and
    // they are entitled to that — teardown calls vk.WaitIdle() against a
    // runtime that may be streaming over a network, so a visible pause is
    // plausible rather than hypothetical. Restore the default disposition and
    // re-raise, so the process dies the way it would have without a handler
    // and the shell reports the signal rather than a spurious exit code.
    signal(sig, SIG_DFL);
    raise(sig);
    return;
  }
  g_quit = 1;
}

// Scenes are staged by mxr_stage_scene() into the CMake binary directory, and
// this binary is linked into that same directory, so its own location IS the
// scene root and needs no flag. See the note at MXR_STAGE_ROOT in
// CMakeLists.txt for why that holds.
std::string ExeDir() {
  char buf[4096];
  const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0) {
    return ".";
  }
  buf[n] = '\0';
  char* slash = strrchr(buf, '/');
  if (!slash) {
    return ".";
  }
  *slash = '\0';
  return buf;
}

void ListScenes(FILE* out) {
  fprintf(out, "scenes (--scene <id>):\n");
  for (int i = 0; i < mxr_scene_count(); ++i) {
    const MxrScene* s = mxr_scene_at(i);
    fprintf(out, "  %-10s %s\n", s->id, s->label);
  }
}

// atoi() cannot fail, which is the problem: `--timeout abc` becomes 0, i.e.
// "do not wait" — the opposite of what someone typing a timeout wants, and
// silent. Returns false on anything that is not a whole non-negative number.
bool ParseSeconds(const char* text, int* out) {
  char* end = nullptr;
  const long v = strtol(text, &end, 10);
  if (end == text || *end != '\0' || v < 0 || v > 86400) {
    return false;
  }
  *out = static_cast<int>(v);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  // ---- arguments, BEFORE any OpenXR call ------------------------------------
  // A bad scene id must cost nothing and touch no runtime: it is the one error
  // a user hits repeatedly, and making them wait on xrCreateInstance to hear
  // about a typo would be gratuitous.
  const char* scene_id = nullptr;
  int timeout_s = 120;  // long enough to walk over and put the headset on —
                        // chosen by that description, not measured.
  bool probe = false;

  // A flag that takes a value must be REJECTED when the value is missing, not
  // silently reported as an unknown flag: `--scene` with nothing after it used
  // to print "unknown argument '--scene'", which is false and sends you to the
  // wrong half of the usage text.
  auto needs_value = [&](int i) {
    fprintf(stderr, "mujocoxr: %s needs a value\n\n", argv[i]);
    fputs(kUsage, stderr);
    ListScenes(stderr);
  };
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--scene")) {
      if (i + 1 >= argc) { needs_value(i); return 2; }
      scene_id = argv[++i];
    } else if (!strcmp(argv[i], "--timeout")) {
      if (i + 1 >= argc) { needs_value(i); return 2; }
      if (!ParseSeconds(argv[++i], &timeout_s)) {
        fprintf(stderr, "mujocoxr: --timeout wants whole seconds >= 0, got "
                        "'%s'\n", argv[i]);
        return 2;
      }
    } else if (!strcmp(argv[i], "--probe")) {
      probe = true;
    } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
      fputs(kUsage, stdout);
      ListScenes(stdout);
      return 0;
    } else {
      fprintf(stderr, "mujocoxr: unknown argument '%s'\n\n", argv[i]);
      fputs(kUsage, stderr);
      ListScenes(stderr);
      return 2;
    }
  }

  const MxrScene* scene =
      scene_id ? mxr_scene_by_id(scene_id) : mxr_scene_at(0);
  if (!scene) {
    fprintf(stderr, "mujocoxr: no scene with id '%s'\n", scene_id);
    ListScenes(stderr);
    return 2;
  }

  mxr_install_error_hooks();  // before the first MuJoCo call
  LOGI("MuJoCoXR starting (MuJoCo %s)", mj_versionString());

  // ---- resources, and THE ONE TEARDOWN --------------------------------------
  // Declared together and released in exactly one place. THE RELEASE ORDER IS
  // LOAD-BEARING, and a second copy of it is a second chance to get it wrong:
  // this file shipped one for review, on the model-load path, and it had the
  // inverted order that reliably segfaults. The rule was already written down
  // three times in this tree and violated once — so it gets one implementation
  // instead of a fourth restatement.
  //
  // A lambda for the same reason load_scene below is NOT one: call sites. That
  // has one and reads better inline; this has seven.
  XrShell xr;
  VkContext vk;
  SceneRenderer renderer;
  SimScene sim;
  mjModel* model = nullptr;
  mjData* data = nullptr;

  auto shutdown = [&](int rc) {
    renderer.Destroy();
    sim.Destroy();
    if (data) {
      mj_deleteData(data);
    }
    if (model) {
      mj_deleteModel(model);
    }
    // XR BEFORE VULKAN. The XR swapchain images are VkImages the RUNTIME
    // created on the device below, so xrDestroySwapchain and xrDestroySession
    // dereference it — destroying the device first is a use-after-free inside
    // the runtime, measured as a reliable SIGSEGV on the Ctrl-C path. WaitIdle
    // first, because the last frame may still be in flight.
    //
    // Every call here is a no-op on an object that was never created, which is
    // exactly what lets ONE block serve a failure at instance creation and a
    // clean exit after an hour of running. Nothing above this point in main()
    // has created anything, so argument errors return directly.
    vk.WaitIdle();
    xr.Destroy();
    vk.Destroy();
    return rc;
  };

  // ---- signals, BEFORE the only long-blocking call --------------------------
  // Registered here rather than beside the frame loop because CreateInstance
  // below can block for --timeout seconds (120 by default) waiting for a
  // headset. A Ctrl-C in that window used to take the default disposition and
  // kill the process with a live XrInstance against a host-singleton runtime —
  // the exact outcome the header of this file gives as the reason for having
  // handlers at all. XrShell polls the same flag (set_abort_flag) so that
  // installing them here does not instead make Ctrl-C do nothing for 120 s.
  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);
  xr.set_abort_flag(&g_quit);

  // ---- instance -------------------------------------------------------------
  // No platform chain and no platform extension: those two arguments exist for
  // Android alone (app/android/xr_platform.h). --probe does not wait, because
  // its job is to report the absence of a headset rather than sit through it.
  if (!xr.CreateInstance(nullptr, nullptr, XR_API_VERSION_1_1,
                         probe ? 0 : timeout_s)) {
    // THE CLOUDXR-SPECIFIC HALF LIVES HERE, not in app/openxr/. Four distinct
    // faults report XR_ERROR_RUNTIME_UNAVAILABLE, and naming only the first
    // strands anyone whose fault is one of the other three: they fix the
    // manifest, see an identical -51, and have nowhere left to go. But three
    // of the four are facts about one vendor's runtime, and the shared tier
    // must not know that vendor exists — it already printed the one thing a
    // shell cannot recover afterwards, the XR_RUNTIME_JSON the loader used.
    // This is the same call as kUsage's: name the doc, echo what only the
    // process can see, and keep the table in one place.
    if (xr.instance_result() == XR_ERROR_RUNTIME_UNAVAILABLE) {
      const char* rundir = getenv("NV_CXR_RUNTIME_DIR");
      LOGE("If that runtime is CloudXR, four different faults produce this "
           "one code — docs/validation-linux.md tabulates all four:");
      LOGE("  1. the launch script exports an XR_RUNTIME_JSON that does not "
           "exist; the manifest on disk is $HOME/.cloudxr/openxr_cloudxr.json");
      LOGE("  2. IPC socket: NV_CXR_RUNTIME_DIR = %s — it must match the "
           "value the SERVICE was started with",
           rundir ? rundir : "(unset — the client will look in "
                             "~/.cache/ipc_cloudxr)");
      LOGE("  3. the service exited early: it needs libcloudxr.so and "
           "libNvStreamBase.so on LD_LIBRARY_PATH, or it never opens a socket");
      LOGE("  4. client/service version mismatch — the runtime prints it "
           "above. Do NOT set IPC_IGNORE_VERSION=1: it converts that message "
           "into a SIGBUS. Pair the service with its co-built client library.");
    }
    return shutdown(1);
  }

  // Instance-scope actions: this is the whole interaction-profile decision, and
  // it runs with no session and no headset. It is why --probe is worth having.
  if (!xr.CreateActions()) {
    return shutdown(1);
  }

  if (probe) {
    // A VERDICT SENTENCE, not just an exit code: exiting 0 must not read as
    // "ready to run". And the system half is REPORTED, NOT JUDGED — which is
    // why it names the system rather than claiming a headset, since a
    // streaming runtime answers xrGetSystem from a virtual device long before
    // anything is worn. Measured: CloudXR reports `Head Device` with nothing
    // connected, so "system found" alone would put "exit 0 means ready" back
    // in through a different door.
    if (xr.have_system()) {
      LOGI("probe: instance OK, %d interaction profiles accepted, system = "
           "'%s'", xr.profiles_accepted(), xr.system_name());
      LOGI("probe: a streaming runtime reports a system before any headset "
           "connects — this does NOT mean a headset is attached");
    } else {
      LOGI("probe: instance OK, %d interaction profiles accepted, NO SYSTEM "
           "(xrGetSystem: %d)", xr.profiles_accepted(), xr.system_result());
    }
    // What this does NOT tell you: suggestions are app->runtime. A profile
    // accepted here is one the runtime knows how to bind, not one it WILL bind
    // to whatever controller connects. Read `interaction profile (right): ...`
    // from a real session for that.
    return shutdown(0);
  }

  if (!xr.have_system()) {
    return shutdown(1);  // WaitForSystem already said why
  }

  // ---- session, Vulkan, swapchains ------------------------------------------
  if (!(vk.Create(&xr) &&
        xr.CreateSession(vk.instance(), vk.physical(), vk.device(),
                         vk.queue_family()) &&
        xr.CreateSwapchains({VK_FORMAT_R8G8B8A8_SRGB, VK_FORMAT_B8G8R8A8_SRGB,
                             VK_FORMAT_R8G8B8A8_UNORM}) &&
        xr.AttachActions() && vk.InitRenderTargets(&xr))) {
    LOGE("XR/Vulkan bring-up failed");
    return shutdown(1);
  }
  if (xr.passthrough()) {
    vk.SetClearColor(0, 0, 0, 0);  // transparent: camera feed behind
  }

  // ---- the scene ------------------------------------------------------------
  // Straight-line, not a lambda: there is exactly one call site, because there
  // is no scene swap on this platform.
  const std::string path = ExeDir() + "/" + scene->id + "/ar_scene.xml";
  LOGI("scene: %s (%s), from %s", scene->label, scene->id, path.c_str());
  char err[1024] = {0};
  model = mj_loadXML(path.c_str(), nullptr, err, sizeof(err));
  if (!model) {
    // The RESOLVED ABSOLUTE PATH, which is the whole diagnosis: a copied binary
    // finds nothing, and this line is how you see that in one read. It is also
    // the most reachable error path in the program, which is why it is the one
    // that must not be reached through a bespoke teardown.
    LOGE("mj_loadXML(%s) failed: %s", path.c_str(), err);
    return shutdown(1);
  }
  LOGI("model loaded: nq=%ld nv=%ld nu=%ld nmesh=%ld",
       static_cast<long>(model->nq), static_cast<long>(model->nv),
       static_cast<long>(model->nu), static_cast<long>(model->nmesh));

  data = mj_makeData(model);
  const bool scene_ready = renderer.Create(&vk, model);
  if (scene_ready) {
    sim.Init(model, data, "XrFrameState::predictedDisplayTime");
  } else {
    LOGE("renderer creation failed; clear-color only");
  }

  // ---- frame loop -----------------------------------------------------------
  signal(SIGINT, OnSignal);
  signal(SIGTERM, OnSignal);
  LOGI("running — Ctrl-C to exit");

  bool exit_loop = false;
  int64_t frame_count = 0;
  while (!g_quit && !exit_loop) {
    InputState input;
    xr.PollEvents(&exit_loop, &input);
    if (!xr.session_running()) {
      // The session-end transition, which the core owns; see the same block in
      // app/android/main.cc for what goes wrong without it.
      if (xr.TakeSessionEndEdge()) {
        sim.EndSession();
      }
      // Android gets this throttle for free from ALooper_pollOnce(250). A
      // naive port of the loop spins a whole core waiting for READY.
      struct timespec ts = {0, 250*1000*1000};
      nanosleep(&ts, nullptr);
      continue;
    }

    XrFrameState frame_state;
    if (!xr.WaitBeginFrame(&frame_state)) {
      continue;
    }
    xr.SyncInput(frame_state.predictedDisplayTime, &input);
    sim.Advance(input, xr.display_time_s(frame_state.predictedDisplayTime));

    // UNCONDITIONAL, and that is the whole point. Guarding this on
    // grip_valid meant a session with no controller printed NOTHING after
    // "running", and silence is byte-identical to wedged — so the state this
    // was developed in is the state a newcomer lands in, with no way to tell
    // "the runtime has no controller" from "this program has stopped". A
    // running loop must say it is running even when it has nothing to report,
    // and it must say WHY it has nothing.
    if (++frame_count % 72 == 0) {
      if (input.grip_valid) {
        LOGI("grip p=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f) trig=%.2f "
             "sqz=%.2f",
             input.grip_pos[0], input.grip_pos[1], input.grip_pos[2],
             input.grip_quat[0], input.grip_quat[1], input.grip_quat[2],
             input.grip_quat[3], input.trigger, input.squeeze);
      } else {
        LOGI("no grip pose (frame %lld) — the runtime is not locating a "
             "controller. With no headset attached that is expected; with one "
             "attached, read the `interaction profile (...)` lines above: "
             "`NONE bound` means the runtime has no controller, not that the "
             "bindings are wrong.",
             static_cast<long long>(frame_count));
      }
    }

    std::vector<XrCompositionLayerProjectionView> proj_views;
    if (frame_state.shouldRender) {
      std::vector<XrView> views;
      if (xr.LocateViews(frame_state.predictedDisplayTime, &views)) {
        const mjvScene* scn = scene_ready ? sim.Compose() : nullptr;
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
          if (scn) {
            renderer.Draw(cmd, static_cast<int>(i), scn);
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

  if (g_quit) {
    LOGI("interrupted");
  }
  LOGI("MuJoCoXR exiting");
  return shutdown(0);
}
