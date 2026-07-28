// WebXR shell for MuJoCoXR. The only file that touches app/web/abi.h, and
// the only hand-written JavaScript in the repo — no bundler, no npm, no
// package.json, because CMake already fetches and pins every dependency and
// a second dependency manager would have no relationship to the build.
//
// It owns exactly three things the wasm side cannot see: the XR session, the
// gamepad, and the opaque XRWebGLLayer framebuffer. Everything else is one
// call per frame into the C ABI.
//
// Every STRIDE below is queried from wasm; none is hardcoded. Field order
// WITHIN the input block is hardcoded, against the contract written at
// app/web/abi.h — see the IN_* constants.

import createMxr from './mxr.js';

const statusEl = document.getElementById('status');
const buttonEl = document.getElementById('enter');

// Shell lines are tagged '[mujocoxr]'; the C core's mxr_log.h emits
// '[mujocoxr I|W|E]'. One chrome://inspect filter on "mujocoxr" catches
// both halves, and the presence of a level letter tells you which side of
// the ABI a line came from.
function status(msg) {
  statusEl.textContent = msg;
  console.log('[mujocoxr]', msg);
}

// Field indices into the input block. abi.h owns this contract; naming them
// here is the difference between adding a field and silently reinterpreting
// two. There is exactly one writer (writeInputBlock, below) — the previous
// shape split indices 0-6 and 7-8 across two functions with an unnamed seam,
// so inserting grip_linear_velocity[3] would have turned trigger and squeeze
// into velocity components with nothing to catch it.
const IN_POS = 0;      // 3 floats, XR reference space, metres
const IN_QUAT = 3;     // 4 floats, xyzw
const IN_TRIGGER = 7;
const IN_SQUEEZE = 8;

let mod = null;
let abi = null;
let session = null;
let refSpace = null;
let recenterEdge = false;
// Per-session latches, all reset in resetSessionState().
let loggedProfile = false;
let loggedMapping = false;
let loggedFrameError = false;
let sessionLogged = false;
let clockIsRaf = false;
let xrFbName = 0;

function resetSessionState() {
  recenterEdge = false;
  loggedProfile = false;
  loggedMapping = false;
  loggedFrameError = false;
  sessionLogged = false;
  clockIsRaf = false;
}

// Emscripten's GL layer keys objects by integer. WebXR hands us an OPAQUE
// WebGLFramebuffer that was never created through that layer, so it has no
// name and C cannot bind it. Registering it in GL.framebuffers gives it one.
//
// `fb.name` is load-bearing, not decoration: Emscripten's glGetIntegerv
// reads `.name` off whatever object getParameter returns, so without it a C
// readback of GL_FRAMEBUFFER_BINDING reports 0 while the binding is correct
// — which is exactly the check the browser gate makes.
//
// Cached on the object: the layer's framebuffer is stable for the session,
// and allocating a new id per frame would burn one per frame instead of one
// per session.
function framebufferName(fb) {
  if (!fb) {
    return 0;  // inline sessions render to the default framebuffer
  }
  if (fb.mxrName === undefined) {
    const GL = mod.GL;
    fb.mxrName = GL.getNewId(GL.framebuffers);
    fb.name = fb.mxrName;
    GL.framebuffers[fb.mxrName] = fb;
    xrFbName = fb.mxrName;
    console.log('[mujocoxr] registered XR framebuffer as GL id', fb.mxrName);
  }
  return fb.mxrName;
}

// Drops the table's reference to a framebuffer that the UA invalidated when
// the session ended. The integer id itself is NOT reclaimed — GL.getNewId is
// a monotonic counter, so ids are never recycled — but one integer per
// session is not a leak, and a retained dead WebGLFramebuffer is.
function releaseFramebufferName() {
  if (xrFbName) {
    mod.GL.framebuffers[xrFbName] = null;
    xrFbName = 0;
  }
}

function bindAbi(m) {
  const c = (name, ret, args) => m.cwrap(name, ret, args);
  const a = {
    init: c('mxr_init', 'number', []),
    loadModel: c('mxr_load_model', 'number', []),
    lastError: c('mxr_last_error', 'string', []),
    setClockSource: c('mxr_set_clock_source', null, ['string']),
    endSession: c('mxr_end_session', null, []),
    maxViews: c('mxr_max_views', 'number', []),
    inputFloats: c('mxr_input_floats', 'number', []),
    viewFloats: c('mxr_view_floats', 'number', []),
    viewportInts: c('mxr_viewport_ints', 'number', []),
    nearZ: c('mxr_near_z', 'number', []),
    farZ: c('mxr_far_z', 'number', []),
    inputPtr: c('mxr_input_buffer', 'number', []),
    viewPtr: c('mxr_view_buffer', 'number', []),
    viewportPtr: c('mxr_viewport_buffer', 'number', []),
    beginFrame: c('mxr_begin_frame', null,
                  ['number', 'number', 'number', 'number', 'number', 'number']),
    drawView: c('mxr_draw_view', null, ['number']),
  };
  return a;
}

// The shapes and buffer addresses, queried ONCE. Every one of them returns a
// constexpr or the address of a file-scope array, so none can change: growing
// linear memory under ALLOW_MEMORY_GROWTH preserves addresses and only
// detaches the typed-array views. "Query, don't agree" is unaffected — the
// query still happens, it just happens once instead of ~630 times a second.
function queryLayout() {
  return {
    maxViews: abi.maxViews(),
    inputFloats: abi.inputFloats(),
    viewFloats: abi.viewFloats(),
    viewportInts: abi.viewportInts(),
    // >> 2 converts a byte address into an index into a 4-byte-element
    // typed array (HEAPF32 / HEAP32).
    inBase: abi.inputPtr() >> 2,
    viewBase: abi.viewPtr() >> 2,
    vpBase: abi.viewportPtr() >> 2,
  };
}
let layout = null;

// Writes the whole input block and returns only the two values that travel
// as scalar arguments to mxr_begin_frame. Reads the xr-standard gamepad
// defensively: an exception thrown inside a requestAnimationFrame callback
// silently kills the loop after one frame, so nothing here may assume a
// button exists.
function writeInputBlock(frame, input) {
  const out = { valid: false, aDown: false };
  input[IN_TRIGGER] = 0;
  input[IN_SQUEEZE] = 0;
  for (const src of session.inputSources) {
    if (!src.gripSpace) {
      continue;  // gaze, screen and transient-pointer sources have none
    }
    const pose = frame.getPose(src.gripSpace, refSpace);
    if (!pose) {
      continue;
    }
    const p = pose.transform.position;
    const q = pose.transform.orientation;
    input[IN_POS + 0] = p.x; input[IN_POS + 1] = p.y; input[IN_POS + 2] = p.z;
    input[IN_QUAT + 0] = q.x; input[IN_QUAT + 1] = q.y;
    input[IN_QUAT + 2] = q.z; input[IN_QUAT + 3] = q.w;
    out.valid = true;

    const gp = src.gamepad;
    if (!loggedProfile) {
      loggedProfile = true;
      console.log('[mujocoxr] input source: mapping=' +
                  (gp ? gp.mapping : 'none') + ' buttons=' +
                  (gp ? gp.buttons.length : 0) + ' profiles=' +
                  JSON.stringify(src.profiles));
    }
    if (gp && gp.buttons.length > 0) {
      input[IN_TRIGGER] = gp.buttons[0].value;
      if (gp.mapping !== 'xr-standard') {
        // Once per session: this is a per-frame path, and a warning at
        // display refresh rate is a measurable stall with chrome://inspect
        // attached — which is the documented way to read it.
        if (!loggedMapping) {
          loggedMapping = true;
          console.warn('[mujocoxr] gamepad mapping is "' + gp.mapping +
                       '", not xr-standard: only the trigger is bound');
        }
      } else {
        if (gp.buttons.length > 1) {
          input[IN_SQUEEZE] = gp.buttons[1].value;
        }
        if (gp.buttons.length > 4) {
          out.aDown = gp.buttons[4].pressed;
        }
      }
    }
    break;  // first source with a grip pose wins; the Android shell picks
            // the first hand with a located grip pose the same way
  }
  return out;
}

function onFrame(t, frame) {
  session.requestAnimationFrame(onFrame);
  try {
    const pose = frame.getViewerPose(refSpace);
    if (!pose) {
      return;
    }
    const layer = session.renderState.baseLayer;
    // Two counts, deliberately. `nviews` is what the runtime reported and is
    // what mxr_begin_frame is given RAW: abi.h states that the caller clamps
    // nothing, and main.cc's over-capacity warning is the diagnostic for a
    // runtime that reports more views than the arena holds. Clamping here
    // would make that warning unreachable and turn the one degradation the
    // kMaxViews slack exists for into a silent truncation. `nwrite` is the
    // clamp, and it bounds only the writes into the fixed-size arenas.
    const nviews = pose.views.length;
    const nwrite = Math.min(nviews, layout.maxViews);

    // ONE clock per session, latched on the first frame — the first moment
    // an XRFrame exists, and therefore the first moment
    // predictedDisplayTime can be tested for at all. If the UA does not
    // expose it (WebKit does not) the value is `undefined`, `undefined*1e-3`
    // is NaN, and NaN crosses the C ABI as a double with every downstream
    // guard failing open. Fall back to the rAF `t` — a different timebase,
    // which is why it must be latched for the whole session rather than
    // chosen per frame — and NAME the fallback, so the core's stalled-clock
    // diagnostic points at the clock actually being read.
    if (!sessionLogged) {
      sessionLogged = true;
      clockIsRaf = !Number.isFinite(frame.predictedDisplayTime);
      const clockSource = clockIsRaf
          ? 'requestAnimationFrame t (XRFrame.predictedDisplayTime absent)'
          : 'XRFrame.predictedDisplayTime';
      // Only on the fallback. mxr_load_model already named
      // XRFrame.predictedDisplayTime as the intended source, so re-sending
      // it on the happy path would only duplicate the core's `clock = …`
      // line every session. The export exists to CORRECT that name, which is
      // what abi.h says it is for.
      if (clockIsRaf) {
        abi.setClockSource(clockSource);
        console.warn('[mujocoxr] XRFrame.predictedDisplayTime is absent; ' +
                     'falling back to the rAF timestamp');
      }
      const rs = session.renderState;
      // The clip planes read back from the UA (updateRenderState is applied
      // before the first callback, so this is the applied value, not the
      // requested one), and the floor datum: in a local-floor space a
      // standing viewer's head is 1.4-1.9 m up. ~0 means the origin is at
      // head height, which is a whole-scene 1.6 m offset the axes gizmo
      // cannot catch.
      console.log('[mujocoxr] session: clock = ' + clockSource +
                  ', renderState near/far = ' + rs.depthNear + '/' +
                  rs.depthFar + ', viewer y = ' +
                  pose.transform.position.y.toFixed(3) + ' m');
    }

    // HEAPF32/HEAP32 are re-read every frame: ALLOW_MEMORY_GROWTH detaches
    // the old typed arrays when the heap grows, and a cached view would
    // silently start writing nowhere. The addresses in `layout` do survive
    // growth, which is why they are queried once.
    const f32 = mod.HEAPF32;
    const i32 = mod.HEAP32;

    const input = f32.subarray(layout.inBase,
                               layout.inBase + layout.inputFloats);
    const ctl = writeInputBlock(frame, input);

    for (let i = 0; i < nwrite; ++i) {
      const view = pose.views[i];
      const vp = layer.getViewport(view);
      const vbase = layout.viewBase + i*layout.viewFloats;
      f32.set(view.projectionMatrix, vbase);
      // world-to-view: the INVERSE of the view pose. view.transform.matrix
      // here gives a scene that moves with the head.
      f32.set(view.transform.inverse.matrix, vbase + 16);
      const vpbase = layout.vpBase + i*layout.viewportInts;
      i32[vpbase + 0] = vp.x;
      i32[vpbase + 1] = vp.y;
      i32[vpbase + 2] = vp.width;
      i32[vpbase + 3] = vp.height;
    }

    // The tab losing visibility means the pose is stale, not merely late.
    // Reporting the grip invalid drops the clutch through the same path as
    // lost tracking; the accumulator's catch-up cap covers the dt side.
    const visible = session.visibilityState === 'visible';
    const displayMs = clockIsRaf ? t : frame.predictedDisplayTime;
    abi.beginFrame(displayMs*1e-3, nviews, framebufferName(layer.framebuffer),
                   ctl.valid && visible ? 1 : 0, ctl.aDown ? 1 : 0,
                   recenterEdge ? 1 : 0);
    recenterEdge = false;  // latched for exactly one frame
    // nwrite, not nviews: only the views actually written to the arena have
    // a projection and a viewport. mxr_draw_view would ignore the rest
    // anyway (g_nviews is clamped C-side), so this is belt and braces.
    for (let i = 0; i < nwrite; ++i) {
      abi.drawView(i);
    }
  } catch (e) {
    // Once per session. A persistent frame error at 72-90 Hz floods the one
    // console you are trying to read it in, and the status line is not
    // composited during an immersive session anyway.
    if (!loggedFrameError) {
      loggedFrameError = true;
      console.error('[mujocoxr] frame aborted (further frame errors this ' +
                    'session are suppressed):', e);
      status('frame error: ' + e);
    }
  }
}

async function enterXr() {
  const s = await navigator.xr.requestSession('immersive-ar', {
    requiredFeatures: ['local-floor'],
  });
  // Everything below can throw, and until it all succeeds `s` is a live
  // session that only this scope knows about. Assigning the global first
  // would leave a session nobody can end, which the next button press then
  // requests a second session on top of.
  try {
    // No fallback list, deliberately. WebXR falls back local-floor -> local,
    // and `local` is at HEAD HEIGHT at session start — which lifts the whole
    // scene ~1.6 m while leaving handedness perfectly correct, so the axes
    // gizmo still passes. A silent fallback that a gate cannot catch is
    // worse than a hard failure.
    const rs = await s.requestReferenceSpace('local-floor');
    console.log('[mujocoxr] reference space: local-floor');

    // WebXR defaults to 0.1/1000, which clips the gripper at arm's length
    // and wastes most of the depth buffer. Queried from wasm rather than
    // written as literals: src/mesh_buffers.h owns kNearZ/kFarZ for both
    // targets, and on this one nothing else reads them, so literals here
    // would be the only live values and drift would be undetectable.
    const near = abi.nearZ();
    const far = abi.farZ();
    s.updateRenderState({
      baseLayer: new XRWebGLLayer(s, mod.GL.currentContext.GLctx),
      depthNear: near,
      depthFar: far,
    });
    console.log('[mujocoxr] clip planes from wasm: near=' + near +
                ' far=' + far);

    resetSessionState();
    session = s;
    refSpace = rs;
  } catch (e) {
    await s.end();
    throw e;
  }

  session.addEventListener('end', () => {
    abi.endSession();
    releaseFramebufferName();
    resetSessionState();
    session = null;
    refSpace = null;
    status('session ended');
    buttonEl.disabled = false;
  });
  refSpace.addEventListener('reset', () => { recenterEdge = true; });

  buttonEl.disabled = true;
  status('in session — squeeze to clutch, trigger for the gripper, A to reset');
  session.requestAnimationFrame(onFrame);
}

async function main() {
  status('loading wasm…');
  mod = await createMxr();
  abi = bindAbi(mod);
  // The documented way to debug a headset browser is chrome://inspect, and a
  // console with no handle on the module can only read log lines. This makes
  // the ABI pokeable from that console — which is also how the render path
  // is exercised without a headset. DEBUG ONLY: not a public surface, and
  // nothing in the page reads it.
  window.mxr = { module: mod, abi };
  if (abi.init() !== 0) {
    status('init failed: ' + abi.lastError());
    return;
  }

  // Before loadModel, and therefore before ~11 MB of VBO, the IBO, the VAO
  // and the linked program exist. The spec permits makeXRCompatible to
  // resolve "potentially after the context is lost and restored" on a
  // different adapter; there is no webglcontextlost handler, so on a
  // hybrid-GPU machine doing this at session entry would silently discard
  // every GL object and render a blank scene with no error.
  if (navigator.xr) {
    await mod.GL.currentContext.GLctx.makeXRCompatible();
  }

  status('compiling the Franka scene (67 meshes)…');
  if (abi.loadModel() !== 0) {
    status('model load failed: ' + abi.lastError());
    return;
  }
  layout = queryLayout();

  // navigator.xr is undefined on an INSECURE ORIGIN as well as on a browser
  // with no WebXR, and plain http:// from a dev box to a headset is the most
  // likely way this page is first opened. Reporting "no WebXR" there sends
  // the user to install a different browser to fix a URL scheme.
  if (!window.isSecureContext) {
    status('this page is not a secure context, so WebXR is unavailable — ' +
           'serve it over https, or over http from localhost ' +
           '(adb reverse tcp:8000 tcp:8000)');
    return;
  }
  if (!navigator.xr) {
    status('this browser has no WebXR; nothing to enter');
    return;
  }
  const ok = await navigator.xr.isSessionSupported('immersive-ar');
  if (!ok) {
    status('immersive-ar is not supported here (try a headset browser, or ' +
           'the Chrome DevTools WebXR panel)');
    return;
  }
  // Entry is gated on a successful init, so there is no in-session failure
  // state to degrade into — the button simply never becomes available.
  status('ready — press Enter AR');
  buttonEl.disabled = false;
  buttonEl.addEventListener('click', () => {
    enterXr().catch((e) => {
      status('could not enter AR (most likely: the runtime refused the ' +
             'local-floor reference space) — ' + e);
      buttonEl.disabled = false;
    });
  });
}

// main() has three awaits and no caller to catch them: without this a
// rejected createMxr() or a missing cwrap is an unhandled rejection, and the
// status line freezes on "loading wasm…" forever — the one failure the
// error-sink design cannot otherwise report.
main().catch((e) => {
  status('startup failed: ' + e);
  console.error('[mujocoxr] startup failed:', e);
});
