cmake_minimum_required(VERSION 3.16)

project(mujocoxr LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fused multiply-add changes results, and baselines/host-x86_64.txt asserts
# that MuJoCo's dynamics for this model are the SAME on every architecture.
# wasm32 has no f64.fma so clang cannot contract there; AArch64 has FMADD and
# clang defaults to -ffp-contract=on, so the Android build would contract and
# the other two would not. wasm's strictness is the only policy all three can
# implement, so it becomes the policy. Set before MuJoCo is added so the
# engine itself is built with it. Verified 2026-07-28: re-recording the host
# baseline with this flag is bit-identical to the committed reference, so the
# reference is not contraction-dependent and nothing had to be re-recorded.
add_compile_options(-ffp-contract=off)

# Upstream MuJoCo pinned at v3.10.0, consumed unmodified (zero source patches).
# The filament-mjr-compat gate is the only in-tree flag that excludes
# src/render/classic + src/ui, yielding a GL-free libmujoco.so — no desktop
# OpenGL anywhere; this app brings its own Vulkan renderer.
set(MUJOCO_USE_FILAMENT_MJR_COMPAT ON CACHE BOOL "" FORCE)
set(MUJOCO_USE_FILAMENT OFF CACHE BOOL "" FORCE)
set(MUJOCO_ENABLE_AVX OFF CACHE BOOL "" FORCE)
set(MUJOCO_ENABLE_AVX_INTRINSICS OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)

# These MUST be forced before FetchContent_MakeAvailable: upstream declares
# them with option(), and option() does not override an existing cache entry,
# so setting them afterwards silently does nothing.
#
# Threads OFF is not a limitation. mjCModel::TryCompile wraps mesh
# compilation in a ScopedDisableThreading under __EMSCRIPTEN__ regardless of
# this flag, so it is always serial anyway (measured: 0.34 s serial on host)
# — and turning it off deletes the SharedArrayBuffer requirement, which
# deletes the COOP/COEP headers, which is the difference between `python3 -m
# http.server` and a custom server.
if(EMSCRIPTEN)
  set(MUJOCO_WASM_THREADS OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_TESTS_WASM OFF CACHE BOOL "" FORCE)
  set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL "" FORCE)
endif()

# Point MUJOCOXR_MUJOCO_DIR at an existing MuJoCo v3.10.0 checkout to skip
# the download; the default fetches the pinned upstream release.
set(MUJOCOXR_MUJOCO_DIR "" CACHE PATH "Existing MuJoCo v3.10.0 source tree")
include(FetchContent)
if(MUJOCOXR_MUJOCO_DIR)
  add_subdirectory(${MUJOCOXR_MUJOCO_DIR} mujoco)
else()
  FetchContent_Declare(
    mujoco
    URL https://github.com/google-deepmind/mujoco/archive/refs/tags/3.10.0.tar.gz
  )
  FetchContent_MakeAvailable(mujoco)
endif()

# How every first-party target names MuJoCo. On wasm, libmujoco is a static
# archive and obj_decoder/stl_decoder register themselves with MuJoCo's
# resource table from file-scope constructors — so a linker that drops
# unreferenced archive members produces a clean link in which all 67 meshes
# then fail to load at runtime. Same fix upstream's own wasm target uses.
#
# It has to come FIRST in every link list: archive extraction is
# order-sensitive, and a plain libmujoco.a ahead of the --whole-archive copy
# makes wasm-ld extract the same members twice.
if(EMSCRIPTEN)
  set(MXR_MUJOCO -Wl,--whole-archive mujoco -Wl,--no-whole-archive
                 ccd lodepng tinyxml2 qhullstatic_r)
else()
  set(MXR_MUJOCO mujoco)
endif()

# What it costs to consume the src/ header tier on this platform: the include
# path, and on Android bionic's liblog, because src/mxr_log.h expands to
# __android_log_print there and nothing else pulls that library in.
#
# The rule this exists to make mechanical: A TARGET THAT EXPOSES src/ ON ITS
# INCLUDE PATH LINKS mxr_platform. Nothing else has to be known. That fact
# used to be restated at four separate link lines and was wrong at two of
# them in opposite directions — `teleop_replay` had been forgotten (Android
# `--target teleop_replay` failed with `undefined symbol:
# __android_log_print`), and `ik_prototype`'s was then argued to be dead
# because ik_dls.{c,h} and host/ik_prototype.cc contain no logging call at
# all. They do not; it still is not dead. host/ik_prototype.cc:15 includes
# mxr_error.h, and :124 hands mxr_handle_error/mxr_handle_warning to
# mju_user_error — taking their address emits both static inline bodies, and
# both bodies call LOGE/LOGW. Grepping for log calls cannot see that; the
# linker can, and did.
add_library(mxr_platform INTERFACE)
target_include_directories(mxr_platform INTERFACE src)
if(ANDROID)
  target_link_libraries(mxr_platform INTERFACE log)
endif()

# Warnings, on FIRST-PARTY code only — this is an INTERFACE target that every
# target we write links and that fetched MuJoCo does not, so upstream keeps
# its own warning policy and ours cannot be drowned out by it.
#
# -Werror=format is the one promoted to an error, and it is not decoration.
# MxrRobot and IkDls deliberately carry same-named fields of different types:
# `spec->tcp_body` and `spec->gripper_act` are `const char*` names, while
# `ik->tcp_body` and `ik->gripper_act` are the resolved int ids, and both
# pairs appear within a few lines of each other in src/teleop.cc and
# src/ik_dls.c with the string halves passed to %s. That pairing is the right
# design — the name and the id are different things and want the same word —
# but with no -Wformat a swap compiles clean and prints garbage or crashes.
# This makes it a build failure instead. The alternative was renaming the
# table columns to tcp_body_name / gripper_act_name, which was rejected
# because those names are also the table's vocabulary in a dozen comments and
# two error messages ("give the rows distinct tcp_body names").
#
# Verified clean at the time of writing across all three toolchains.
# -Wno-missing-field-initializers is not laziness: `VkImageCreateInfo info{
# VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};` is THE idiom for Vulkan and OpenXR —
# set sType, zero the rest — and -Wextra flags every one of the ~60 of them in
# app/android/. Warnings nobody can act on are how a build stops being read.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mxr_platform INTERFACE
    -Wall -Wextra -Wno-missing-field-initializers -Werror=format)
endif()

# PUBLIC mxr_platform rather than PUBLIC src: this target puts the whole
# header tier on its consumers' include path, so it owes the tier's link
# cost, whether or not ik_dls.c itself logs.
add_library(mxr_ik STATIC src/ik_dls.c src/robot_spec.c)
target_link_libraries(mxr_ik PUBLIC mujoco mxr_platform)

# The portable tier. `src/` is only where these files live — what makes the
# tier real is THIS TARGET'S RESTRICTED SOURCE LIST, compiled in the default
# host configuration on every build. Adding a file to src/ does not join the
# tier; adding it to this list does.
#
# Two mechanisms enforce it, and they cover different things:
#
#   SYMBOLS, unconditionally. teleop_replay links mxr_core into a host
#   executable against libmujoco alone — no GL, no Vulkan, no OpenXR — so a
#   shell symbol reaching into the core fails the link on every machine.
#
#   HEADERS, only for headers this machine does not have. OpenXR and
#   Emscripten headers are fetched/toolchain-supplied and do fail here; but
#   any box that builds the Android shell also has /usr/include/vulkan and
#   /usr/include/GLES3, and the core compiles clean against them. A core
#   file taking a VkFormat or a GLenum in a signature would otherwise sail
#   through, so close that gap explicitly rather than claim the compiler
#   does it.
#
# The SCAN below therefore covers strictly more than the compiled list: every
# src/*.h and every src/*.c, not just the three .cc. A header is where the link
# half is blind — a `typedef VkFormat mxr_format_t` or an EGLDisplay in a
# signature emits no symbol at all, so nothing fails to link, and
# src/mesh_buffers.h is exactly the GPU-adjacent header that would carry one.
# Globbing the directory also means a NEW core file is scanned the day it
# appears rather than the day someone remembers to add it. src/mxr_log.h's
# guarded <android/log.h> is not a false positive: android/ is a platform
# header, not a graphics or runtime API, and is deliberately absent from the
# alternation.
#
# The *.c glob closes a hole rather than tidying: src/ik_dls.c is in neither
# MXR_CORE_SOURCES (it compiles into mxr_ik) nor the *.h glob, so it has never
# been scanned — and src/robot_spec.c would have inherited that exemption on
# the day it was added, along with the whole robot table.
set(MXR_CORE_SOURCES
  src/teleop.cc
  src/sim_scene.cc
  src/mesh_buffers.cc)
file(GLOB MXR_CORE_HEADERS CONFIGURE_DEPENDS
     RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
     ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
foreach(core_src ${MXR_CORE_SOURCES} ${MXR_CORE_HEADERS})
  file(READ ${CMAKE_CURRENT_SOURCE_DIR}/${core_src} core_text)
  if(core_text MATCHES "#[ \t]*include[ \t]*[<\"](GLES|GL/|EGL/|KHR/|vulkan/|openxr/|emscripten/)")
    message(FATAL_ERROR
      "${core_src} includes a graphics/runtime header. src/ is the portable "
      "tier: it is compiled for host, Android and wasm from one source list, "
      "and must not name any shell API. Put it in app/<target>/ instead.")
  endif()
endforeach()
# Editing a scanned file does not re-run configure on its own, so the check
# would pass on a stale read until something else re-configured.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${MXR_CORE_SOURCES} ${MXR_CORE_HEADERS})

add_library(mxr_core STATIC ${MXR_CORE_SOURCES})
target_link_libraries(mxr_core PUBLIC mujoco mxr_ik mxr_platform)

# Host IK prototype and the invariant-baseline recorder. Plain CLI binaries;
# the baseline recorder cross-compiles unchanged for the on-device check.
add_executable(ik_prototype host/ik_prototype.cc)
target_link_libraries(ik_prototype PRIVATE mxr_ik)

# mxr_platform, not mxr_core: this one includes src/mxr_error.h (and through
# it mxr_log.h) but links none of the core's objects.
add_executable(baseline bench/baseline.cc)
target_link_libraries(baseline PRIVATE ${MXR_MUJOCO} mxr_platform)

# The core's headless consumer, and the reason the layering above is
# enforceable rather than aspirational. Records/checks a golden teleop trace
# with the same --ref idiom as `baseline`.
add_executable(teleop_replay bench/teleop_replay.cc)
target_link_libraries(teleop_replay PRIVATE ${MXR_MUJOCO} mxr_core)

# CLI benchmark: verbatim copy of upstream MuJoCo's sample/testspeed.cc
# (MUJOCO_BUILD_EXAMPLES stays OFF; the upstream sample tree is never built).
# Threads are unavailable in the web build by decision, above.
if(NOT EMSCRIPTEN)
  find_package(Threads REQUIRED)
  add_executable(testspeed bench/testspeed.cc)
  target_link_libraries(testspeed PRIVATE mujoco Threads::Threads)
endif()

# The staged scenes both teleop_replay and the web build load: each
# ar_scene.xml lives in assets/<id>/ and its robot in third_party/, and
# mj_loadXML resolves `<include file="panda.xml"/>` and meshdir="assets"
# relative to the XML, so they have to be co-located. The assets/
# subdirectory is PRESERVED, not flattened to basenames like the APK does —
# the APK gets away with that only because mjVFS keys on basenames, and a
# real filesystem does not.
#
# THE SCENE ID IS THE ONLY NAME. `<id>` is simultaneously the staging
# directory below, the APK asset directory, the wasm MEMFS mount point, the
# web `?scene=` token and the baselines/teleop-<id>-*.txt stem. It is
# declared in exactly two places — mxr_stage_scene() here and MxrScene::id in
# src/robot_spec.c — and SRC/ROBOT_SPEC.C IS AUTHORITATIVE: this file exists
# to put the bytes where that file says they are. They cannot be checked
# against each other at build time, so a disagreement surfaces at load as
# mj_loadXML failing on a named path (mxr_last_error / logcat), which is the
# reason the id travels as a string rather than an index.
set(MXR_MENAGERIE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/menagerie)
set(MXR_STAGE_ROOT ${CMAKE_CURRENT_BINARY_DIR})
set(MXR_SCENE_IDS)

# mxr_stage_scene(<id> <menagerie_dir> <file>...)
#
# The file list is explicit rather than a directory copy, because everything
# staged here is also everything the browser downloads. Menagerie ships a
# 2.2 MB panda.png screenshot and four mjx_* variants that the Franka scene
# never references; copying the directory wholesale put all of it on the wire
# and cost 2.2 MB gzipped for a file nothing reads. LICENSE is not optional —
# the models are Apache-2.0 and the NOTICE ships with them.
#
# At least one file is required, and the FIRST must be the robot XML the
# wrapper <include>s: it doubles as the fetch probe below.
function(mxr_stage_scene id menagerie_dir)
  set(dest ${MXR_STAGE_ROOT}/${id})
  set(wrapper ${CMAKE_CURRENT_SOURCE_DIR}/assets/${id}/ar_scene.xml)
  # file(COPY) runs at CONFIGURE time and nothing re-triggers configure on its
  # own, so without this line editing a wrapper leaves a stale copy in the
  # build tree and both teleop_replay and the browser silently keep testing
  # the old scene. Same shape as the POST_BUILD trap on shell.js below. Only
  # the first-party file is listed: the Menagerie tree is fetched, not edited.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${wrapper})
  # Probe a staged FILE, not the directory. An interrupted or partial
  # sparse-checkout leaves third_party/menagerie/<dir> present but short of its
  # contents, and file(COPY) on a missing source is a hard CMake error — so the
  # directory test turned the friendly "run scripts/fetch-menagerie.sh" message
  # below into a stack of CMake errors in exactly the case it was written for.
  list(GET ARGN 0 probe)
  if(EXISTS ${MXR_MENAGERIE}/${menagerie_dir}/${probe})
    file(COPY ${MXR_MENAGERIE}/${menagerie_dir}/assets DESTINATION ${dest})
    set(staged)
    foreach(f ${ARGN})
      list(APPEND staged ${MXR_MENAGERIE}/${menagerie_dir}/${f})
    endforeach()
    file(COPY ${staged} ${wrapper} DESTINATION ${dest})
    set(MXR_SCENE_IDS ${MXR_SCENE_IDS} ${id} PARENT_SCOPE)
  else()
    message(STATUS
      "${id} scene not fetched (${MXR_MENAGERIE}/${menagerie_dir}/${probe} is "
      "absent) — run scripts/fetch-menagerie.sh, then re-run cmake. "
      "teleop_replay and the web asset bundle need it.")
  endif()
endfunction()

# hand.xml is NOT here, and its removal is the reason to read this comment
# rather than copy the shape below. It was staged under the note "<- panda.xml
# includes", which is false: panda.xml contains no <include> at all (hand.xml
# is a standalone gripper-only variant, a sibling of panda_nohand.xml). It was
# reachable from nothing, and app/android/package-apk.sh never staged it — the
# APK has always loaded without it. So it was 4 KB on the wire serving a
# comment. Verified by the Franka trace gate, which is unmoved by dropping it.
mxr_stage_scene(franka franka_emika_panda
  panda.xml    # <- assets/franka/ar_scene.xml includes
  scene.xml    # <- the `baseline` gate loads this one, not ar_scene.xml
  LICENSE)

# No scene.xml: `baseline` refuses a model with nu < 8 (bench/baseline.cc), so
# nothing in the tree loads the SO101's, and everything staged is downloaded.
mxr_stage_scene(so101 robotstudio_so101
  so101.xml    # <- assets/so101/ar_scene.xml includes
  LICENSE)

# Everything above is the common prefix and builds in EVERY configuration —
# deliberately not an else-branch, so nobody can turn this into
# if/elseif/else and quietly make the portable core Android-or-web-only.
if(NOT ANDROID AND NOT EMSCRIPTEN)
  # A plain `cmake -S . -B build-web` with no emcc exits 0, produces no web
  # artifacts and says nothing, which is the worst onboarding failure
  # available. Name both cross-builds from the one everybody runs first.
  message(STATUS "MuJoCoXR: host tools only. For the headset app:")
  message(STATUS "  cmake -S . -B build-android "
                 "-DCMAKE_TOOLCHAIN_FILE=$ENV{ANDROID_NDK}/build/cmake/"
                 "android.toolchain.cmake -DANDROID_ABI=arm64-v8a ...")
  message(STATUS "  emcmake cmake -S . -B build-web "
                 "-DCMAKE_BUILD_TYPE=Release   (for the browser)")
endif()

if(EMSCRIPTEN)
  # ---- wasm build: CLI tools under node, and the WebXR shell ----
  #
  # Build NAMED TARGETS ONLY, never `all`: upstream's add_subdirectory(wasm)
  # is unconditional under EMSCRIPTEN and points CMAKE_RUNTIME_OUTPUT_DIRECTORY
  # at ${CMAKE_SOURCE_DIR}/wasm/dist — which, under FetchContent, is a
  # `wasm/dist` inside THIS repo.
  set(MXR_WASM_LINK_OPTIONS
      -fexceptions
      # emcc defaults the stack to 64 KB. qhull over 396,791 mesh vertices
      # overruns it, and the result is corruption rather than an error.
      -sSTACK_SIZE=4MB
      -sINITIAL_MEMORY=256MB
      -sALLOW_MEMORY_GROWTH=1)

  # Every staged scene is preloaded at its own id, so one wasm binary runs
  # either gate: `node teleop_replay.js /franka/ar_scene.xml` and
  # `/so101/ar_scene.xml` are the same command with a different argument.
  #
  # THE COST IS PAID THREE TIMES, not once: this list goes to `mxr`,
  # `baseline` and `teleop_replay`, and each of the latter two loads exactly
  # one scene per run. For the gate binaries that is only build-tree bytes on
  # a developer machine — node reads the .data locally — so it buys the
  # one-binary-per-target property cheaply. For `mxr` it is bytes on the wire
  # into a headset browser, ~33 -> ~52 MB, and THAT is the one worth fixing;
  # the per-scene fallback is deferred, deliberately, and noted below.
  set(MXR_WASM_PRELOAD)
  foreach(id ${MXR_SCENE_IDS})
    list(APPEND MXR_WASM_PRELOAD
         "SHELL:--preload-file ${MXR_STAGE_ROOT}/${id}@/${id}")
  endforeach()

  # `scene` is <id>/<file>: the ONE scene whose absence should stop the build
  # rather than surface as a runtime open() failure inside node.
  function(mxr_wasm_cli target scene)
    target_link_options(${target} PRIVATE ${MXR_WASM_LINK_OPTIONS}
      -sEXIT_RUNTIME=1  # so main()'s return code reaches node
      ${MXR_WASM_PRELOAD}
      # The references travel with the binary, so the gate is one command
      # with no host-filesystem access and no NODERAWFS.
      "SHELL:--preload-file ${CMAKE_CURRENT_SOURCE_DIR}/baselines@/baselines")
    if(NOT EXISTS ${MXR_STAGE_ROOT}/${scene})
      message(FATAL_ERROR
        "${target} needs the staged scene at ${MXR_STAGE_ROOT}/${scene} — run "
        "scripts/fetch-menagerie.sh and re-run cmake.")
    endif()
  endfunction()

  mxr_wasm_cli(baseline franka/scene.xml)
  mxr_wasm_cli(teleop_replay franka/ar_scene.xml)

  # ---- the WebXR app ----
  # GLSL travels as a C string literal rather than a fetch: the shaders must
  # arrive with the wasm module they were compiled against, and a second
  # network request is a second thing that can 404 in a headset browser.
  # (The Android side compiles the same sources to SPIR-V instead; neither
  # target ships GLSL as a separate file.)
  set(MXR_WEB_SHADER_INCS)
  foreach(shader scene.vert scene.frag)
    set(shader_out ${CMAKE_CURRENT_BINARY_DIR}/shaders/${shader}.inc)
    add_custom_command(
      OUTPUT ${shader_out}
      COMMAND ${CMAKE_COMMAND} -E make_directory
              ${CMAKE_CURRENT_BINARY_DIR}/shaders
      COMMAND ${CMAKE_COMMAND}
              -DIN=${CMAKE_CURRENT_SOURCE_DIR}/app/web/shaders/${shader}
              -DOUT=${shader_out}
              -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/embed_shader.cmake
      DEPENDS app/web/shaders/${shader} cmake/embed_shader.cmake)
    list(APPEND MXR_WEB_SHADER_INCS ${shader_out})
  endforeach()
  add_custom_target(mxr_web_shaders DEPENDS ${MXR_WEB_SHADER_INCS})

  add_executable(mxr app/web/main.cc app/web/scene_renderer.cc)
  add_dependencies(mxr mxr_web_shaders)
  target_include_directories(mxr PRIVATE app/web ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(mxr PRIVATE ${MXR_MUJOCO} mxr_core)
  target_link_options(mxr PRIVATE ${MXR_WASM_LINK_OPTIONS}
    # No EXIT_RUNTIME: main() returns immediately and the browser owns the
    # loop, so the runtime has to stay alive for the ABI calls.
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createMxr
    -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2
    # cwrap builds the call thunks; GL is how shell.js registers WebXR's
    # opaque framebuffer into Emscripten's object table. Exports are declared
    # by EMSCRIPTEN_KEEPALIVE at each definition, never listed here.
    "SHELL:-sEXPORTED_RUNTIME_METHODS=['cwrap','GL','HEAPF32','HEAP32']"
    # Every scene, because the page cannot know which one will be picked. The
    # cost is the whole catalogue on the wire before the menu is drawn (33 MB
    # -> 51 MB for the second robot); a named per-scene fallback is ~8 lines
    # of `foreach` here plus a fetch in shell.js, deferred until someone
    # measures a load on a headset browser.
    ${MXR_WASM_PRELOAD})
  set_target_properties(mxr PROPERTIES OUTPUT_NAME mxr SUFFIX ".js")

  # The page and the shell sit beside the generated mxr.js so the whole web
  # target is one directory to serve. A POST_BUILD copy would be wrong here:
  # editing shell.js does not relink mxr, so the copy would never run and you
  # would debug a stale shell.
  add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/index.html
           ${CMAKE_CURRENT_BINARY_DIR}/shell.js
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/app/web/index.html
            ${CMAKE_CURRENT_SOURCE_DIR}/app/web/shell.js
            ${CMAKE_CURRENT_BINARY_DIR}
    DEPENDS app/web/index.html app/web/shell.js)
  add_custom_target(mxr_web_page
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/index.html
            ${CMAKE_CURRENT_BINARY_DIR}/shell.js)
  add_dependencies(mxr mxr_web_page)
endif()

if(ANDROID)
  # No per-target `log` lines here. liblog rides on mxr_platform, which
  # mxr_ik and mxr_core each carry PUBLIC and `baseline` links directly, so
  # every binary that can see src/mxr_log.h already has it and `testspeed` —
  # a verbatim upstream sample that includes none of our headers — correctly
  # does not. Verified with readelf on all five arm64 binaries. See
  # mxr_platform above for why that is one declaration instead of four.

  # ---- MuJoCoXR app: NativeActivity + raw OpenXR + Vulkan ----
  FetchContent_Declare(
    openxr
    URL https://github.com/KhronosGroup/OpenXR-SDK/archive/refs/tags/release-1.1.38.tar.gz
  )
  set(BUILD_API_LAYERS OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(openxr)

  add_library(native_app_glue STATIC
    ${CMAKE_ANDROID_NDK}/sources/android/native_app_glue/android_native_app_glue.c)
  target_include_directories(native_app_glue PUBLIC
    ${CMAKE_ANDROID_NDK}/sources/android/native_app_glue)

  # GLSL -> SPIR-V headers (host glslangValidator; apt: glslang-tools).
  find_program(GLSLANG_VALIDATOR glslangValidator REQUIRED)
  set(MXR_SHADER_HEADERS)
  foreach(shader scene.vert scene.frag)
    string(REPLACE "." "_" shader_var ${shader})
    set(shader_out ${CMAKE_CURRENT_BINARY_DIR}/shaders/${shader}.spv.h)
    add_custom_command(
      OUTPUT ${shader_out}
      COMMAND ${CMAKE_COMMAND} -E make_directory
              ${CMAKE_CURRENT_BINARY_DIR}/shaders
      COMMAND ${GLSLANG_VALIDATOR} -V --vn ${shader_var}_spv -o ${shader_out}
              ${CMAKE_CURRENT_SOURCE_DIR}/app/android/shaders/${shader}
      DEPENDS app/android/shaders/${shader})
    list(APPEND MXR_SHADER_HEADERS ${shader_out})
  endforeach()
  add_custom_target(mxr_shaders DEPENDS ${MXR_SHADER_HEADERS})

  # Shell only: everything portable arrives via mxr_core, which is the same
  # target the host build compiles and links, so this list staying short is
  # the visible measure of how much of the app is shared.
  add_library(mujocoxr SHARED
    app/android/main.cc
    app/android/xr_shell.cc
    app/android/vk_context.cc
    app/android/assets.cc
    app/android/scene_renderer.cc)
  add_dependencies(mujocoxr mxr_shaders)
  target_include_directories(mujocoxr PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  # `log` is absent on purpose even though app/android/*.cc call LOGI
  # directly: mxr_core carries mxr_platform PUBLIC, so it arrives with the
  # header. `android` is a different library and is genuinely needed here.
  target_link_libraries(mujocoxr PRIVATE
    mxr_core openxr_loader native_app_glue vulkan android)
  # NativeActivity resolves ANativeActivity_onCreate from the app glue.
  set_property(TARGET mujocoxr APPEND_STRING PROPERTY
    LINK_FLAGS " -u ANativeActivity_onCreate")
endif()
