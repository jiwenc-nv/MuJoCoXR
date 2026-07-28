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

# PUBLIC mxr_platform rather than PUBLIC src: this target puts the whole
# header tier on its consumers' include path, so it owes the tier's link
# cost, whether or not ik_dls.c itself logs.
add_library(mxr_ik STATIC src/ik_dls.c)
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
# src/*.h, not just the three .cc. A header is where the link half is blind —
# a `typedef VkFormat mxr_format_t` or an EGLDisplay in a signature emits no
# symbol at all, so nothing fails to link, and src/mesh_buffers.h is exactly
# the GPU-adjacent header that would carry one. Globbing the whole directory
# also means a NEW core header is scanned the day it appears rather than the
# day someone remembers to add it. src/mxr_log.h's guarded <android/log.h> is
# not a false positive: android/ is a platform header, not a graphics or
# runtime API, and is deliberately absent from the alternation.
set(MXR_CORE_SOURCES
  src/teleop.cc
  src/sim_scene.cc
  src/mesh_buffers.cc)
file(GLOB MXR_CORE_HEADERS CONFIGURE_DEPENDS
     RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
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

# The staged scene teleop_replay loads: ar_scene.xml lives in assets/ and
# panda.xml in third_party/, and mj_loadXML resolves
# `<include file="panda.xml"/>` and meshdir="assets" relative to the XML, so
# they have to be co-located. The assets/ subdirectory is PRESERVED, not
# flattened to basenames like the APK does — the APK gets away with that only
# because mjVFS keys on basenames, and a real filesystem does not.
set(MXR_MENAGERIE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/menagerie)
set(MXR_SCENE_DIR ${CMAKE_CURRENT_BINARY_DIR}/franka)
#
# file(COPY) runs at CONFIGURE time and nothing re-triggers configure on its
# own, so without this line editing assets/ar_scene.xml leaves a stale copy
# in the build tree and both teleop_replay and the browser silently keep
# testing the old scene. Same shape as the POST_BUILD trap on shell.js
# below. Only the first-party file is listed: the Menagerie tree is fetched,
# not edited.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
             ${CMAKE_CURRENT_SOURCE_DIR}/assets/ar_scene.xml)
#
# The file list is explicit rather than a directory copy, because everything
# staged here is also everything the browser downloads. Menagerie ships a
# 2.2 MB panda.png screenshot and four mjx_* variants that this scene never
# references; copying the directory wholesale put all of it on the wire and
# cost 2.2 MB gzipped for a file nothing reads. LICENSE is not optional —
# the model is Apache-2.0 and the NOTICE ships with it.
if(EXISTS ${MXR_MENAGERIE}/franka_emika_panda/panda.xml)
  file(COPY ${MXR_MENAGERIE}/franka_emika_panda/assets
       DESTINATION ${MXR_SCENE_DIR})
  file(COPY
       ${MXR_MENAGERIE}/franka_emika_panda/panda.xml   # <- ar_scene includes
       ${MXR_MENAGERIE}/franka_emika_panda/hand.xml    # <- panda.xml includes
       ${MXR_MENAGERIE}/franka_emika_panda/scene.xml   # <- the baseline gate
       ${MXR_MENAGERIE}/franka_emika_panda/LICENSE
       ${CMAKE_CURRENT_SOURCE_DIR}/assets/ar_scene.xml
       DESTINATION ${MXR_SCENE_DIR})
else()
  message(STATUS
    "Franka scene not fetched — run scripts/fetch-menagerie.sh, then "
    "re-run cmake. teleop_replay and the web asset bundle need it.")
endif()

# Everything above is the common prefix and builds in EVERY configuration —
# deliberately not an else-branch, so nobody can turn this into
# if/elseif/else and quietly make the portable core Android-only.
if(NOT ANDROID)
  # A plain `cmake` produces host tools and no app, which is easy to mistake
  # for a failed configure. Name the cross-build from the one everybody runs
  # first.
  message(STATUS "MuJoCoXR: host tools only. For the headset app:")
  message(STATUS "  cmake -S . -B build-android "
                 "-DCMAKE_TOOLCHAIN_FILE=$ENV{ANDROID_NDK}/build/cmake/"
                 "android.toolchain.cmake -DANDROID_ABI=arm64-v8a ...")
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
