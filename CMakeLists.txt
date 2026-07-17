cmake_minimum_required(VERSION 3.16)

project(mujocoxr LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

add_library(mxr_ik STATIC src/ik_dls.c)
target_include_directories(mxr_ik PUBLIC src)
target_link_libraries(mxr_ik PUBLIC mujoco)

# Host IK prototype and the invariant-baseline recorder. Plain CLI binaries;
# the baseline recorder cross-compiles unchanged for the on-device check.
add_executable(ik_prototype host/ik_prototype.cc)
target_link_libraries(ik_prototype PRIVATE mxr_ik)

add_executable(baseline bench/baseline.cc)
target_include_directories(baseline PRIVATE src)
target_link_libraries(baseline PRIVATE mujoco)

# CLI benchmark: verbatim copy of upstream MuJoCo's sample/testspeed.cc
# (MUJOCO_BUILD_EXAMPLES stays OFF; the upstream sample tree is never built).
find_package(Threads REQUIRED)
add_executable(testspeed bench/testspeed.cc)
target_link_libraries(testspeed PRIVATE mujoco Threads::Threads)

if(ANDROID)
  target_link_libraries(ik_prototype PRIVATE log)
  target_link_libraries(baseline PRIVATE log)

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
              ${CMAKE_CURRENT_SOURCE_DIR}/app/shaders/${shader}
      DEPENDS app/shaders/${shader})
    list(APPEND MXR_SHADER_HEADERS ${shader_out})
  endforeach()
  add_custom_target(mxr_shaders DEPENDS ${MXR_SHADER_HEADERS})

  add_library(mujocoxr SHARED
    app/main.cc
    app/xr_shell.cc
    app/vk_context.cc
    app/assets.cc
    app/scene_renderer.cc
    app/teleop.cc)
  add_dependencies(mujocoxr mxr_shaders)
  target_include_directories(mujocoxr PRIVATE src ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(mujocoxr PRIVATE
    mujoco mxr_ik openxr_loader native_app_glue vulkan android log)
  # NativeActivity resolves ANativeActivity_onCreate from the app glue.
  set_property(TARGET mujocoxr APPEND_STRING PROPERTY
    LINK_FLAGS " -u ANativeActivity_onCreate")
endif()
