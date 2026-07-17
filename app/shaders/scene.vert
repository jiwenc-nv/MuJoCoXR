#version 450
// MuJoCoXR scene shader: one pipeline for MESH / BOX. Geometry is in
// MuJoCo world space; eye.viewproj already folds in stage_T_world and the
// XrView pose/fov (mjvGLCamera is bypassed by design).

layout(location = 0) in vec3 in_pos;
layout(location = 1) in vec3 in_normal;

layout(set = 0, binding = 0) uniform Eye {
  mat4 viewproj;    // P * V * stage_T_world
  vec4 light_dir;   // world-space travel direction of the one light
} eye;

layout(push_constant) uniform PC {
  mat4 model;   // world from geom-local (rotation * scale, translation)
  vec4 ncol0;   // normal-matrix columns
  vec4 ncol1;
  vec4 ncol2;
  vec4 color;
} pc;

layout(location = 0) out vec3 v_normal_w;
layout(location = 1) out vec3 v_pos_w;

void main() {
  vec4 pw = pc.model * vec4(in_pos, 1.0);
  v_pos_w = pw.xyz;
  v_normal_w = mat3(pc.ncol0.xyz, pc.ncol1.xyz, pc.ncol2.xyz) * in_normal;
  gl_Position = eye.viewproj * pw;
}
