#version 300 es
// MuJoCoXR scene shader: one program for MESH / BOX. Geometry is in
// MuJoCo world space; eye_viewproj already folds in xr_from_mj and the
// XRView projection/transform (mjvGLCamera is bypassed by design).
//
// Twin of app/android/shaders/scene.vert. What is preserved is the LEAF
// identifiers — viewproj, light_dir, model, ncol0..2, color, in_pos,
// in_normal, v_pos_w, v_normal_w — and the arithmetic. What differs, in
// every body line that touches a block member, is `eye.viewproj` ->
// `eye_viewproj` and `pc.model` -> `pc_model`: GLSL ES 3.00 has no push
// constants, and making the bodies literally identical would mean a
// per-draw UBO and ~70 buffer updates a frame. So `diff` over the two files
// shows the mechanical block.member -> block_member rewrite plus the
// layout-qualifier difference, and nothing else — any drift in the actual
// shading is a body line that is not one of those two rewrites.

in vec3 in_pos;
in vec3 in_normal;

uniform mat4 eye_viewproj;   // P * V * xr_from_mj
uniform vec4 eye_light_dir;  // world-space travel direction of the one light

uniform mat4 pc_model;   // world from geom-local (rotation * scale, translation)
uniform vec4 pc_ncol0;   // normal-matrix columns
uniform vec4 pc_ncol1;
uniform vec4 pc_ncol2;
uniform vec4 pc_color;

out vec3 v_normal_w;
out vec3 v_pos_w;

void main() {
  vec4 pw = pc_model * vec4(in_pos, 1.0);
  v_pos_w = pw.xyz;
  v_normal_w = mat3(pc_ncol0.xyz, pc_ncol1.xyz, pc_ncol2.xyz) * in_normal;
  gl_Position = eye_viewproj * pw;
}
