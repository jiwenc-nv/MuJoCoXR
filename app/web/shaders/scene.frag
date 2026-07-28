#version 300 es
// Half-lambert with one hardcoded directional light. Alpha passes through
// straight (unpremultiplied): the context is created with
// premultipliedAlpha:false and the framebuffer clears to alpha 0, so the AR
// passthrough shows behind the scene.
//
// Twin of app/android/shaders/scene.frag. Leaf identifiers and the
// arithmetic are preserved; the block members are mechanically rewritten
// (`eye.light_dir` -> `eye_light_dir`, `pc.color` -> `pc_color`) because
// GLSL ES 3.00 has no push constants. `const float ambient = 0.35;` is
// BYTE-IDENTICAL in both files, which is the drift check this duplication
// buys: no C++ code reads it, so hoisting it to src/mesh_buffers.h would
// cost a uniform plus a plumbing path in both renderers to share one float.
precision highp float;

in vec3 v_normal_w;
in vec3 v_pos_w;

uniform vec4 eye_light_dir;
uniform vec4 pc_color;

out vec4 out_color;

void main() {
  vec3 n = normalize(v_normal_w);
  vec3 l = normalize(-eye_light_dir.xyz);
  float diff = max(dot(n, l), 0.0);
  const float ambient = 0.35;
  out_color = vec4(pc_color.rgb * (ambient + (1.0 - ambient) * diff),
                   pc_color.a);
}
