#version 450
// Half-lambert with one hardcoded directional light. Alpha passes through
// straight (unpremultiplied): the background clears to alpha 0 so the AR
// passthrough shows behind the scene.

layout(location = 0) in vec3 v_normal_w;
layout(location = 1) in vec3 v_pos_w;

layout(set = 0, binding = 0) uniform Eye {
  mat4 viewproj;
  vec4 light_dir;
} eye;

layout(push_constant) uniform PC {
  mat4 model;
  vec4 ncol0;
  vec4 ncol1;
  vec4 ncol2;
  vec4 color;
} pc;

layout(location = 0) out vec4 out_color;

void main() {
  vec3 n = normalize(v_normal_w);
  vec3 l = normalize(-eye.light_dir.xyz);
  float diff = max(dot(n, l), 0.0);
  const float ambient = 0.35;
  out_color = vec4(pc.color.rgb * (ambient + (1.0 - ambient) * diff),
                   pc.color.a);
}
