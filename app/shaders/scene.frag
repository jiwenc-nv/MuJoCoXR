#version 450
// Half-lambert with one hardcoded directional light; procedural checker on
// the floor plane (tex_data is never read — do-not-build list).

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
  vec3 base = pc.color.rgb;
  if (pc.ncol0.w > 0.5) {
    float cx = floor(v_pos_w.x / 0.6);
    float cy = floor(v_pos_w.y / 0.6);
    base *= (mod(cx + cy, 2.0) < 1.0) ? 1.0 : 0.62;
  }
  const float ambient = 0.35;
  out_color = vec4(base * (ambient + (1.0 - ambient) * diff), pc.color.a);
}
