
/**
 * G-buffer: Packing and upacking for the of the G-buffer data.
 *
 * See `GPU_SHADER_CREATE_INFO(eevee_surf_deferred)` for a breakdown of the G-buffer layout.
 */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)

vec2 gbuffer_normal_pack(vec3 N)
{
  return N.xy;
}

vec3 gbuffer_normal_unpack(vec2 N_packed)
{
  return vec3(N_packed.xy, sqrt(1.0 - sqr(N_packed.x) - sqr(N_packed.y)));
}

float gbuffer_ior_pack(float ior)
{
  return ior;
}

float gbuffer_ior_unpack(float ior_packed)
{
  return ior_packed;
}

float gbuffer_thickness_pack(float thickness)
{
  return thickness;
}

float gbuffer_thickness_unpack(float thickness_packed)
{
  return thickness_packed;
}

vec3 gbuffer_sss_radii_pack(vec3 sss_radii)
{
  return sss_radii;
}

vec3 gbuffer_sss_radii_unpack(vec3 sss_radii_packed)
{
  return sss_radii_packed;
}

vec4 gbuffer_color_pack(vec3 color)
{
  return vec4(color, 1.0);
}

vec3 gbuffer_color_unpack(vec4 color_packed)
{
  return color_packed.rgb;
}
