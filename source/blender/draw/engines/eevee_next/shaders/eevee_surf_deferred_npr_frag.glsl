/* SPDX-FileCopyrightText: 2022-2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * NPR Evaluation.
 */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_hair_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_ambient_occlusion_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

#pragma BLENDER_REQUIRE(eevee_deferred_combine_lib.glsl)

#define TEX_HANDLE_NULL 0u
#define TEX_HANDLE_COMBINED_COLOR 1u
#define TEX_HANDLE_DIFFUSE_COLOR 2u
#define TEX_HANDLE_DIFFUSE_DIRECT 3u
#define TEX_HANDLE_DIFFUSE_INDIRECT 4u
#define TEX_HANDLE_SPECULAR_COLOR 5u
#define TEX_HANDLE_SPECULAR_DIRECT 6u
#define TEX_HANDLE_SPECULAR_INDIRECT 7u
#define TEX_HANDLE_POSITION 8u
#define TEX_HANDLE_NORMAL 9u
#define TEX_HANDLE_BACK_COMBINED_COLOR 10u
#define TEX_HANDLE_BACK_POSITION 11u

vec4 closure_to_rgba(Closure cl)
{
  return vec4(0.0);
}

void npr_input_impl(vec2 texel_offset,
                    inout vec4 combined_color,
                    inout vec4 diffuse_color,
                    inout vec4 diffuse_direct,
                    inout vec4 diffuse_indirect,
                    inout vec4 specular_color,
                    inout vec4 specular_direct,
                    inout vec4 specular_indirect,
                    inout vec3 position,
                    inout vec3 normal)
{
  /*TODO(NPR): Texel-based offset is pretty bad.*/
  ivec2 texel = ivec2(gl_FragCoord.xy + texel_offset);
  texel = clamp(texel, ivec2(0), uniform_buf.film.render_extent);

  float depth = texelFetch(hiz_tx, texel, 0).r;
  vec2 screen_uv = vec2(texel) / uniform_buf.film.render_extent;
  position = drw_point_screen_to_world(vec3(screen_uv, depth));

  if (depth == 1.0) {
    combined_color = vec4(0.0);
    diffuse_color = vec4(0.0);
    diffuse_direct = vec4(0.0);
    diffuse_indirect = vec4(0.0);
    specular_color = vec4(0.0);
    specular_direct = vec4(0.0);
    specular_indirect = vec4(0.0);
    normal = drw_world_incident_vector(position);
  }
  else {
    DeferredCombine dc = deferred_combine(texel);
    deferred_combine_clamp(dc);

    combined_color = deferred_combine_final_output(dc);
    combined_color.a = saturate(1.0 - combined_color.a);
    diffuse_color = vec4(dc.diffuse_color, 1.0);
    diffuse_direct = vec4(dc.diffuse_direct, 1.0);
    diffuse_indirect = vec4(dc.diffuse_indirect, 1.0);
    specular_color = vec4(dc.specular_color, 1.0);
    specular_direct = vec4(dc.specular_direct, 1.0);
    specular_indirect = vec4(dc.specular_indirect, 1.0);
    normal = dc.average_normal;
  }
}

void npr_refraction_impl(out TextureHandle combined_color, out TextureHandle position)
{
  combined_color = TextureHandle(TEX_HANDLE_BACK_COMBINED_COLOR, 0);
  position = TextureHandle(TEX_HANDLE_BACK_POSITION, 0);
}

vec4 TextureHandle_eval(TextureHandle tex, ivec2 texel_offset, int channel_count)
{
  vec4 result = vec4(0.0);

  /*TODO(NPR): Texel-based offset is pretty bad.*/
  ivec2 texel = ivec2(gl_FragCoord.xy + texel_offset);
  texel = clamp(texel, ivec2(0), uniform_buf.film.render_extent);

  switch (tex.type) {
    case TEX_HANDLE_BACK_COMBINED_COLOR: {
      float depth = texelFetch(hiz_back_tx, texel, 0).r;
      if (depth != 1.0) {
        result = texelFetch(radiance_back_tx, texel, 0);
        result.a = saturate(1.0 - result.a);
      }
    } break;
    case TEX_HANDLE_BACK_POSITION: {
      float depth = texelFetch(hiz_back_tx, texel, 0).r;
      vec2 screen_uv = vec2(texel) / uniform_buf.film.render_extent;
      result.rgb = drw_point_screen_to_world(vec3(screen_uv, depth));
      result.a = 1.0;
    } break;
    default:
      break;
  }

  return result;
}

vec4 TextureHandle_eval(TextureHandle tex, int channel_count)
{
  return TextureHandle_eval(tex, ivec2(0), channel_count);
}

void main()
{
  init_globals();

  vec4 result = nodetree_npr();
  out_radiance = vec4(result.rgb * result.a, 0.0);
  out_transmittance = vec4(1.0 - result.a);
}
