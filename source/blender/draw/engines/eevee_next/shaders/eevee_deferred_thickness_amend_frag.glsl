/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Amend thickness after object rasterization using shadow-maps.
 *
 * Required resources:
 * - atlas_tx
 * - tilemaps_tx
 */

#pragma BLENDER_REQUIRE(draw_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_shadow_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_light_iter_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_thickness_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

/* If using compute, the shader should define its own pixel. */
#if !defined(PIXEL) && defined(GPU_FRAGMENT_SHADER)
#  define PIXEL gl_FragCoord.xy
#endif

void thickness_from_shadow_single(uint l_idx,
                                  const bool is_directional,
                                  vec3 P,
                                  vec3 Ng,
                                  inout float thickness_accum,
                                  inout float weight_accum)
{
  LightData light = light_buf[l_idx];

  if (light.tilemap_index == LIGHT_NO_SHADOW) {
    return;
  }

  LightVector lv = light_vector_get(light, is_directional, P);
  float attenuation = light_attenuation_surface(light, is_directional, true, false, Ng, lv);
  if ((attenuation < LIGHT_ATTENUATION_THRESHOLD)) {
    return;
  }
  /* Weight result by facing ratio to avoid harsh transitions. */
  float weight = saturate(dot(lv.L, -Ng));
  ShadowEvalResult result = shadow_sample(
      is_directional, shadow_atlas_tx, shadow_tilemaps_tx, light, P);

  if (result.light_visibilty == 0.0) {
    /* Flatten the accumulation to avoid weighting the outliers too much. */
    thickness_accum += safe_sqrt(result.occluder_distance) * weight;
    weight_accum += weight;
  }
}

#define THICKNESS_NO_VALUE -1.0
/**
 * Return the apparent thickness of an object behind surface considering all shadow maps
 * available. If no shadow-map has a record of the other side of the surface, this function
 * returns THICKNESS_NO_VALUE.
 */
float thickness_from_shadow(vec3 P, vec3 Ng, float vPz)
{
  float thickness_accum = 0.0;
  float weight_accum = 0.0;

  LIGHT_FOREACH_BEGIN_DIRECTIONAL (light_cull_buf, l_idx) {
    thickness_from_shadow_single(l_idx, true, P, Ng, thickness_accum, weight_accum);
  }
  LIGHT_FOREACH_END

  LIGHT_FOREACH_BEGIN_LOCAL (light_cull_buf, light_zbin_buf, light_tile_buf, PIXEL, vPz, l_idx) {
    thickness_from_shadow_single(l_idx, false, P, Ng, thickness_accum, weight_accum);
  }
  LIGHT_FOREACH_END

  if (weight_accum == 0.0) {
    return THICKNESS_NO_VALUE;
  }

  float thickness = thickness_accum / weight_accum;
  /* Flatten the accumulation to avoid weighting the outliers too much. */
  thickness = square(thickness);

  /* Add a bias because it is usually too small to prevent self shadowing. */
  return thickness;
}

void main()
{
  ivec2 texel = ivec2(gl_FragCoord.xy);

  float depth = texelFetch(hiz_tx, texel, 0).r;

  /* Bias the shading point position because of depth buffer precision.
   * Constant is taken from https://www.terathon.com/gdc07_lengyel.pdf. */
  const float bias = 2.4e-7;
  depth -= bias;

  vec3 P = drw_point_screen_to_world(vec3(uvcoordsvar.xy, depth));
  float vPz = dot(drw_view_forward(), P) - dot(drw_view_forward(), drw_view_position());

  vec3 Ng = gbuffer_normal_unpack(imageLoad(gbuf_normal_img, ivec3(texel, 0)).rg);

  float shadow_thickness = thickness_from_shadow(P, Ng, vPz);

  /* Use manual fetch because gbuffer_read_thickness expect a read only texture input. */
  uint header = texelFetch(gbuf_header_tx, texel, 0).r;
  int data_layer = gbuffer_normal_count(header);
  vec2 data_packed = imageLoad(gbuf_normal_img, ivec3(texel, data_layer)).rg;
  float thickness = gbuffer_thickness_unpack(data_packed.x);

  if (shadow_thickness == THICKNESS_NO_VALUE) {
    return;
  }

  float thickness_modified = sign(thickness) * min(shadow_thickness, abs(thickness));
  if (thickness_modified == thickness) {
    return;
  }

  data_packed.x = thickness_modified;
  imageStore(gbuf_normal_img, ivec3(texel, data_layer), vec4(data_packed, 0.0, 0.0));
}
