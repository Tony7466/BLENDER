/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Process in screen space the diffuse radiance input to mimic subsurface transmission.
 *
 * This implementation follows the technique described in the siggraph presentation:
 * "Efficient screen space subsurface scattering Siggraph 2018"
 * by Evgenii Golubev
 *
 * But, instead of having all the precomputed weights for all three color primaries,
 * we precompute a weight profile texture to be able to support per pixel AND per channel radius.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_rotation_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_geom_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

void main(void)
{
  ivec2 texel = ivec2(gl_GlobalInvocationID.xy);
  vec2 center_uv = (vec2(texel) + 0.5) / vec2(textureSize(gbuf_header_tx, 0));

  float depth = texelFetch(hiz_tx, texel, 0).r;
  vec3 vP = get_view_space_from_depth(center_uv, depth);

  GBufferData gbuf = gbuffer_read(gbuf_header_tx, gbuf_closure_tx, gbuf_color_tx, texel);

  if (gbuf.diffuse.sss_id == 0u) {
    return;
  }

  float max_radius = max_v3(gbuf.diffuse.sss_radius);

  float homcoord = ProjectionMatrix[2][3] * vP.z + ProjectionMatrix[3][3];
  vec2 sample_scale = vec2(ProjectionMatrix[0][0], ProjectionMatrix[1][1]) *
                      (0.5 * max_radius / homcoord);

  float pixel_footprint = sample_scale.x * float(textureSize(gbuf_header_tx, 0).x);
  if (pixel_footprint <= 1.0) {
    return;
  }

  /* Avoid too small radii that have float imprecision. */
  vec3 clamped_sss_radius = max(vec3(1e-4), gbuf.diffuse.sss_radius / max_radius) * max_radius;
  /* Scale albedo because we can have HDR value caused by BSDF sampling. */
  vec3 albedo = gbuf.diffuse.color / max(1e-6, max_v3(gbuf.diffuse.color));
  vec3 d = burley_setup(clamped_sss_radius, albedo);

  /* Do not rotate too much to avoid too much cache misses. */
  float golden_angle = M_PI * (3.0 - sqrt(5.0));
  float theta = interlieved_gradient_noise(vec2(texel), 0, 0.0) * golden_angle;

  mat2 sample_space = from_scale(sample_scale) * from_rotation(Angle(theta));

  vec3 accum_weight = vec3(0.0);
  vec3 accum_radiance = vec3(0.0);

  for (int i = 0; i < uniform_buf.subsurface.sample_len; i++) {
    vec2 sample_uv = center_uv + sample_space * uniform_buf.subsurface.samples[i].xy;
    float pdf_inv = uniform_buf.subsurface.samples[i].z;

    /* TODO(fclem): L0 cache using LDS. */
    float sample_depth = textureLod(hiz_tx, sample_uv * uniform_buf.hiz.uv_scale, 0.0).r;
    vec4 sample_data = texture(radiance_id_tx, sample_uv);

    vec3 sample_vP = get_view_space_from_depth(sample_uv, sample_depth);
    vec3 sample_radiance = sample_data.rgb;
    uint sample_sss_id = uint(sample_data.a);

    if (sample_sss_id != gbuf.diffuse.sss_id) {
      continue;
    }

    /* Slide 34. */
    float r = distance(sample_vP, vP);
    vec3 weight = burley_eval(d, r) * pdf_inv;

    accum_radiance += sample_radiance * weight;
    accum_weight += weight;
  }
  /* Normalize the sum (slide 34). */
  accum_radiance *= safe_rcp(accum_weight);

  /* Put result in direct diffuse. */
  imageStore(out_direct_light_img, texel, vec4(accum_radiance, 0.0));
  imageStore(out_indirect_light_img, texel, vec4(0.0, 0.0, 0.0, 0.0));
}
