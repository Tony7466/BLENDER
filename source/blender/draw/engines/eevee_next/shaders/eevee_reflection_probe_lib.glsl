#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

void light_world_eval(ClosureReflection reflection, vec3 P, vec3 V, inout vec3 out_specular)
{
  ivec3 texture_size = textureSize(reflectionProbes, 0);
  /* TODO: This should be based by actual resolution. Currently the resolution is fixed but
   * eventually this should based on a user setting. */
  float lod_cube_max = 12.0;

  vec3 T, B;
  make_orthonormal_basis(reflection.N, T, B);

  float weight = 0.0;
  vec3 out_radiance = vec3(0.0);
  /* Note: this was dynamic based on the mipmap level that was read from. */
  /* lod 0 => 1
   *     1 => 32
   *     2 => 40
   *     3 => 64
   *  else => 128
   * multiplied by the filter quality. */
  const int max_sample_count = 32;
  /* Note this is dynamically based on the mip map level.
   pinfo->lodfactor = bias + 0.5f * log(square_f(target_size) / pinfo->samples_len) / log(2);
   */
  /* Pow4f for Disney Roughness distributed across lod more evenly */
  float roughness = clamp(pow4f(reflection.roughness), 1e-4f, 0.9999f); /* Avoid artifacts */

  const float lod_factor = 1.0 +
                           0.5 * log(float(square_i(texture_size.x)) / float(max_sample_count)) /
                               log(2);
  /* We should find a math formular that would approx max_sample_count and bias. */
  for (int i = 0; i < max_sample_count; i++) {
    vec3 Xi = sample_cylinder(hammersley_2d(float(i), max_sample_count));

    /* Microfacet normal */
    float pdf;
    vec3 H = sample_ggx(Xi, roughness, V, reflection.N, T, B, pdf);

    vec3 L = -reflect(V, H);
    float NL = dot(reflection.N, L);

    if (NL > 0.0) {
      float NH = max(1e-8, dot(reflection.N, H)); /*cosTheta */

      /* Coarse Approximation of the mapping distortion
       * Unit Sphere -> Cubemap Face */
      const float dist = 4.0 * M_PI / 6.0;
      /* http://http.developer.nvidia.com/GPUGems3/gpugems3_ch20.html : Equation 13 */
      float lod = clamp(lod_factor - 0.5 * log2(pdf * dist), 0.0, lod_cube_max);

      vec3 l_col = textureLod_cubemapArray(reflectionProbes, vec4(L, 0.0), lod).rgb;

      /* Clamped brightness. */
      float luma = max(1e-8, max_v3(l_col));

      /* For artistic freedom this should be read from the scene/reflection probe.
       * Note: Eevee-legacy read the firefly_factor from gi_glossy_clamp. */
      const float firefly_factor = 1e16;
      l_col *= 1.0 - max(0.0, luma - firefly_factor) / luma;

      out_radiance += l_col * NL;
      weight += NL;
    }
  }

  /* TODO: for artistic freedom want to read this from the reflection probe. That can be added as
   * part of the reflection probe patch. */
  const float intensity_factor = 1.0;
  out_specular += vec3(intensity_factor * out_radiance / weight);
}
