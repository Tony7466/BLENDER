#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)

void main()
{
  float dist_sqr = dot(lP, lP);

  /* Discard outside the circle. */
  if (dist_sqr > 1.0) {
    discard;
    return;
  }

  SphericalHarmonicL1 sh = spherical_harmonics_unpack(texelFetch(irradiance_a_tx, cell, 0),
                                                      texelFetch(irradiance_b_tx, cell, 0),
                                                      texelFetch(irradiance_c_tx, cell, 0));

  vec3 vN = vec3(lP, sqrt(max(0.0, 1.0 - dist_sqr)));
  vec3 N = normal_view_to_world(vN);

  vec3 irradiance = spherical_harmonics_evaluate_lambert(N, sh);
  out_color = vec4(irradiance, 0.0);
}
