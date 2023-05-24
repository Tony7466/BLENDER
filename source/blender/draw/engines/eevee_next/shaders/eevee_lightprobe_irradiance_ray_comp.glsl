
/**
 * For every irradiance probe sample, compute the incomming radiance from both side.
 * This is the same as the surfel ray but we do not actually transport the light, we only capture
 * the irradiance as spherical harmonic coefficients.
 *
 * Dispatched as 1 thread per irradiance probe sample.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_spherical_harmonics_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surfel_list_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)

void irradiance_capture(vec3 L, vec3 irradiance, inout SphericalHarmonicL1 sh)
{
  vec3 lL = transform_direction(capture_info_buf.irradiance_grid_world_to_local_rotation, L);

  spherical_harmonics_encode_signal_sample(lL, vec4(irradiance, 1.0), sh);
}

void irradiance_capture(Surfel surfel_emitter, vec3 P, inout SphericalHarmonicL1 sh)
{
  vec3 L = safe_normalize(surfel_emitter.position - P);
  bool facing = dot(-L, surfel_emitter.normal) > 0.0;
  vec3 irradiance = facing ? surfel_emitter.outgoing_light_front :
                             surfel_emitter.outgoing_light_back;

  irradiance_capture(L, irradiance, sh);
}

void main()
{
  ivec3 grid_coord = ivec3(gl_GlobalInvocationID);

  if (any(greaterThanEqual(grid_coord, capture_info_buf.irradiance_grid_size))) {
    return;
  }

  vec3 P = lightprobe_irradiance_grid_sample_position(
      capture_info_buf.irradiance_grid_local_to_world,
      capture_info_buf.irradiance_grid_size,
      grid_coord);

  /* Project to get ray linked list. */
  float irradiance_sample_ray_distance;
  int list_index = surfel_list_index_get(P, irradiance_sample_ray_distance);

  /* Walk the ray to get which surfels the irradiance sample is between. */
  int surfel_prev = -1;
  int surfel_next = list_start_buf[list_index];
  for (; surfel_next > -1; surfel_next = surfel_buf[surfel_next].next) {
    /* Reminder: List is sorted with highest value first. */
    if (surfel_buf[surfel_next].ray_distance < irradiance_sample_ray_distance) {
      break;
    }
    surfel_prev = surfel_next;
  }

  vec3 sky_L = cameraVec(P);

  SphericalHarmonicL1 sh;
  sh.L0.M0 = imageLoad(irradiance_L0_img, grid_coord);
  sh.L1.Mn1 = imageLoad(irradiance_L1_a_img, grid_coord);
  sh.L1.M0 = imageLoad(irradiance_L1_b_img, grid_coord);
  sh.L1.Mp1 = imageLoad(irradiance_L1_c_img, grid_coord);

  /* Spherical harmonics need to be weighted by sphere area. */
  const float sphere_area = 4.0 * M_PI;
  /* Un-normalize for accumulation. */
  float weight_captured = float(capture_info_buf.irradiance_accum_sample_count) / sphere_area;
  sh.L0.M0 *= weight_captured;
  sh.L1.Mn1 *= weight_captured;
  sh.L1.M0 *= weight_captured;
  sh.L1.Mp1 *= weight_captured;

  if (surfel_next > -1) {
    irradiance_capture(surfel_buf[surfel_next], P, sh);
  }
  else {
    /* TODO(fclem): Sky radiance. */
    irradiance_capture(sky_L, vec3(0.0), sh);
  }

  if (surfel_prev > -1) {
    irradiance_capture(surfel_buf[surfel_prev], P, sh);
  }
  else {
    /* TODO(fclem): Sky radiance. */
    irradiance_capture(-sky_L, vec3(0.0), sh);
  }

  /* Normalize for storage. */
  weight_captured += 2.0 / sphere_area;
  sh.L0.M0 /= weight_captured;
  sh.L1.Mn1 /= weight_captured;
  sh.L1.M0 /= weight_captured;
  sh.L1.Mp1 /= weight_captured;

  imageStore(irradiance_L0_img, grid_coord, sh.L0.M0);
  imageStore(irradiance_L1_a_img, grid_coord, sh.L1.Mn1);
  imageStore(irradiance_L1_b_img, grid_coord, sh.L1.M0);
  imageStore(irradiance_L1_c_img, grid_coord, sh.L1.Mp1);
}
