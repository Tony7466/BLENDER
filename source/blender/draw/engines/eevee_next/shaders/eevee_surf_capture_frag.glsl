
/**
 * Surface Capture: Output surface parameters to diverse storage.
 *
 * This is a separate shader to allow custom closure behavior and avoid putting more complexity
 * into other surface shaders.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(common_hair_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_surf_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_nodetree_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)

vec4 closure_to_rgba(Closure cl)
{
  return vec4(0.0);
}

void main()
{
  init_globals();

  /* TODO(fclem): Remove random sampling for capture and accumulate color. */
  g_closure_rand = 0.5;

  nodetree_surface();

  g_diffuse_data.color *= g_diffuse_data.weight;
  g_reflection_data.color *= g_reflection_data.weight;
  g_refraction_data.color *= g_refraction_data.weight;

  vec3 albedo = g_diffuse_data.color + g_reflection_data.color;

  /* ----- Surfel output ----- */

  if (capture_info_buf.do_surfel_count) {
    /* Reject surfels based on distance to the grid. */
    vec3 lP = transform_point(capture_info_buf.irradiance_grid_world_to_local, g_data.P);
    float dist = max_v3(abs(lP));
    float lod = max(log2(dist), 0.0);
    // float noise = interlieved_gradient_noise(gl_FragCoord.xy, 5.0, 0.0);
    // float noise = utility_tx_fetch(utility_tx, gl_FragCoord.xy, UTIL_BLUE_NOISE_LAYER).r;
    float noise = transparency_hashed_alpha_threshold(1.0, noise_offset, g_data.P);
    bool is_valid_surfel = noise < (1.0 / exp2(lod * 2.0));
    /* Generate a surfel only once. This check allow cases where no axis is dominant. */
    bool is_surface_view_aligned = dominant_axis(g_data.Ng) == dominant_axis(cameraForward);
    if (is_surface_view_aligned && is_valid_surfel) {
      uint surfel_id = atomicAdd(capture_info_buf.surfel_len, 1u);
      if (capture_info_buf.do_surfel_output) {
        surfel_buf[surfel_id].position = g_data.P;
        surfel_buf[surfel_id].normal = gl_FrontFacing ? g_data.Ng : -g_data.Ng;
        surfel_buf[surfel_id].lod = lod;
        surfel_buf[surfel_id].albedo_front = albedo;
        surfel_buf[surfel_id].radiance_direct.front.rgb = g_emission;
        /* TODO(fclem): 2nd surface evaluation. */
        surfel_buf[surfel_id].albedo_back = albedo;
        surfel_buf[surfel_id].radiance_direct.back.rgb = g_emission;
      }
    }
  }
}
