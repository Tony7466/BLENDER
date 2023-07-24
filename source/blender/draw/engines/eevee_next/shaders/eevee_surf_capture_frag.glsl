
/**
 * Surface Capture: Output surface parameters to diverse storage.
 *
 * This is a separate shader to allow custom closure behavior and avoid putting more complexity
 * into other surface shaders.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_matrix_lib.glsl)
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

float lod_from_position(vec3 P)
{
  vec3 lP = transform_point(capture_info_buf.irradiance_grid_world_to_local, P);
  vec3 distance_to_grid_volume = (abs(lP) - 1.0) *
                                 to_scale(capture_info_buf.irradiance_grid_local_to_world);
  float dist = max_v3(max(distance_to_grid_volume, 0.0));
  float lod = max(log2(dist), 0.0);
  return floor(lod);
}

ivec2 closest_valid_surfel(ivec2 pixel, int lod)
{
  /* NOTE: We skip LOD0 because we render at twice the base density for correct LOD alignment. */
  int lod_skip = lod + 1;
  return ((pixel >> lod_skip) << lod_skip) + (1 << lod);
}

bvec2 lod_surfel_output(ivec2 pixel, int lod)
{
  return equal(closest_valid_surfel(pixel, lod), pixel);
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
    float lod = lod_from_position(g_data.P);

    float lod_center = lod;
    /* Plus 1 for the lod0 skip, plus 1 for size of the next lod. */
    float delta = exp2(lod_center + 0.0);
    float lod_delta_x_pos = lod_from_position(g_data.P + dFdx(g_data.P) * delta);
    float lod_delta_y_pos = lod_from_position(g_data.P + dFdy(g_data.P) * delta);
    float lod_delta_x_neg = lod_from_position(g_data.P - dFdx(g_data.P) * delta);
    float lod_delta_y_neg = lod_from_position(g_data.P - dFdy(g_data.P) * delta);

    float lod_delta_min = min4(lod_delta_x_pos, lod_delta_y_pos, lod_delta_x_neg, lod_delta_y_neg);
    float lod_delta_max = max4(lod_delta_x_pos, lod_delta_y_pos, lod_delta_x_neg, lod_delta_y_neg);

    ivec2 pixel = ivec2(gl_FragCoord.xy);
    /* This is conservative. We spawn surfels of both lods at LOD transitions. */
    bool is_valid_surfel_center = all(lod_surfel_output(pixel, int(lod_center)));
    bool is_valid_surfel_border = all(lod_surfel_output(pixel, int(lod_delta_min)));

    is_valid_surfel_border = is_valid_surfel_border && (lod_delta_min != lod_center);
#if 0 /* Need to be finished. For now, just over spaw surfel to enforce watertighness. */
    /* Remove false positive. */
    if (is_valid_surfel_border) {
      ivec2 valid_surfel = closest_valid_surfel(pixel, int(lod_center));
      bool covered_by_bigger_surfel = any(lessThan(pixel, valid_surfel));
      is_valid_surfel_border = !covered_by_bigger_surfel;
    }
#endif
    if (is_valid_surfel_center) {
      lod = lod_center;
    }
    else if (is_valid_surfel_border) {
      lod = lod_delta_min;
    }
    bool is_valid_surfel = is_valid_surfel_center || is_valid_surfel_border;

    /* Generate a surfel only once. This check allow cases where no axis is dominant. */
    bool is_surface_view_aligned = dominant_axis(g_data.Ng) == dominant_axis(cameraForward);
    if (is_surface_view_aligned && is_valid_surfel) {
      uint surfel_id = atomicAdd(capture_info_buf.surfel_len, 1u);
      if (capture_info_buf.do_surfel_output) {
        surfel_buf[surfel_id].position = g_data.P;
        surfel_buf[surfel_id].normal = gl_FrontFacing ? g_data.Ng : -g_data.Ng;
        surfel_buf[surfel_id].lod = int(lod);
        surfel_buf[surfel_id].albedo_front = albedo;
        surfel_buf[surfel_id].radiance_direct.front.rgb = g_emission;
        /* TODO(fclem): 2nd surface evaluation. */
        surfel_buf[surfel_id].albedo_back = albedo;
        surfel_buf[surfel_id].radiance_direct.back.rgb = g_emission;
      }
    }
  }
}
