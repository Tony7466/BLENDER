
/**
 * For every irradiance probe sample, check if close to a surounding surfel and try to offset the
 * irradiance sample position. This is similar to the surfel ray but we do not actually transport
 * the light.
 *
 * Dispatched as 1 thread per irradiance probe sample.
 */

#pragma BLENDER_REQUIRE(eevee_surfel_list_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_lightprobe_lib.glsl)

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

  vec3 virtual_offset = vec3(0.0);
  float offset_count = 0.0;

  /* TODO(fclem) We could use a larger search radius by traversing the neighbor clusters. */
  int surfel_id = imageLoad(cluster_list_img, grid_coord).r;
  for (; surfel_id > -1; surfel_id = surfel_buf[surfel_id].next) {
    Surfel surfel = surfel_buf[surfel_id];
    vec3 probe_to_surfel = surfel.position - P;
    float distance_to_surfel = length(probe_to_surfel);
    vec3 direction_to_surfel = probe_to_surfel * safe_rcp(distance_to_surfel);
    /* If the surfel is exactly on top of a grid sample, offset by normal. */
    if (distance_to_surfel < 1e-8) {
      direction_to_surfel = -surfel.normal;
    }

    bool is_front_facing = dot(direction_to_surfel, surfel.normal) < 0.0;
    /**
     * There is 2 cases when we want to offset:
     * - When the grid sample is too close to a front-facing surface.
     * - When the grid sample is close enough to a back-facing surface to be offseted.
     */
    if (is_front_facing) {
      /* Only offset a tiny amount. */
      float offset_length = capture_info_buf.min_distance_to_surface - distance_to_surfel;
      if (offset_length > 0.0) {
        /* Repel the sample point. */
        virtual_offset -= direction_to_surfel * offset_length;
        offset_count += 1.0;
      }
    }
    else {
      /* Offset enough to be outside of the geometry. */
      float search_radius = capture_info_buf.max_virtual_offset +
                            capture_info_buf.min_distance_to_surface;
      if (distance_to_surfel < search_radius) {
        float offset_length = capture_info_buf.min_distance_to_surface + distance_to_surfel;
        /* Attract the sample point. */
        virtual_offset += direction_to_surfel * offset_length;
        offset_count += 1.0;
      }
    }
  }
  virtual_offset *= safe_rcp(offset_count);

  float offset_length = length(virtual_offset);
  if (offset_length > capture_info_buf.max_virtual_offset) {
    virtual_offset *= capture_info_buf.max_virtual_offset / offset_length;
  }

  imageStore(virtual_offset_img, grid_coord, vec4(virtual_offset, 0.0));
}
