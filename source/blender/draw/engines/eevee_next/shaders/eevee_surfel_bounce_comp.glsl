
/**
 * Accumulate light from a bounce of indirect light into each surfel radiance.
 * This feeds back the light for the next bounce.
 *
 * Dispatched as one thread per surfel.
 */

#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)

vec3 finalize_integration(vec4 radiance)
{
  /* Divide by sample count. */
  radiance.rgb *= safe_rcp(radiance.w);
  /* TODO: Find why this is needed. */
  radiance.rgb *= 2.0;
  /* Multiply by hemisphere area. */
  return radiance.rgb * M_TAU;
}

void main()
{
  int surfel_index = int(gl_GlobalInvocationID);
  if (surfel_index >= capture_info_buf.surfel_len) {
    return;
  }

  vec4 radiance_bounce_front = surfel_buf[surfel_index].radiance_bounce_front;
  vec4 radiance_bounce_back = surfel_buf[surfel_index].radiance_bounce_back;
  surfel_buf[surfel_index].radiance_front += finalize_integration(radiance_bounce_front);
  surfel_buf[surfel_index].radiance_back += finalize_integration(radiance_bounce_back);
  surfel_buf[surfel_index].radiance_bounce_front = vec4(0.0);
  surfel_buf[surfel_index].radiance_bounce_back = vec4(0.0);
}
