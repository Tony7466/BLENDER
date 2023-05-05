
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
  /* Multiply by hemisphere area since we are integrating over it. */
  return radiance.rgb * M_TAU;
}

void main()
{
  int surfel_index = int(gl_GlobalInvocationID.x);
  if (surfel_index >= capture_info_buf.surfel_len) {
    return;
  }
  Surfel surfel = surfel_buf[surfel_index];
  vec3 radiance_front = finalize_integration(surfel.incomming_light_front);
  vec3 radiance_back = finalize_integration(surfel.incomming_light_back);
  /* Re-inject the bounced light for the next bounce event. */
  surfel_buf[surfel_index].outgoing_light_front = radiance_front;
  surfel_buf[surfel_index].outgoing_light_back = radiance_back;
  /* Add to final radiance. */
  surfel_buf[surfel_index].radiance_front += radiance_front;
  surfel_buf[surfel_index].radiance_back += radiance_back;
  /* Reset accumulator for next bounce. */
  surfel_buf[surfel_index].incomming_light_front = vec4(0.0);
  surfel_buf[surfel_index].incomming_light_back = vec4(0.0);
}
