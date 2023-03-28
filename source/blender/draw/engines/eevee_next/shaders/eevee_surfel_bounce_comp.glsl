
/**
 * Accumulate light from a bounce of indirect light into each surfel radiance.
 * This feeds back the light for the next bounce.
 */

#pragma BLENDER_REQUIRE(common_math_lib.glsl)

void main()
{
  int surfel_index = int(gl_GlobalInvocationID);
  if (surfel_index >= capture_info_buf.surfel_len) {
    return;
  }

  vec4 radiance_bounce_front = surfel_buf[surfel_index].radiance_bounce_front;
  vec4 radiance_bounce_back = surfel_buf[surfel_index].radiance_bounce_back;
  surfel_buf[surfel_index].radiance_front += radiance_bounce_front.rgb *
                                             safe_rcp(radiance_bounce_front.w);
  surfel_buf[surfel_index].radiance_back += radiance_bounce_back.rgb *
                                            safe_rcp(radiance_bounce_back.w);
  surfel_buf[surfel_index].radiance_bounce_front = vec4(0.0);
  surfel_buf[surfel_index].radiance_bounce_back = vec4(0.0);
}
