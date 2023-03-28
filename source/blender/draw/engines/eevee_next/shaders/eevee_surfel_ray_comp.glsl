
/**
 * For every surfel, compute the incomming radiance from both side.
 * For that, walk the ray surfel linked-list and gather the light from the neighbor surfels.
 *
 * Dispatched as 1 thread per surfel.
 */

#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)

/**
 * Evaluate radiance transfer from `surfel_a` to `surfel_b`.
 * Assume ray direction is uniformly distributed over the sphere.
 * NOTE: In practice the ray is not evenly distributed but that's a by-product
 * of the surfel list method.
 *
 * Return radiance + pdf.
 */
void radiance_transfer(Surfel surfel_a, inout Surfel surfel_b)
{
  bool facing = dot(surfel_a.normal, surfel_b.normal) < 0.0;

  vec3 L = normalize(surfel_a.position - surfel_b.position);
  vec3 N = surfel_b.normal;
  float NL = dot(N, L);
  float pdf = abs(NL);
  vec3 irradiance = facing ? surfel_a.radiance_front : surfel_a.radiance_back;

  if (NL > 0.0) {
    surfel_b.radiance_bounce_front += vec4(surfel_b.albedo_front * irradiance * pdf, pdf);
  }
  else {
    surfel_b.radiance_bounce_back += vec4(surfel_b.albedo_back * irradiance * pdf, pdf);
  }
}

void sky_radiance(inout Surfel surfel)
{
  vec3 L = cameraVec(surfel.position);
  vec3 N = surfel.normal;
  float NL = dot(N, L);
  /* TODO(fclem): Sky cubemap sampling. */
  vec3 Li = vec3(0.0);
  /* Assume white albedo. */
  // float pdf = M_1_PI;
  float pdf = abs(NL);

  vec3 radiance = Li;

  if (NL > 0.0) {
    surfel.radiance_bounce_front += vec4(radiance * pdf, pdf);
  }
  else {
    surfel.radiance_bounce_back += vec4(radiance * pdf, pdf);
  }
}

void main()
{
  int surfel_index = int(gl_GlobalInvocationID);
  if (surfel_index >= capture_info_buf.surfel_len) {
    return;
  }

  Surfel surfel = surfel_buf[surfel_index];

  vec4 radiance_with_pdf = vec4(0.0);
  if (surfel.next > -1) {
    radiance_transfer(surfel_buf[surfel.next], surfel);
  }
  else {
    sky_radiance(surfel);
  }

  if (surfel.prev > -1) {
    radiance_transfer(surfel_buf[surfel.prev], surfel);
  }
  else {
    sky_radiance(surfel);
  }

  surfel_buf[surfel_index].radiance_bounce_front = surfel.radiance_bounce_front;
  surfel_buf[surfel_index].radiance_bounce_back = surfel.radiance_bounce_back;
}
