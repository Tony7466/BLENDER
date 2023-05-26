
/**
 * For every surfel, compute the incomming radiance from both side.
 * For that, walk the ray surfel linked-list and gather the light from the neighbor surfels.
 * This shader is dispatched for a random ray in a uniform hemisphere as we evaluate the
 * radiance in both directions.
 *
 * Dispatched as 1 thread per surfel.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(common_view_lib.glsl)
#pragma BLENDER_REQUIRE(common_math_lib.glsl)
#pragma BLENDER_REQUIRE(cubemap_lib.glsl)

void radiance_transfer(inout Surfel surfel, vec3 irradiance, vec3 L)
{
  float NL = dot(surfel.normal, L);
  /* Lambertian BSDF. Albedo applied later depending on which side of the surfel was hit. */
  float bsdf = M_1_PI;
  /* Outgoing light. */
  vec3 radiance = bsdf * irradiance * abs(NL);
  if (NL > 0.0) {
    surfel.incomming_light_front += vec4(radiance * surfel.albedo_front, 1.0);
  }
  else {
    surfel.incomming_light_back += vec4(radiance * surfel.albedo_back, 1.0);
  }
}

void radiance_transfer(inout Surfel surfel, Surfel surfel_emitter)
{
  vec3 L = safe_normalize(surfel_emitter.position - surfel.position);
  bool facing = dot(-L, surfel_emitter.normal) > 0.0;
  vec3 irradiance = facing ? surfel_emitter.outgoing_light_front :
                             surfel_emitter.outgoing_light_back;

  radiance_transfer(surfel, irradiance, L);
}

vec3 radiance_sky_sample(vec3 R)
{
  return textureLod_cubemapArray(reflectionProbes, vec4(R, 0.0), 0.0).rgb;
}

void main()
{
  int surfel_index = int(gl_GlobalInvocationID.x);
  if (surfel_index >= int(capture_info_buf.surfel_len)) {
    return;
  }

  Surfel surfel = surfel_buf[surfel_index];

  vec3 sky_L = cameraVec(surfel.position);

  if (surfel.next > -1) {
    radiance_transfer(surfel, surfel_buf[surfel.next]);
  }
  else {
    vec3 world_radiance = radiance_sky_sample(sky_L);
    radiance_transfer(surfel, world_radiance, sky_L);
  }

  if (surfel.prev > -1) {
    radiance_transfer(surfel, surfel_buf[surfel.prev]);
  }
  else {
    vec3 world_radiance = radiance_sky_sample(-sky_L);
    radiance_transfer(surfel, world_radiance, -sky_L);
  }

  surfel_buf[surfel_index].incomming_light_front = surfel.incomming_light_front;
  surfel_buf[surfel_index].incomming_light_back = surfel.incomming_light_back;
}
