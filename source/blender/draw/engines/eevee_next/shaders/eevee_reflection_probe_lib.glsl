void light_world_eval(ClosureReflection reflection, vec3 P, vec3 V, inout vec3 out_specular)
{
  float linear_roughness = fast_sqrt(reflection.roughness);
  /* TODO: This should be based by actual resolution. Currently the resolution is fixed but
   * eventually this should based on a user setting. */
  float lod_cube_max = 12.0;
  float lod = linear_roughness * lod_cube_max;

  vec3 R = -reflect(V, reflection.N);
  vec3 world_light = textureLod_cubemapArray(reflectionProbes, vec4(R, 0.0), lod).rgb;
  out_specular += world_light;
}
