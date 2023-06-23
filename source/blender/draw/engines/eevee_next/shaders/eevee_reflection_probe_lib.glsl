
float4 light_probe_eval(ClosureReflection reflection, vec3 P, vec3 V, in ReflectionProbeData probe)
{
  float linear_roughness = fast_sqrt(reflection.roughness);
  /* TODO: This should be based by actual resolution. Currently the resolution is fixed but
   * eventually this should based on a user setting.
   * (12.0 - subdivision level)
   */
  float lod_cube_max = 12.0;
  float lod = linear_roughness * lod_cube_max;

  vec3 R = -reflect(V, reflection.N);
  vec4 probe_light = textureLod_cubemapArray(reflectionProbes, vec4(R, probe.layer), lod);
  return probe_light;
}

void light_probes_eval(ClosureReflection reflection, vec3 P, vec3 V, inout vec3 out_specular)
{
  vec4 light = vec4(0.0);
  float tot_weight = 0.0;
  for (int probe_index = 1; probe_index < REFLECTION_PROBES_MAX; probe_index++) {
    ReflectionProbeData probe_data = reflection_probe_buf[probe_index];
    /* Probe data are tightly packed so we can break the for loop. */
    if (probe_data.layer == -1) {
      break;
    }
    /* Calculate the weight of the probe on P. */
    /* TODO: rewrite to mix? */
    float probe_weight = 1.0 -
                         smoothstep(probe_data.influence_distance * (1.0 - probe_data.falloff),
                                    probe_data.influence_distance,
                                    length(probe_data.pos - P));
    if (probe_weight > 0.0) {
      vec4 probe_light = light_probe_eval(reflection, P, V, probe_data);
      light.rgb += probe_light.rgb * probe_light.a * probe_weight;
      light.a += probe_light.a * probe_weight;
    }
    tot_weight += probe_weight;
  }

  float world_mix = 1.0 - clamp(light.a, 0.0, 1.0);
  if (world_mix > 0.0) {
    out_specular += mix(
        light.rgb, light_probe_eval(reflection, P, V, reflection_probe_buf[0]).rgb, world_mix);
  }
}
