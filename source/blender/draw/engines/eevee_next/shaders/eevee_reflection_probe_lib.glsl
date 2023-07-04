
#pragma BLENDER_REQUIRE(eevee_cubemap_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)

vec4 reflection_probes_sample(vec3 L, float lod, ReflectionProbeData probe_data)
{
  vec2 octahedral_uv_packed = octahedral_uv_from_direction(L);
  vec2 octahedral_uv = octahedral_reflection_probe_unpack(octahedral_uv_packed, probe_data, lod);
  return textureLod(reflectionProbes, vec3(octahedral_uv, probe_data.layer), lod);
}

vec3 reflection_probes_world_sample(vec3 L, float lod)
{
  return reflection_probes_sample(L, lod, reflection_probe_buf[0]).rgb;
}