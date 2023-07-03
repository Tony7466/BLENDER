
#pragma BLENDER_REQUIRE(eevee_cubemap_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)

vec3 light_world_sample(vec3 L, float lod)
{
  return textureLod(reflectionProbes, L, lod).rgb;
}
