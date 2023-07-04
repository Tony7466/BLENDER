
/* Shader to convert cubemap to octahedral projection. */

#pragma BLENDER_REQUIRE(eevee_octahedron_lib.glsl)

void main()
{
  ReflectionProbeData probe_data = reflection_probe_buf[0];

  ivec3 octahedral_coord = ivec3(gl_GlobalInvocationID.xyz);
  // TODO: dispatch should also consider this.
  ivec3 texture_size = imageSize(octahedral_img);
  ivec2 octahedral_size = ivec2(texture_size.x >> probe_data.layer_subdivision,
                                texture_size.y >> probe_data.layer_subdivision);
  /* Group doesn't fit in output texture. */
  if (any(greaterThanEqual(octahedral_coord.xy, octahedral_size.xy))) {
    return;
  }
  vec2 octahedral_uv = vec2(octahedral_coord.xy) / vec2(octahedral_size.xy);
  vec3 R = octahedral_uv_to_direction(octahedral_uv);

  vec4 col = textureLod(cubemap_tx, R, 0.0);

  int probes_per_dimension = 1 << probe_data.layer_subdivision;
  ivec2 area_coord = ivec2(probe_data.area_index % probes_per_dimension,
                           probe_data.area_index / probes_per_dimension);
  ivec2 area_offset = area_coord * octahedral_size;

  imageStore(octahedral_img, octahedral_coord + ivec3(area_offset, 0), col);
}