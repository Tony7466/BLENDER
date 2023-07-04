/**
 * Convert from a cubemap vector to an octahedron UV coordinate.
 */
vec2 octahedral_uv_from_direction(vec3 co)
{
  /* Projection onto octahedron. */
  co /= dot(vec3(1.0), abs(co));

  /* Out-folding of the downward faces. */
  if (co.z < 0.0) {
    vec2 sign = step(0.0, co.xy) * 2.0 - 1.0;
    co.xy = (1.0 - abs(co.yx)) * sign;
  }

  /* Mapping to [0;1]^2 texture space. */
  vec2 uvs = co.xy * (0.5) + 0.5;

  return uvs;
}

vec3 octahedral_uv_to_direction(vec2 co)
{
  co = co * 2.0 - 1.0;

  vec2 abs_co = abs(co);
  vec3 v = vec3(co, 1.0 - (abs_co.x + abs_co.y));

  if (abs_co.x + abs_co.y > 1.0) {
    v.xy = (abs(co.yx) - 1.0) * -sign(co.xy);
  }

  return v;
}

vec2 octahedral_reflection_probe_unpack(vec2 octahedral_uv,
                                        ReflectionProbeData probe_data,
                                        float lod)
{
  /* Fix artifacts near edges. */
  int i_lod = int(ceil(lod));
  float texel_size = 1.0 / (1 << (12 - probe_data.layer_subdivision - i_lod));
  octahedral_uv = (1.0 - 2.0 * texel_size) * octahedral_uv + texel_size;

  int areas_per_dimension = 1 << probe_data.layer_subdivision;
  vec2 area_scalar = vec2(1.0 / float(areas_per_dimension));
  octahedral_uv *= area_scalar;

  vec2 area_offset = vec2(probe_data.area_index % areas_per_dimension,
                          probe_data.area_index / areas_per_dimension) *
                     area_scalar;
  return octahedral_uv + area_offset;
}
