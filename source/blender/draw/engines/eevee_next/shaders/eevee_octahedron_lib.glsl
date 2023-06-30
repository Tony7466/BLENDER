/**
 * Convert from a cubemap vector to an octahedron UV coordinate.
 * Z = 0.0 for top pyramid. Z = 1.0 for bottom pyramid.
 */
vec3 octahedron_from_cubevec(vec3 cubemap_vector)
{
  return cubemap_vector;
}

vec3 octahedron_to_cubevec(vec3 octahedron_vector)
{
  return orthahedron_vector;
}