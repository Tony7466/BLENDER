
/* Fallback implementation for hardware not supporting cubemap arrays.
 * `samplerCubeArray` fallback declaration as sampler2DArray in `glsl_shader_defines.glsl`. */

float cubemap_face_index(vec3 P)
{
  vec3 aP = abs(P);
  if (all(greaterThan(aP.xx, aP.yz))) {
    return (P.x > 0.0) ? 0.0 : 1.0;
  }
  else if (all(greaterThan(aP.yy, aP.xz))) {
    return (P.y > 0.0) ? 2.0 : 3.0;
  }
  else {
    return (P.z > 0.0) ? 4.0 : 5.0;
  }
}

vec2 cubemap_face_coord(vec3 P, float face)
{
  if (face < 2.0) {
    return (P.zy / P.x) * vec2(-0.5, -sign(P.x) * 0.5) + 0.5;
  }
  else if (face < 4.0) {
    return (P.xz / P.y) * vec2(sign(P.y) * 0.5, 0.5) + 0.5;
  }
  else {
    return (P.xy / P.z) * vec2(0.5, -sign(P.z) * 0.5) + 0.5;
  }
}

vec3 cubemap_adj_x(float face)
{
  bool y_axis = (face == 2.0 || face == 3.0);
  return y_axis ? vec3(0.0, 0.0, 1.0) : vec3(0.0, 1.0, 0.0);
}

vec3 cubemap_adj_y(float face)
{
  bool x_axis = (face < 2.0);
  return x_axis ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
}

vec3 cubemap_adj_xy(float face)
{
  if (face < 2.0) {
    return vec3(0.0, 1.0, 1.0);
  }
  else if (face < 4.0) {
    return vec3(1.0, 0.0, 1.0);
  }
  else {
    return vec3(1.0, 1.0, 0.0);
  }
}

/** extract the parameters to perform packing of cubemaps into a single cubemap. */
void cubemap_unpack_params(int subdivision,
                           int area_index,
                           out float unpack_mul,
                           out vec2 unpack_add)
{
  int cubes_per_dimension = 1 << subdivision;
  ivec2 area_offset = ivec2(area_index % cubes_per_dimension, area_index / cubes_per_dimension);
  unpack_mul = 1.0 / cubes_per_dimension;
  unpack_add = vec2(area_offset) * unpack_mul;
}

#ifdef GPU_METAL
template<typename T>
vec4 cubemap_seamless(thread _mtl_combined_image_sampler_2d_array<T, access::sample> *tex,
                      vec4 cubevec,
                      float lod,
                      int subdivision,
                      int area_index)
#else
vec4 cubemap_seamless(sampler2DArray tex, vec4 cubevec, float lod, int subdivision, int area_index)
#endif
{
  float unpack_mul;
  vec2 unpack_add;
  cubemap_unpack_params(subdivision, area_index, unpack_mul, unpack_add);

  int tex_size = textureSize(tex, int(lod)).x;
  float cube_size = max(float(tex_size) * unpack_mul, 1);

  /* Manual Cube map Layer indexing. */
  float face = cubemap_face_index(cubevec.xyz);
  vec2 uv = cubemap_face_coord(cubevec.xyz, face);
  vec3 coord = vec3(uv * unpack_mul + unpack_add, cubevec.w * 6.0 + face);

  vec4 col = textureLod(tex, coord, lod);

  vec2 uv_border = (abs(uv - 0.5) + (0.5 / cube_size - 0.5)) * 2.0 * cube_size;
  bvec2 border = greaterThan(uv_border, vec2(0.0));
  if (all(border)) {
    /* Corners case. */
    vec3 cubevec_adj;
    float face_adj;
    /* Get the other face coords. */
    cubevec_adj = cubevec.xyz * cubemap_adj_x(face);
    face_adj = cubemap_face_index(cubevec_adj);
    /* Still use the original cubevec to get the outer texels or the face. */
    uv = cubemap_face_coord(cubevec.xyz, face_adj);
    coord = vec3(uv * unpack_mul + unpack_add, cubevec.w * 6.0 + face_adj);
    vec4 col1 = textureLod(tex, coord, lod);

    /* Get the 3rd face coords. */
    cubevec_adj = cubevec.xyz * cubemap_adj_y(face);
    face_adj = cubemap_face_index(cubevec_adj);
    /* Still use the original cubevec to get the outer texels or the face. */
    uv = cubemap_face_coord(cubevec.xyz, face_adj);
    coord = vec3(uv * unpack_mul + unpack_add, cubevec.w * 6.0 + face_adj);
    vec4 col2 = textureLod(tex, coord, lod);

    /* Mix all colors to get the corner color. */
    vec4 col3 = (col + col1 + col2) / 3.0;

    vec2 mix_fac = saturate(uv_border * 0.5);
    return mix(mix(col, col2, mix_fac.x), mix(col1, col3, mix_fac.x), mix_fac.y);
  }
  else if (any(border)) {
    /* Edges case. */
    /* Get the other face coords. */
    vec3 cubevec_adj = cubevec.xyz * cubemap_adj_xy(face);
    face = cubemap_face_index(cubevec_adj);
    /* Still use the original cubevec to get the outer texels or the face. */
    uv = cubemap_face_coord(cubevec.xyz, face);
    coord = vec3(uv * unpack_mul + unpack_add, cubevec.w * 6.0 + face);

    float mix_fac = saturate(max(uv_border.x, uv_border.y) * 0.5);
    return mix(col, textureLod(tex, coord, lod), mix_fac);
  }
  else {
    return col;
  }
}

#ifdef GPU_METAL
template<typename T, access A>
vec4 textureLod_packedCubemapArray(thread _mtl_combined_image_sampler_2d_array<T, A> tex,
                                   vec4 cubevec,
                                   float lod,
                                   int subdivision,
                                   int area_index)
#else
vec4 textureLod_packedCubemapArray(
    sampler2DArray tex, vec4 cubevec, float lod, int subdivision, int area_index)
#endif
{
  float lod1 = floor(lod);
  float lod2 = ceil(lod);

  vec4 col_lod1 = cubemap_seamless(tex, cubevec, lod1, subdivision, area_index);
  vec4 col_lod2 = cubemap_seamless(tex, cubevec, lod2, subdivision, area_index);

  return mix(col_lod1, col_lod2, lod - lod1);
}

#ifndef GPU_ARB_texture_cube_map_array
#  ifdef GPU_METAL
template<typename T, access A>
vec4 textureLod_cubemapArray(thread _mtl_combined_image_sampler_2d_array<T, A> tex,
                             vec4 cubevec,
                             float lod)
#  else
vec4 textureLod_cubemapArray(sampler2DArray tex, vec4 cubevec, float lod)
#  endif
{
  return textureLod_packedCubemapArray(tex, cubevec, lod, 0, 0);
}

#endif
