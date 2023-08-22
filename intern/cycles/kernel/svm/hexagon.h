/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

CCL_NAMESPACE_BEGIN

/* Hexagon */

#define HRATIO 1.1547005f
#define HSQRT3 1.7320508f
#define HSQRT2 1.4142136f

/*
 * SDF Functions based on:
 * - https://www.iquilezles.org/www/articles/distfunctions2d/distfunctions2d.htm
 */

ccl_device_inline float sdf_dimension(float w, float *r)
{
  float roundness = *r;
  float sw = compatible_signf(w);
  w = fabsf(w);
  roundness = mix(0.0f, w, clamp(roundness, 0.0f, 1.0f));
  const float dimension = max(w - roundness, 0.0f);
  *r = roundness * 0.5f;
  return dimension * sw;
}

ccl_device_inline float hex_value_sdf(const float3 pos, float r, float rd)
{
  float2 p = float3_to_float2(pos);
  r = sdf_dimension(r, &rd);
  const float3 k = make_float3(HSQRT3 * -0.5f, 0.5f, HRATIO * 0.5f);
  p = fabs(p);
  float2 kxy = float3_to_float2(k);
  p = p - (2.0f * min(dot(float3_to_float2(k), p), 0.0f) * float3_to_float2(k));
  p = p - make_float2(clamp(p.x, -k.z * r, k.z * r), r);
  return len(p) * signf(p.y) - rd * 2.0f;
}

ccl_device_inline float hex_value(const float3 hp, const float radius)
{
  float3 fac = make_float3(fabsf(hp.x - hp.y), fabsf(hp.y - hp.z), fabsf(hp.z - hp.x));
  float f = max(fac.x, max(fac.y, fac.z));
  return (radius == 0.0f) ? f : mix(f, len(fac) / HSQRT2, radius);
}

ccl_device_inline float3 xy_to_hex(const float3 xy, const float ratio)
{
  float3 p = xy;
  p.x *= ratio;
  p.z = -0.5f * p.x - p.y;
  p.y = -0.5f * p.x + p.y;
  return p;
}

ccl_device_noinline_cpu float svm_hexagon(float3 p,
                                          const float scale,
                                          const float size,
                                          const float radius,
                                          const float roundness,
                                          const NodeHexagonCoordinateMode coord_mode,
                                          const NodeHexagonValueMode value_mode,
                                          const NodeHexagonDirection direction,
                                          float3 *hex_coords,
                                          float3 *grid_position,
                                          float3 *cell_coords,
                                          float3 *cell_id,
                                          bool calc_value)
{
  const float ratio = (direction == NODE_HEXAGON_DIRECTION_HORIZONTAL_TILED ||
                       direction == NODE_HEXAGON_DIRECTION_VERTICAL_TILED) ?
                          1.0f :
                          HRATIO;
  if (direction == NODE_HEXAGON_DIRECTION_VERTICAL ||
      direction == NODE_HEXAGON_DIRECTION_VERTICAL_TILED)
  {
    p = make_float3(p.y, p.x, p.y);
  }
  p = xy_to_hex(p * scale, ratio);
  *hex_coords = p;
  float3 ip = floor(p + 0.5f);
  const float s = ip.x + ip.y + ip.z;
  float3 abs_d = make_float3(0.0f);
  if (s != 0.0f) {
    abs_d = fabs(ip - p);
    if (abs_d.x >= abs_d.y && abs_d.x >= abs_d.z) {
      ip.x -= s;
    }
    else if (abs_d.y >= abs_d.x && abs_d.y >= abs_d.z) {
      ip.y -= s;
    }
    else {
      ip.z -= s;
    }
  }
  float3 hp = p - ip;
  hp *= (size != 0.0f) ? 1.0f / size : 0.0f;
  const float3 xy_coords = make_float3(hp.x * HSQRT3, hp.y - hp.z, 0.0f);
  if (coord_mode == NODE_HEXAGON_COORDS_HEX) {
    *cell_coords = hp;
    *cell_id = ip;
  }
  else {
    *cell_coords = xy_coords;
    *cell_id = make_float3(
        ip.x / ratio, (ip.y - ip.z + (1.0f - safe_floored_modulo(ip.x, 2.0f))) / 2.0f, 0.0f);
  }
  if (direction == NODE_HEXAGON_DIRECTION_VERTICAL ||
      direction == NODE_HEXAGON_DIRECTION_VERTICAL_TILED)
  {
    hp = make_float3(hp.y, hp.x, hp.z);
    *cell_coords = make_float3(cell_coords->y, cell_coords->x, cell_coords->z);
    *cell_id = make_float3(cell_id->y, cell_id->x, cell_id->z);
  }
  *grid_position = safe_divide(*cell_id, make_float3(scale));
  /* Calc value. */
  if (!calc_value) {
    return 0.0f;
  }

  if (value_mode == NODE_HEXAGON_VALUE_DOT) {
    return len(hp);
  }
  else if (value_mode == NODE_HEXAGON_VALUE_SDF) {
    return hex_value_sdf(xy_coords, radius, roundness);
  }
  else { /* NODE_HEXAGON_VALUE_HEX */
    return hex_value(hp, radius);
  }
}

ccl_device int svm_node_tex_hexagon(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  uint4 node2 = read_node(kg, &offset);
  uint4 defaults = read_node(kg, &offset);

  /* Input and Output Sockets */
  uint vec_in_stack_offset, coord_mode_offset, value_mode_offset, direction_mode_offset,
      scale_stack_offset, size_stack_offset, radius_stack_offset, roundness_stack_offset;
  uint color_stack_offset, cell_stack_offset, grid_position_stack_offset, cell_coords_stack_offset,
      hex_coords_stack_offset, value_stack_offset;

  svm_unpack_node_uchar4(node.y,
                         &vec_in_stack_offset,
                         &coord_mode_offset,
                         &value_mode_offset,
                         &direction_mode_offset);
  svm_unpack_node_uchar4(
      node.z, &scale_stack_offset, &size_stack_offset, &radius_stack_offset, &color_stack_offset);
  svm_unpack_node_uchar4(node.w,
                         &cell_stack_offset,
                         &grid_position_stack_offset,
                         &cell_coords_stack_offset,
                         &value_stack_offset);

  svm_unpack_node_uchar2(node2.x, &roundness_stack_offset, &hex_coords_stack_offset);

  float3 co = stack_load_float3(stack, vec_in_stack_offset);
  float scale = stack_load_float_default(stack, scale_stack_offset, defaults.x);
  float size = stack_load_float_default(stack, size_stack_offset, defaults.y);
  float radius = stack_load_float_default(stack, radius_stack_offset, defaults.z);
  float roundness = stack_load_float_default(stack, roundness_stack_offset, defaults.w);

  float3 color = zero_float3();
  float3 hex_coords = zero_float3();
  float3 grid_position = zero_float3();
  float3 cell_coords = zero_float3();
  float3 cell_id = zero_float3();

  bool calc_value = stack_valid(value_stack_offset);

  float value = svm_hexagon(co,
                            scale,
                            size,
                            radius,
                            roundness,
                            (NodeHexagonCoordinateMode)coord_mode_offset,
                            (NodeHexagonValueMode)value_mode_offset,
                            (NodeHexagonDirection)direction_mode_offset,
                            &hex_coords,
                            &grid_position,
                            &cell_coords,
                            &cell_id,
                            calc_value);

  if (stack_valid(cell_stack_offset)) {
    stack_store_float3(stack, cell_stack_offset, cell_id);
  }
  if (stack_valid(grid_position_stack_offset)) {
    stack_store_float3(stack, grid_position_stack_offset, grid_position);
  }
  if (stack_valid(hex_coords_stack_offset)) {
    stack_store_float3(stack, hex_coords_stack_offset, hex_coords);
  }
  if (stack_valid(cell_coords_stack_offset)) {
    stack_store_float3(stack, cell_coords_stack_offset, cell_coords);
  }
  if (calc_value) {
    stack_store_float(stack, value_stack_offset, value);
  }
  if (stack_valid(color_stack_offset)) {
    color = hash_float3_to_float3(cell_id);
    stack_store_float3(stack, color_stack_offset, color);
  }
  return offset;
}

CCL_NAMESPACE_END
