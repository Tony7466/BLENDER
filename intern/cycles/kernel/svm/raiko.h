/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

#define ASSIGN_REMAP_INPUTS_1 \
  remap[0] = stack_load_float_default(stack, so.step_center_1, defaults_16.x); \
  remap[1] = stack_load_float_default(stack, so.step_width_1, defaults_16.y); \
  remap[2] = stack_load_float_default(stack, so.step_value_1, defaults_16.z); \
  remap[3] = stack_load_float_default(stack, so.ellipse_height_1, defaults_16.w); \
  remap[4] = stack_load_float_default(stack, so.ellipse_width_1, defaults_17.x); \
  remap[5] = stack_load_float_default(stack, so.inflection_point_1, defaults_17.y); \
  remap_randomness[0] = stack_load_float_default( \
      stack, so.step_center_randomness_1, defaults_22.x); \
  remap_randomness[1] = stack_load_float_default( \
      stack, so.step_width_randomness_1, defaults_22.y); \
  remap_randomness[2] = stack_load_float_default( \
      stack, so.step_value_randomness_1, defaults_22.z); \
  remap_randomness[3] = stack_load_float_default( \
      stack, so.ellipse_height_randomness_1, defaults_22.w); \
  remap_randomness[4] = stack_load_float_default( \
      stack, so.ellipse_width_randomness_1, defaults_23.x); \
  remap_randomness[5] = stack_load_float_default( \
      stack, so.inflection_point_randomness_1, defaults_23.y); \
  if (remap_randomness[0] != 0.0f) { \
    remap_min[index_count] = float_max(remap[0] - remap_randomness[0], 0.0f); \
    remap_max[index_count] = float_max(remap[0] + remap_randomness[0], 0.0f); \
    remap_index_list[index_count] = 0; \
    ++index_count; \
  } \
  if (remap_randomness[1] != 0.0f) { \
    remap_min[index_count] = float_max(remap[1] - remap_randomness[1], 0.0f); \
    remap_max[index_count] = float_max(remap[1] + remap_randomness[1], 0.0f); \
    remap_index_list[index_count] = 1; \
    ++index_count; \
  } \
  if (remap_randomness[2] != 0.0f) { \
    remap_min[index_count] = remap[2] - remap_randomness[2]; \
    remap_max[index_count] = remap[2] + remap_randomness[2]; \
    remap_index_list[index_count] = 2; \
    ++index_count; \
  } \
  if (remap_randomness[3] != 0.0f) { \
    remap_min[index_count] = clamp(remap[3] - remap_randomness[3], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[3] + remap_randomness[3], 0.0f, 1.0f); \
    remap_index_list[index_count] = 3; \
    ++index_count; \
  } \
  if (remap_randomness[4] != 0.0f) { \
    remap_min[index_count] = clamp(remap[4] - remap_randomness[4], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[4] + remap_randomness[4], 0.0f, 1.0f); \
    remap_index_list[index_count] = 4; \
    ++index_count; \
  } \
  if (remap_randomness[5] != 0.0f) { \
    remap_min[index_count] = clamp(remap[5] - remap_randomness[5], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[5] + remap_randomness[5], 0.0f, 1.0f); \
    remap_index_list[index_count] = 5; \
    ++index_count; \
  }

#define ASSIGN_REMAP_INPUTS_2 \
  remap[6] = stack_load_float_default(stack, so.step_center_2, defaults_17.z); \
  remap[7] = stack_load_float_default(stack, so.step_width_2, defaults_17.w); \
  remap[8] = stack_load_float_default(stack, so.step_value_2, defaults_18.x); \
  remap[9] = stack_load_float_default(stack, so.ellipse_height_2, defaults_18.y); \
  remap[10] = stack_load_float_default(stack, so.ellipse_width_2, defaults_18.z); \
  remap[11] = stack_load_float_default(stack, so.inflection_point_2, defaults_18.w); \
  remap_randomness[6] = stack_load_float_default( \
      stack, so.step_center_randomness_2, defaults_23.z); \
  remap_randomness[7] = stack_load_float_default( \
      stack, so.step_width_randomness_2, defaults_23.w); \
  remap_randomness[8] = stack_load_float_default( \
      stack, so.step_value_randomness_2, defaults_24.x); \
  remap_randomness[9] = stack_load_float_default( \
      stack, so.ellipse_height_randomness_2, defaults_24.y); \
  remap_randomness[10] = stack_load_float_default( \
      stack, so.ellipse_width_randomness_2, defaults_24.z); \
  remap_randomness[11] = stack_load_float_default( \
      stack, so.inflection_point_randomness_2, defaults_24.w); \
  if (remap_randomness[6] != 0.0f) { \
    remap_min[index_count] = float_max(remap[6] - remap_randomness[6], 0.0f); \
    remap_max[index_count] = float_max(remap[6] + remap_randomness[6], 0.0f); \
    remap_index_list[index_count] = 6; \
    ++index_count; \
  } \
  if (remap_randomness[7] != 0.0f) { \
    remap_min[index_count] = float_max(remap[7] - remap_randomness[7], 0.0f); \
    remap_max[index_count] = float_max(remap[7] + remap_randomness[7], 0.0f); \
    remap_index_list[index_count] = 7; \
    ++index_count; \
  } \
  if (remap_randomness[8] != 0.0f) { \
    remap_min[index_count] = remap[8] - remap_randomness[8]; \
    remap_max[index_count] = remap[8] + remap_randomness[8]; \
    remap_index_list[index_count] = 8; \
    ++index_count; \
  } \
  if (remap_randomness[9] != 0.0f) { \
    remap_min[index_count] = clamp(remap[9] - remap_randomness[9], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[9] + remap_randomness[9], 0.0f, 1.0f); \
    remap_index_list[index_count] = 9; \
    ++index_count; \
  } \
  if (remap_randomness[10] != 0.0f) { \
    remap_min[index_count] = clamp(remap[10] - remap_randomness[10], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[10] + remap_randomness[10], 0.0f, 1.0f); \
    remap_index_list[index_count] = 10; \
    ++index_count; \
  } \
  if (remap_randomness[11] != 0.0f) { \
    remap_min[index_count] = clamp(remap[11] - remap_randomness[11], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[11] + remap_randomness[11], 0.0f, 1.0f); \
    remap_index_list[index_count] = 11; \
    ++index_count; \
  }

#define ASSIGN_REMAP_INPUTS_3 \
  remap[12] = stack_load_float_default(stack, so.step_center_3, defaults_19.x); \
  remap[13] = stack_load_float_default(stack, so.step_width_3, defaults_19.y); \
  remap[14] = stack_load_float_default(stack, so.step_value_3, defaults_19.z); \
  remap[15] = stack_load_float_default(stack, so.ellipse_height_3, defaults_19.w); \
  remap[16] = stack_load_float_default(stack, so.ellipse_width_3, defaults_20.x); \
  remap[17] = stack_load_float_default(stack, so.inflection_point_3, defaults_20.y); \
  remap_randomness[12] = stack_load_float_default( \
      stack, so.step_center_randomness_3, defaults_25.x); \
  remap_randomness[13] = stack_load_float_default( \
      stack, so.step_width_randomness_3, defaults_25.y); \
  remap_randomness[14] = stack_load_float_default( \
      stack, so.step_value_randomness_3, defaults_25.z); \
  remap_randomness[15] = stack_load_float_default( \
      stack, so.ellipse_height_randomness_3, defaults_25.w); \
  remap_randomness[16] = stack_load_float_default( \
      stack, so.ellipse_width_randomness_3, defaults_26.x); \
  remap_randomness[17] = stack_load_float_default( \
      stack, so.inflection_point_randomness_3, defaults_26.y); \
  if (remap_randomness[12] != 0.0f) { \
    remap_min[index_count] = float_max(remap[12] - remap_randomness[12], 0.0f); \
    remap_max[index_count] = float_max(remap[12] + remap_randomness[12], 0.0f); \
    remap_index_list[index_count] = 12; \
    ++index_count; \
  } \
  if (remap_randomness[13] != 0.0f) { \
    remap_min[index_count] = float_max(remap[13] - remap_randomness[13], 0.0f); \
    remap_max[index_count] = float_max(remap[13] + remap_randomness[13], 0.0f); \
    remap_index_list[index_count] = 13; \
    ++index_count; \
  } \
  if (remap_randomness[14] != 0.0f) { \
    remap_min[index_count] = remap[14] - remap_randomness[14]; \
    remap_max[index_count] = remap[14] + remap_randomness[14]; \
    remap_index_list[index_count] = 14; \
    ++index_count; \
  } \
  if (remap_randomness[15] != 0.0f) { \
    remap_min[index_count] = clamp(remap[15] - remap_randomness[15], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[15] + remap_randomness[15], 0.0f, 1.0f); \
    remap_index_list[index_count] = 15; \
    ++index_count; \
  } \
  if (remap_randomness[16] != 0.0f) { \
    remap_min[index_count] = clamp(remap[16] - remap_randomness[16], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[16] + remap_randomness[16], 0.0f, 1.0f); \
    remap_index_list[index_count] = 16; \
    ++index_count; \
  } \
  if (remap_randomness[17] != 0.0f) { \
    remap_min[index_count] = clamp(remap[17] - remap_randomness[17], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[17] + remap_randomness[17], 0.0f, 1.0f); \
    remap_index_list[index_count] = 17; \
    ++index_count; \
  }

#define ASSIGN_REMAP_INPUTS_4 \
  remap[18] = stack_load_float_default(stack, so.step_center_4, defaults_20.z); \
  remap[19] = stack_load_float_default(stack, so.step_width_4, defaults_20.w); \
  remap[20] = stack_load_float_default(stack, so.step_value_4, defaults_21.x); \
  remap[21] = stack_load_float_default(stack, so.ellipse_height_4, defaults_21.y); \
  remap[22] = stack_load_float_default(stack, so.ellipse_width_4, defaults_21.z); \
  remap[23] = stack_load_float_default(stack, so.inflection_point_4, defaults_21.w); \
  remap_randomness[18] = stack_load_float_default( \
      stack, so.step_center_randomness_4, defaults_26.z); \
  remap_randomness[19] = stack_load_float_default( \
      stack, so.step_width_randomness_4, defaults_26.w); \
  remap_randomness[20] = stack_load_float_default( \
      stack, so.step_value_randomness_4, defaults_27.x); \
  remap_randomness[21] = stack_load_float_default( \
      stack, so.ellipse_height_randomness_4, defaults_27.y); \
  remap_randomness[22] = stack_load_float_default( \
      stack, so.ellipse_width_randomness_4, defaults_27.z); \
  remap_randomness[23] = stack_load_float_default( \
      stack, so.inflection_point_randomness_4, defaults_27.w); \
  if (remap_randomness[18] != 0.0f) { \
    remap_min[index_count] = float_max(remap[18] - remap_randomness[18], 0.0f); \
    remap_max[index_count] = float_max(remap[18] + remap_randomness[18], 0.0f); \
    remap_index_list[index_count] = 18; \
    ++index_count; \
  } \
  if (remap_randomness[19] != 0.0f) { \
    remap_min[index_count] = float_max(remap[19] - remap_randomness[19], 0.0f); \
    remap_max[index_count] = float_max(remap[19] + remap_randomness[19], 0.0f); \
    remap_index_list[index_count] = 19; \
    ++index_count; \
  } \
  if (remap_randomness[20] != 0.0f) { \
    remap_min[index_count] = remap[20] - remap_randomness[20]; \
    remap_max[index_count] = remap[20] + remap_randomness[20]; \
    remap_index_list[index_count] = 20; \
    ++index_count; \
  } \
  if (remap_randomness[21] != 0.0f) { \
    remap_min[index_count] = clamp(remap[21] - remap_randomness[21], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[21] + remap_randomness[21], 0.0f, 1.0f); \
    remap_index_list[index_count] = 21; \
    ++index_count; \
  } \
  if (remap_randomness[22] != 0.0f) { \
    remap_min[index_count] = clamp(remap[22] - remap_randomness[22], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[22] + remap_randomness[22], 0.0f, 1.0f); \
    remap_index_list[index_count] = 22; \
    ++index_count; \
  } \
  if (remap_randomness[23] != 0.0f) { \
    remap_min[index_count] = clamp(remap[23] - remap_randomness[23], 0.0f, 1.0f); \
    remap_max[index_count] = clamp(remap[23] + remap_randomness[23], 0.0f, 1.0f); \
    remap_index_list[index_count] = 23; \
    ++index_count; \
  }

struct DeterministicVariables {
  NodeRaikoMode mode;
  bool normalize_r_gon_parameter;
  float4 coord;
  float scale;
  float smoothness;
  float accuracy;
  bool integer_sides;
  bool elliptical_corners;
  bool invert_order_of_transformation;
  bool transform_fields_noise;
  bool transform_coordinates_noise;
  bool uniform_scale_randomness;
  float noise_fragmentation;
  float noise_fields_strength_1;
  float noise_coordinates_strength_1;
  float noise_scale_1;
  float noise_detail_1;
  float noise_roughness_1;
  float noise_lacunarity_1;
  float noise_fields_strength_2;
  float noise_coordinates_strength_2;
  float noise_scale_2;
  float noise_detail_2;
  float noise_roughness_2;
  float noise_lacunarity_2;
  int grid_dimensions;
  float4 grid_vector_1;
  float4 grid_vector_2;
  float4 grid_vector_3;
  float4 grid_vector_4;
  int step_count;

  bool calculate_r_sphere_field;
  bool calculate_r_gon_parameter_field;
  bool calculate_max_unit_parameter_field;
  bool calculate_coordinates_outputs;

  bool smoothness_non_zero;
  bool noise_fragmentation_non_zero;
  bool calculate_fields_noise_1;
  bool calculate_fields_noise_2;
  bool calculate_coordinates_noise_1;
  bool calculate_coordinates_noise_2;
};

struct OutVariables {
  float out_r_sphere_field;
  float r_gon_parameter_field;
  float max_unit_parameter_field;
  float segment_id_field;
  float4 out_index_field;
  float4 out_position_field;
  float4 out_r_sphere_coordinates;
};

struct StackOffsets {
  uint vector;
  uint w;
  uint accuracy;
  uint scale;
  uint smoothness;
  uint r_gon_sides;
  uint r_gon_roundness;
  uint r_gon_exponent;
  uint sphere_exponent;
  uint r_gon_sides_randomness;
  uint r_gon_roundness_randomness;
  uint r_gon_exponent_randomness;
  uint sphere_exponent_randomness;
  uint transform_rotation;
  uint transform_scale;
  uint transform_scale_w;
  uint transform_rotation_randomness;
  uint transform_scale_randomness;
  uint transform_scale_w_randomness;
  uint noise_fragmentation;
  uint noise_fields_strength_1;
  uint noise_coordinates_strength_1;
  uint noise_scale_1;
  uint noise_detail_1;
  uint noise_roughness_1;
  uint noise_lacunarity_1;
  uint noise_fields_strength_2;
  uint noise_coordinates_strength_2;
  uint noise_scale_2;
  uint noise_detail_2;
  uint noise_roughness_2;
  uint noise_lacunarity_2;
  uint grid_vector_1;
  uint grid_vector_w_1;
  uint grid_vector_2;
  uint grid_vector_w_2;
  uint grid_vector_3;
  uint grid_vector_w_3;
  uint grid_vector_4;
  uint grid_vector_w_4;
  uint grid_points_translation_randomness;
  uint grid_points_translation_w_randomness;
  uint step_center_1;
  uint step_width_1;
  uint step_value_1;
  uint ellipse_height_1;
  uint ellipse_width_1;
  uint inflection_point_1;
  uint step_center_2;
  uint step_width_2;
  uint step_value_2;
  uint ellipse_height_2;
  uint ellipse_width_2;
  uint inflection_point_2;
  uint step_center_3;
  uint step_width_3;
  uint step_value_3;
  uint ellipse_height_3;
  uint ellipse_width_3;
  uint inflection_point_3;
  uint step_center_4;
  uint step_width_4;
  uint step_value_4;
  uint ellipse_height_4;
  uint ellipse_width_4;
  uint inflection_point_4;
  uint step_center_randomness_1;
  uint step_width_randomness_1;
  uint step_value_randomness_1;
  uint ellipse_height_randomness_1;
  uint ellipse_width_randomness_1;
  uint inflection_point_randomness_1;
  uint step_center_randomness_2;
  uint step_width_randomness_2;
  uint step_value_randomness_2;
  uint ellipse_height_randomness_2;
  uint ellipse_width_randomness_2;
  uint inflection_point_randomness_2;
  uint step_center_randomness_3;
  uint step_width_randomness_3;
  uint step_value_randomness_3;
  uint ellipse_height_randomness_3;
  uint ellipse_width_randomness_3;
  uint inflection_point_randomness_3;
  uint step_center_randomness_4;
  uint step_width_randomness_4;
  uint step_value_randomness_4;
  uint ellipse_height_randomness_4;
  uint ellipse_width_randomness_4;
  uint inflection_point_randomness_4;
  uint r_sphere_field;
  uint r_gon_parameter_field;
  uint max_unit_parameter_field;
  uint segment_id_field;
  uint index_field;
  uint index_field_w;
  uint position_field;
  uint position_field_w;
  uint r_sphere_coordinates;
  uint r_sphere_coordinates_w;
};

/* Fast Cramer Coefficients of the 3x3 matrix M. */
struct Fcc_3x3 {
  /* Determinant of the 3x3 matrix M. */
  float M_det;
  /* Determinant of the 2x2 submatrices M_i_j. The first index i denotes the row, the second index
   * j the coloumn being removed from M. */
  float M_1_1_det;
  float M_2_1_det;
  float M_3_1_det;
};

/* Fast Cramer Coefficients of the 4x4 matrix M. */
struct Fcc_4x4 {
  /* Determinant of the 4x4 matrix M. */
  float M_det;
  /* Fast Cramer Coefficients of the 3x3 submatrices M_i_j. The first index i denotes the row, the
   * second index j the coloumn being removed from M. */
  Fcc_3x3 M_1_1_fcc;
  Fcc_3x3 M_2_1_fcc;
  Fcc_3x3 M_3_1_fcc;
  Fcc_3x3 M_4_1_fcc;
};

ccl_device Fcc_3x3 calculate_Fcc_3x3(float3 a_1, float3 a_2, float3 a_3)
{
  Fcc_3x3 A_fcc;

  A_fcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_fcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_fcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_det - a_1.y * A_fcc.M_2_1_det + a_1.z * A_fcc.M_3_1_det;

  return A_fcc;
}

ccl_device Fcc_4x4 calculate_Fcc_4x4(float4 a_1, float4 a_2, float4 a_3, float4 a_4)
{
  Fcc_4x4 A_fcc;

  A_fcc.M_1_1_fcc = calculate_Fcc_3x3(make_float3(a_2.y, a_2.z, a_2.w),
                                      make_float3(a_3.y, a_3.z, a_3.w),
                                      make_float3(a_4.y, a_4.z, a_4.w));
  A_fcc.M_2_1_fcc = calculate_Fcc_3x3(make_float3(a_2.x, a_2.z, a_2.w),
                                      make_float3(a_3.x, a_3.z, a_3.w),
                                      make_float3(a_4.x, a_4.z, a_4.w));
  A_fcc.M_3_1_fcc = calculate_Fcc_3x3(make_float3(a_2.x, a_2.y, a_2.w),
                                      make_float3(a_3.x, a_3.y, a_3.w),
                                      make_float3(a_4.x, a_4.y, a_4.w));
  A_fcc.M_4_1_fcc = calculate_Fcc_3x3(make_float3(a_2.x, a_2.y, a_2.z),
                                      make_float3(a_3.x, a_3.y, a_3.z),
                                      make_float3(a_4.x, a_4.y, a_4.z));

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_fcc.M_det - a_1.y * A_fcc.M_2_1_fcc.M_det +
                a_1.z * A_fcc.M_3_1_fcc.M_det - a_1.w * A_fcc.M_4_1_fcc.M_det;

  return A_fcc;
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 2x2 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float2 fc_linear_system_solve_non_singular_2x2(float2 a_1,
                                                          float2 a_2,
                                                          float2 b,
                                                          float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is faster. */
  return make_float2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 3x3 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float3 fc_linear_system_solve_non_singular_3x3(
    float3 a_1, float3 a_2, float3 a_3, float3 b, Fcc_3x3 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_det - b.y * A_fcc.M_2_1_det + b.z * A_fcc.M_3_1_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_det != 0.0f) {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.y, a_2.z),
        make_float2(a_3.y, a_3.z),
        make_float2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_1_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_fcc.M_2_1_det != 0.0f) {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.x, a_2.z),
        make_float2(a_3.x, a_3.z),
        make_float2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_2_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    float2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.x, a_2.y),
        make_float2(a_3.x, a_3.y),
        make_float2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_fcc.M_3_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 4x4 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float4 fc_linear_system_solve_non_singular_4x4(
    float4 a_1, float4 a_2, float4 a_3, float4 a_4, float4 b, Fcc_4x4 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_fcc.M_det - b.y * A_fcc.M_2_1_fcc.M_det +
                      b.z * A_fcc.M_3_1_fcc.M_det - b.w * A_fcc.M_4_1_fcc.M_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.y, a_2.z, a_2.w),
        make_float3(a_3.y, a_3.z, a_3.w),
        make_float3(a_4.y, a_4.z, a_4.w),
        make_float3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_1_1_fcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_2_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.z, a_2.w),
        make_float3(a_3.x, a_3.z, a_3.w),
        make_float3(a_4.x, a_4.z, a_4.w),
        make_float3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_2_1_fcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_3_1_fcc.M_det != 0.0f) {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.y, a_2.w),
        make_float3(a_3.x, a_3.y, a_3.w),
        make_float3(a_4.x, a_4.y, a_4.w),
        make_float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_3_1_fcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    float3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.y, a_2.z),
        make_float3(a_3.x, a_3.y, a_3.z),
        make_float3(a_4.x, a_4.y, a_4.z),
        make_float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_4_1_fcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

ccl_device float chebychev_norm(float coord)
{
  return fabsf(coord);
}

ccl_device float chebychev_norm(float2 coord)
{
  return float_max(fabsf(coord.x), fabsf(coord.y));
}

ccl_device float chebychev_norm(float3 coord)
{
  return float_max(fabsf(coord.x), float_max(fabsf(coord.y), fabsf(coord.z)));
}

ccl_device float p_norm(float coord)
{
  return fabsf(coord);
}

ccl_device float p_norm(float2 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ? chebychev_norm(coord) :
                              powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent),
                                   1.0f / exponent);
}

ccl_device float p_norm(float3 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0f) ?
             chebychev_norm(coord) :
             powf(powf(fabsf(coord.x), exponent) + powf(fabsf(coord.y), exponent) +
                      powf(fabsf(coord.z), exponent),
                  1.0f / exponent);
}

ccl_device float euclidean_norm(float4 coord)
{
  return sqrtf(square(coord.x) + square(coord.y) + square(coord.z) + square(coord.w));
}

ccl_device float calculate_l_angle_bisector_2d_full_roundness_irregular_elliptical(
    float r_gon_sides, float2 coord, float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    return l_projection_2d;
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sinf(last_angle_bisector_A_x_axis) *
                             sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
    float2 ellipse_center = make_float2(cosf(ref_A_angle_bisector) /
                                            cosf(ref_A_angle_bisector - ref_A_angle_bisector),
                                        sinf(ref_A_angle_bisector) /
                                            cosf(ref_A_angle_bisector - ref_A_angle_bisector)) -
                            l_basis_vector_2 * make_float2(sinf(last_angle_bisector_A_x_axis),
                                                           cosf(last_angle_bisector_A_x_axis));
    float2 transformed_direction_vector = make_float2(
        cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float2 transformed_origin = make_float2(
        (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sinf(ref_A_angle_bisector) +
          ellipse_center.x * cosf(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrtf(square(transformed_direction_vector.x * transformed_origin.x +
                      transformed_direction_vector.y * transformed_origin.y) -
               (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                   (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
        (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
    return l_projection_2d / l_coord_R_l_angle_bisector_2d;
  }
}

ccl_device float calculate_l_angle_bisector_2d_irregular_elliptical(float r_gon_sides,
                                                                    float r_gon_roundness,
                                                                    float2 coord,
                                                                    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                              sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
      return cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      return l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sinf(last_angle_bisector_A_x_axis) *
                               sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center =
          make_float2(cosf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start),
                      sinf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              make_float2(sinf(last_angle_bisector_A_x_axis), cosf(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = make_float2(
          cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = make_float2(
          (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sinf(ref_A_angle_bisector) +
            ellipse_center.x * cosf(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(
               square(transformed_direction_vector.x * transformed_origin.x +
                      transformed_direction_vector.y * transformed_origin.y) -
               (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                   (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      return l_projection_2d / l_coord_R_l_angle_bisector_2d;
    }
  }
}

ccl_device float calculate_l_angle_bisector_2d_full_roundness_irregular_circular(
    float r_gon_sides, float2 coord, float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      cosf(last_angle_bisector_A_x_axis) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          sinf(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      return l_projection_2d;
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_coord_R_l_last_angle_bisector_2d =
        sinf(nearest_ref_MSA_coord) * last_circle_center.y +
        cosf(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                     cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
              square(l_last_circle_radius) - square(last_circle_center.x) -
              square(last_circle_center.y));
    return (cosf(ref_A_angle_bisector) * l_projection_2d) /
           (cosf(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
  }
}

ccl_device float calculate_l_angle_bisector_2d_irregular_circular(float r_gon_sides,
                                                                  float r_gon_roundness,
                                                                  float2 coord,
                                                                  float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      (cosf(inner_last_bevel_start_A_x_axis) /
       cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          (sinf(inner_last_bevel_start_A_x_axis) /
           cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
      float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                              sinf(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                      sqrtf(square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
      return cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                             cosf(last_angle_bisector_A_x_axis);
      return l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_coord_R_l_last_angle_bisector_2d =
          sinf(nearest_ref_MSA_coord) * last_circle_center.y +
          cosf(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                       cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
                square(l_last_circle_radius) - square(last_circle_center.x) -
                square(last_circle_center.y));
      return (cosf(ref_A_angle_bisector) * l_projection_2d) /
             (cosf(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
    }
  }
}

ccl_device float calculate_l_angle_bisector_2d(bool integer_sides,
                                               bool elliptical_corners,
                                               float r_gon_sides,
                                               float r_gon_roundness,
                                               float r_gon_exponent,
                                               float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fractf(r_gon_sides) == 0.0f)) {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
    }
    if (r_gon_roundness == 1.0f) {
      return l_projection_2d;
    }
    else {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
        float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                                sinf(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                        sqrtf(
                                            square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
        return cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
               l_coord_R_l_bevel_start;
      }
      else {
        return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        return l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
      }
      else {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        return l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
    }
    if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_l_angle_bisector_2d_full_roundness_irregular_elliptical(
            r_gon_sides, coord, l_projection_2d);
      }
      else {
        return calculate_l_angle_bisector_2d_full_roundness_irregular_circular(
            r_gon_sides, coord, l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_l_angle_bisector_2d_irregular_elliptical(
            r_gon_sides, r_gon_roundness, coord, l_projection_2d);
      }
      else {
        return calculate_l_angle_bisector_2d_irregular_circular(
            r_gon_sides, r_gon_roundness, coord, l_projection_2d);
      }
    }
  }
}

ccl_device float calculate_l_angle_bisector_4d(bool integer_sides,
                                               bool elliptical_corners,
                                               float r_gon_sides,
                                               float r_gon_roundness,
                                               float r_gon_exponent,
                                               float sphere_exponent,
                                               float4 coord)
{
  return p_norm(
      make_float3(calculate_l_angle_bisector_2d(integer_sides,
                                                elliptical_corners,
                                                integer_sides ? fceilf(r_gon_sides) : r_gon_sides,
                                                r_gon_roundness,
                                                r_gon_exponent,
                                                make_float2(coord.x, coord.y)),
                  coord.z,
                  coord.w),
      sphere_exponent);
}

ccl_device float4
calculate_out_fields_2d_full_roundness_irregular_elliptical(bool calculate_r_sphere_field,
                                                            bool calculate_r_gon_parameter_field,
                                                            bool normalize_r_gon_parameter,
                                                            float r_gon_sides,
                                                            float2 coord,
                                                            float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0f;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0f;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return make_float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;
    if (calculate_r_sphere_field) {
      float l_basis_vector_1 = tan(ref_A_angle_bisector);
      /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
       * precision issues. Change to double if necessary */
      float l_basis_vector_2 = sinf(last_angle_bisector_A_x_axis) *
                               sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
      float2 ellipse_center = make_float2(cosf(ref_A_angle_bisector) /
                                              cosf(ref_A_angle_bisector - ref_A_angle_bisector),
                                          sinf(ref_A_angle_bisector) /
                                              cosf(ref_A_angle_bisector - ref_A_angle_bisector)) -
                              l_basis_vector_2 * make_float2(sinf(last_angle_bisector_A_x_axis),
                                                             cosf(last_angle_bisector_A_x_axis));
      float2 transformed_direction_vector = make_float2(
          cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float2 transformed_origin = make_float2(
          (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sinf(ref_A_angle_bisector) +
            ellipse_center.x * cosf(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrtf(
               square(transformed_direction_vector.x * transformed_origin.x +
                      transformed_direction_vector.y * transformed_origin.y) -
               (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                   (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                  l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= ref_A_angle_bisector;
          }
        }
        max_unit_parameter_2d = ref_A_angle_bisector;
      }
    }
    return make_float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

ccl_device float4
calculate_out_fields_2d_irregular_elliptical(bool calculate_r_sphere_field,
                                             bool calculate_r_gon_parameter_field,
                                             bool calculate_max_unit_parameter_field,
                                             bool normalize_r_gon_parameter,
                                             float r_gon_sides,
                                             float r_gon_roundness,
                                             float2 coord,
                                             float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
        float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                                sinf(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                        sqrtf(
                                            square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
        float l_basis_vector_2 = r_gon_roundness * sinf(last_angle_bisector_A_x_axis) *
                                 sqrtf(square(tan(ref_A_angle_bisector)) + 1.0f);
        float2 ellipse_center =
            make_float2(cosf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start),
                        sinf(ref_A_bevel_start) / cosf(ref_A_angle_bisector - ref_A_bevel_start)) -
            l_basis_vector_2 * make_float2(sinf(last_angle_bisector_A_x_axis),
                                           cosf(last_angle_bisector_A_x_axis));
        float2 transformed_direction_vector = make_float2(
            cosf(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
                (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            cosf(ref_A_angle_bisector - nearest_ref_MSA_coord) /
                (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        float2 transformed_origin = make_float2(
            (ellipse_center.y * sinf(last_angle_bisector_A_x_axis) -
             ellipse_center.x * cosf(last_angle_bisector_A_x_axis)) /
                (l_basis_vector_1 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            -(ellipse_center.y * sinf(ref_A_angle_bisector) +
              ellipse_center.x * cosf(ref_A_angle_bisector)) /
                (l_basis_vector_2 * sinf(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        float l_coord_R_l_angle_bisector_2d =
            (-(transformed_direction_vector.x * transformed_origin.x +
               transformed_direction_vector.y * transformed_origin.y) +
             sqrtf(square(transformed_direction_vector.x * transformed_origin.x +
                          transformed_direction_vector.y * transformed_origin.y) -
                   (square(transformed_direction_vector.x) +
                    square(transformed_direction_vector.y)) *
                       (square(transformed_origin.x) + square(transformed_origin.y) - 1.0f))) /
            (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
        l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
        if (nearest_ref_MSA_coord < 0.0f) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                                 cosf(
                                                                     last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(last_angle_bisector_A_x_axis -
                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                              inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                        inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                 ref_A_bevel_start - nearest_ref_MSA_coord;
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

ccl_device float4
calculate_out_fields_2d_full_roundness_irregular_circular(bool calculate_r_sphere_field,
                                                          bool calculate_r_gon_parameter_field,
                                                          bool calculate_max_unit_parameter_field,
                                                          bool normalize_r_gon_parameter,
                                                          float r_gon_sides,
                                                          float2 coord,
                                                          float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      cosf(last_angle_bisector_A_x_axis) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          sinf(last_angle_bisector_A_x_axis));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return make_float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0f;
    float r_gon_parameter_2d = 0.0f;
    float max_unit_parameter_2d = 0.0f;
    if (calculate_r_sphere_field) {
      float l_coord_R_l_last_angle_bisector_2d =
          sinf(nearest_ref_MSA_coord) * last_circle_center.y +
          cosf(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                       cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
                square(l_last_circle_radius) - square(last_circle_center.x) -
                square(last_circle_center.y));
      l_angle_bisector_2d = (cosf(ref_A_angle_bisector) * l_projection_2d) /
                            (cosf(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0f) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= last_angle_bisector_A_x_axis *
                                  l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        max_unit_parameter_2d = last_angle_bisector_A_x_axis *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
      else {
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(fabsf(ref_A_angle_bisector -
                                                               x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                  x_axis_A_outer_last_bevel_start;
        }
      }
    }
    return make_float4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

ccl_device float4
calculate_out_fields_2d_irregular_circular(bool calculate_r_sphere_field,
                                           bool calculate_r_gon_parameter_field,
                                           bool calculate_max_unit_parameter_field,
                                           bool normalize_r_gon_parameter,
                                           float r_gon_sides,
                                           float r_gon_roundness,
                                           float2 coord,
                                           float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
  float ref_A_angle_bisector = M_PI_F / r_gon_sides;
  float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
  float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5f * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  float2 last_circle_center = make_float2(
      (cosf(inner_last_bevel_start_A_x_axis) /
       cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cosf(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sinf(last_angle_bisector_A_x_axis) -
          (sinf(inner_last_bevel_start_A_x_axis) /
           cosf(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  float2 outer_last_bevel_start = last_circle_center +
                                  l_last_circle_radius * make_float2(cosf(ref_A_angle_bisector),
                                                                     sinf(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - ref_A_angle_bisector) ||
            (x_axis_A_coord < ref_A_angle_bisector))
        {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector -
                                                              x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
        float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                                sinf(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                        sqrtf(
                                            square(cosf(nearest_ref_SA_coord) * l_circle_center) +
                                            square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = cosf(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d *
                                      tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                  ref_A_bevel_start;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU_F - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                               cosf(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                            inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                      inner_last_bevel_start_A_x_axis) +
                                  inner_last_bevel_start_A_x_axis *
                                      l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU_F - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU_F - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        float l_coord_R_l_last_angle_bisector_2d =
            sinf(nearest_ref_MSA_coord) * last_circle_center.y +
            cosf(nearest_ref_MSA_coord) * last_circle_center.x +
            sqrtf(square(sinf(nearest_ref_MSA_coord) * last_circle_center.y +
                         cosf(nearest_ref_MSA_coord) * last_circle_center.x) +
                  square(l_last_circle_radius) - square(last_circle_center.x) -
                  square(last_circle_center.y));
        l_angle_bisector_2d = (cosf(ref_A_angle_bisector) * l_projection_2d) /
                              (cosf(last_angle_bisector_A_x_axis) *
                               l_coord_R_l_last_angle_bisector_2d);
        if (nearest_ref_MSA_coord < 0.0f) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                                 cosf(
                                                                     last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(last_angle_bisector_A_x_axis -
                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis -
                                                              inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis -
                                        inner_last_bevel_start_A_x_axis) +
                                    inner_last_bevel_start_A_x_axis *
                                        l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          }
        }
        else {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(fabsf(ref_A_angle_bisector -
                                               x_axis_A_outer_last_bevel_start)) +
                                 x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector -
                                                              x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - x_axis_A_outer_last_bevel_start) +
                                    x_axis_A_outer_last_bevel_start;
          }
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

ccl_device float4 calculate_out_fields_2d(bool calculate_r_sphere_field,
                                          bool calculate_r_gon_parameter_field,
                                          bool calculate_max_unit_parameter_field,
                                          bool normalize_r_gon_parameter,
                                          bool integer_sides,
                                          bool elliptical_corners,
                                          float r_gon_sides,
                                          float r_gon_roundness,
                                          float r_gon_exponent,
                                          float2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fractf(r_gon_sides) == 0.0f)) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
    float ref_A_angle_bisector = M_PI_F / r_gon_sides;
    float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
    float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0f) {
      float l_angle_bisector_2d = 0.0f;
      float r_gon_parameter_2d = 0.0f;
      float max_unit_parameter_2d = 0.0f;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(fabsf(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0f;
          }
          if (normalize_r_gon_parameter && (r_gon_sides != 2.0f)) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = (r_gon_sides != 2.0f) ? tan(ref_A_angle_bisector) : 0.0f;
        }
      }
      return make_float4(
          l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    if (r_gon_roundness == 1.0f) {
      float r_gon_parameter_2d = 0.0f;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = fabsf(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0f;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return make_float4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI_F, M_PI_F]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          float l_circle_radius = sinf(ref_A_bevel_start) / sinf(ref_A_angle_bisector);
          float l_circle_center = sinf(ref_A_angle_bisector - ref_A_bevel_start) /
                                  sinf(ref_A_angle_bisector);
          float l_coord_R_l_bevel_start = cosf(nearest_ref_SA_coord) * l_circle_center +
                                          sqrtf(square(cosf(nearest_ref_SA_coord) *
                                                       l_circle_center) +
                                                square(l_circle_radius) - square(l_circle_center));
          l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_bevel_start) /
                                l_coord_R_l_bevel_start;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                 ref_A_bevel_start - fabsf(nearest_ref_SA_coord);
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
        return make_float4(
            l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d *
                                        tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                    ref_A_bevel_start;
          }
        }
        return make_float4(
            l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0f) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0f) * M_TAU_F;
      float ref_A_angle_bisector = M_PI_F / r_gon_sides;
      float ref_A_next_ref = 2.0f * ref_A_angle_bisector;
      float segment_id = floorf(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI_F - floorf(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0f * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU_F - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cosf(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector);
          }
        }
        return make_float4(
            l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0f;
        float r_gon_parameter_2d = 0.0f;
        float max_unit_parameter_2d = 0.0f;
        if (calculate_r_sphere_field) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cosf(ref_A_angle_bisector) /
                                                                 cosf(
                                                                     last_angle_bisector_A_x_axis);
          l_angle_bisector_2d = l_projection_2d *
                                cosf(last_angle_bisector_A_x_axis - ref_A_coord) *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(fabsf(last_angle_bisector_A_x_axis - ref_A_coord));
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0f;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
          }
        }
        return make_float4(
            l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
    if (r_gon_roundness == 1.0f) {
      if (elliptical_corners) {
        return calculate_out_fields_2d_full_roundness_irregular_elliptical(
            calculate_r_sphere_field,
            calculate_r_gon_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_full_roundness_irregular_circular(
            calculate_r_sphere_field,
            calculate_r_gon_parameter_field,
            calculate_max_unit_parameter_field,
            normalize_r_gon_parameter,
            r_gon_sides,
            coord,
            l_projection_2d);
      }
    }
    else {
      if (elliptical_corners) {
        return calculate_out_fields_2d_irregular_elliptical(calculate_r_sphere_field,
                                                            calculate_r_gon_parameter_field,
                                                            calculate_max_unit_parameter_field,
                                                            normalize_r_gon_parameter,
                                                            r_gon_sides,
                                                            r_gon_roundness,
                                                            coord,
                                                            l_projection_2d);
      }
      else {
        return calculate_out_fields_2d_irregular_circular(calculate_r_sphere_field,
                                                          calculate_r_gon_parameter_field,
                                                          calculate_max_unit_parameter_field,
                                                          normalize_r_gon_parameter,
                                                          r_gon_sides,
                                                          r_gon_roundness,
                                                          coord,
                                                          l_projection_2d);
      }
    }
  }
}

ccl_device float4 calculate_out_fields_4d(bool calculate_r_sphere_field,
                                          bool calculate_r_gon_parameter_field,
                                          bool calculate_max_unit_parameter_field,
                                          bool normalize_r_gon_parameter,
                                          bool integer_sides,
                                          bool elliptical_corners,
                                          float r_gon_sides,
                                          float r_gon_roundness,
                                          float r_gon_exponent,
                                          float sphere_exponent,
                                          float4 coord)
{
  float4 out_fields = calculate_out_fields_2d(calculate_r_sphere_field,
                                              calculate_r_gon_parameter_field,
                                              calculate_max_unit_parameter_field,
                                              normalize_r_gon_parameter,
                                              integer_sides,
                                              elliptical_corners,
                                              integer_sides ? fceilf(r_gon_sides) : r_gon_sides,
                                              r_gon_roundness,
                                              r_gon_exponent,
                                              make_float2(coord.x, coord.y));
  out_fields.x = p_norm(make_float3(out_fields.x, coord.z, coord.w), sphere_exponent);
  return out_fields;
}

ccl_device void randomize_scale(float scale_randomized[],
                                float scale_randomness[],
                                int scale_index_list[],
                                int scale_index_count,
                                bool uniform_scale_randomness,
                                float3 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= powf(
          2.0f,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_float3_to_float(
                  make_float3(scale_index_list[i], scale_index_list[i], scale_index_list[i]) +
                  seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = powf(
        2.0f, mix(-scale_randomness[0], scale_randomness[0], hash_float3_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

ccl_device void randomize_scale(float scale_randomized[],
                                float scale_randomness[],
                                int scale_index_list[],
                                int scale_index_count,
                                bool uniform_scale_randomness,
                                float4 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= powf(
          2.0f,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_float4_to_float(make_float4(scale_index_list[i],
                                               scale_index_list[i],
                                               scale_index_list[i],
                                               scale_index_list[i]) +
                                   seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = powf(
        2.0f, mix(-scale_randomness[0], scale_randomness[0], hash_float4_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

ccl_device void randomize_float_array(
    float array[], float min[], float max[], int index_list[], int index_count, float3 seed_offset)
{
  for (int i = 0; i < index_count; ++i) {
    array[index_list[i]] = mix(
        min[i],
        max[i],
        hash_float3_to_float(make_float3(index_list[i], index_list[i], index_list[i]) +
                             seed_offset));
  }
}

ccl_device void randomize_float_array(
    float array[], float min[], float max[], int index_list[], int index_count, float4 seed_offset)
{
  for (int i = 0; i < index_count; ++i) {
    array[index_list[i]] = mix(
        min[i],
        max[i],
        hash_float4_to_float(
            make_float4(index_list[i], index_list[i], index_list[i], index_list[i]) +
            seed_offset));
  }
}

ccl_device float elliptical_ramp(float value, float ellipse_height, float ellipse_width)
{
  if (value < 0.0f) {
    return 0.0f;
  }
  else if (value < ellipse_width + ellipse_height * (1.0f - ellipse_width)) {
    return (ellipse_height *
            (value * ellipse_height * (1.0f - ellipse_width) + square(ellipse_width) -
             ellipse_width * sqrtf(square(ellipse_width) - square(value) +
                                   2.0f * value * ellipse_height * (1.0f - ellipse_width)))) /
           (square(ellipse_height * (1.0f - ellipse_width)) + square(ellipse_width));
  }
  else {
    return (ellipse_width == 1.0f) ? ellipse_height :
                                     (value - ellipse_width) / (1.0f - ellipse_width);
  }
}

ccl_device float elliptical_unit_step(float value,
                                      float ellipse_height,
                                      float ellipse_width,
                                      float inflection_point)
{
  if (inflection_point == 0.0f) {
    return (value < 0.0f) ? 0.0f :
                            1.0f - elliptical_ramp(1.0f - value, ellipse_height, ellipse_width);
  }
  else if (inflection_point == 1.0f) {
    return (value < 1.0f) ? elliptical_ramp(value, ellipse_height, ellipse_width) : 1.0f;
  }
  else {
    return (value < inflection_point) ?
               inflection_point *
                   elliptical_ramp(value / inflection_point, ellipse_height, ellipse_width) :
               1.0f - (1.0f - inflection_point) *
                          elliptical_ramp((1.0f - value) / (1.0f - inflection_point),
                                          ellipse_height,
                                          ellipse_width);
  }
}

ccl_device float inverse_mix(float value, float from_min, float from_max)
{
  return (value - from_min) / (from_max - from_min);
}

ccl_device float elliptical_remap(float value,
                                  float from_min,
                                  float from_max,
                                  float to_min,
                                  float to_max,
                                  float ellipse_height,
                                  float ellipse_width,
                                  float inflection_point)
{
  if (from_min == from_max) {
    return (value >= from_min) ? to_max : to_min;
  }
  else {
    return mix(to_min,
               to_max,
               elliptical_unit_step(inverse_mix(value, from_min, from_max),
                                    ellipse_height,
                                    ellipse_width,
                                    inflection_point));
  }
}

ccl_device float chained_elliptical_remap_1_step(float remap[], float value)
{
  return elliptical_remap(value,
                          /* Step Center 1 - 0.5f * Step Width 1 */
                          remap[0] - 0.5f * remap[1],
                          /* Step Center 1 + 0.5f * Step Width 1 */
                          remap[0] + 0.5f * remap[1],
                          /* Step Value 1 */
                          remap[2],
                          0.0f,
                          /* Ellipse Height 1 */
                          remap[3],
                          /* Ellipse Width 1 */
                          remap[4],
                          /* Inflection Point 1 */
                          remap[5]);
}

ccl_device float chained_elliptical_remap_2_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  return elliptical_remap(value,
                          /* Step Center 2 - 0.5f * Step Width 2 */
                          remap[6] - 0.5f * remap[7],
                          /* Step Center 2 + 0.5f * Step Width 2 */
                          remap[6] + 0.5f * remap[7],
                          result,
                          0.0f,
                          /* Ellipse Height 2 */
                          remap[9],
                          /* Ellipse Width 2 */
                          remap[10],
                          /* Inflection Point 2 */
                          remap[11]);
}

ccl_device float chained_elliptical_remap_3_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  result = elliptical_remap(value,
                            /* Step Center 2 - 0.5f * Step Width 2 */
                            remap[6] - 0.5f * remap[7],
                            /* Step Center 2 + 0.5f * Step Width 2 */
                            remap[6] + 0.5f * remap[7],
                            result,
                            /* Step Value 3 */
                            remap[14],
                            /* Ellipse Height 2 */
                            remap[9],
                            /* Ellipse Width 2 */
                            remap[10],
                            /* Inflection Point 2 */
                            remap[11]);
  return elliptical_remap(value,
                          /* Step Center 3 - 0.5f * Step Width 3 */
                          remap[12] - 0.5f * remap[13],
                          /* Step Center 3 + 0.5f * Step Width 3 */
                          remap[12] + 0.5f * remap[13],
                          result,
                          0.0f,
                          /* Ellipse Height 3 */
                          remap[15],
                          /* Ellipse Width 3 */
                          remap[16],
                          /* Inflection Point 3 */
                          remap[17]);
}

ccl_device float chained_elliptical_remap_4_steps(float remap[], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5f * Step Width 1 */
                                  remap[0] - 0.5f * remap[1],
                                  /* Step Center 1 + 0.5f * Step Width 1 */
                                  remap[0] + 0.5f * remap[1],
                                  /* Step Value 1 */
                                  remap[2],
                                  /* Step Value 2 */
                                  remap[8],
                                  /* Ellipse Height 1 */
                                  remap[3],
                                  /* Ellipse Width 1 */
                                  remap[4],
                                  /* Inflection Point 1 */
                                  remap[5]);
  result = elliptical_remap(value,
                            /* Step Center 2 - 0.5f * Step Width 2 */
                            remap[6] - 0.5f * remap[7],
                            /* Step Center 2 + 0.5f * Step Width 2 */
                            remap[6] + 0.5f * remap[7],
                            result,
                            /* Step Value 3 */
                            remap[14],
                            /* Ellipse Height 2 */
                            remap[9],
                            /* Ellipse Width 2 */
                            remap[10],
                            /* Inflection Point 2 */
                            remap[11]);
  result = elliptical_remap(value,
                            /* Step Center 3 - 0.5f * Step Width 3 */
                            remap[12] - 0.5f * remap[13],
                            /* Step Center 3 + 0.5f * Step Width 3 */
                            remap[12] + 0.5f * remap[13],
                            result,
                            /* Step Value 4 */
                            remap[20],
                            /* Ellipse Height 3 */
                            remap[15],
                            /* Ellipse Width 3 */
                            remap[16],
                            /* Inflection Point 3 */
                            remap[17]);
  return elliptical_remap(value,
                          /* Step Center 4 - 0.5f * Step Width 4 */
                          remap[18] - 0.5f * remap[19],
                          /* Step Center 4 + 0.5f * Step Width 4 */
                          remap[18] + 0.5f * remap[19],
                          result,
                          0.0f,
                          /* Ellipse Height 4 */
                          remap[21],
                          /* Ellipse Width 4 */
                          remap[22],
                          /* Inflection Point 4 */
                          remap[23]);
}

template<typename T>
ccl_device float chained_elliptical_remap_select_steps(int step_count,
                                                       float remap[],
                                                       float remap_min[],
                                                       float remap_max[],
                                                       int remap_index_list[],
                                                       int remap_index_count,
                                                       T seed_offset,
                                                       float value)
{
  float remap_randomized[24];
  float result;
  switch (step_count) {
    case 1: {
      for (int i = 0; i < 6; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_1_step(remap_randomized, value);
      break;
    }
    case 2: {
      for (int i = 0; i < 12; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_2_steps(remap_randomized, value);
      break;
    }
    case 3: {
      for (int i = 0; i < 18; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_3_steps(remap_randomized, value);
      break;
    }
    case 4: {
      for (int i = 0; i < 24; ++i) {
        remap_randomized[i] = remap[i];
      }
      randomize_float_array(remap_randomized,
                            remap_min,
                            remap_max,
                            remap_index_list,
                            remap_index_count,
                            seed_offset);
      result = chained_elliptical_remap_4_steps(remap_randomized, value);
      break;
    }
  }
  return result;
}

ccl_device float4 rotate_scale(float4 coord,
                               float translation_rotation_randomized[7],
                               float scale_randomized[4],
                               bool invert_order_of_transformation)
{
  if (invert_order_of_transformation) {
    coord = make_float4(scale_randomized[0],
                        scale_randomized[1],
                        scale_randomized[2],
                        scale_randomized[3]) *
            coord;

    if (translation_rotation_randomized[4] != 0.0f) {
      coord = make_float4(coord.x,
                          cosf(translation_rotation_randomized[4]) * coord.y -
                              sinf(translation_rotation_randomized[4]) * coord.z,
                          sinf(translation_rotation_randomized[4]) * coord.y +
                              cosf(translation_rotation_randomized[4]) * coord.z,
                          coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0f) {
      coord = make_float4(cosf(translation_rotation_randomized[5]) * coord.x +
                              sinf(translation_rotation_randomized[5]) * coord.z,
                          coord.y,
                          cosf(translation_rotation_randomized[5]) * coord.z -
                              sinf(translation_rotation_randomized[5]) * coord.x,
                          coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0f) {
      coord = make_float4(cosf(translation_rotation_randomized[6]) * coord.x -
                              sinf(translation_rotation_randomized[6]) * coord.y,
                          sinf(translation_rotation_randomized[6]) * coord.x +
                              cosf(translation_rotation_randomized[6]) * coord.y,
                          coord.z,
                          coord.w);
    }
  }
  else {
    if (translation_rotation_randomized[4] != 0.0f) {
      coord = make_float4(coord.x,
                          cosf(translation_rotation_randomized[4]) * coord.y -
                              sinf(translation_rotation_randomized[4]) * coord.z,
                          sinf(translation_rotation_randomized[4]) * coord.y +
                              cosf(translation_rotation_randomized[4]) * coord.z,
                          coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0f) {
      coord = make_float4(cosf(translation_rotation_randomized[5]) * coord.x +
                              sinf(translation_rotation_randomized[5]) * coord.z,
                          coord.y,
                          cosf(translation_rotation_randomized[5]) * coord.z -
                              sinf(translation_rotation_randomized[5]) * coord.x,
                          coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0f) {
      coord = make_float4(cosf(translation_rotation_randomized[6]) * coord.x -
                              sinf(translation_rotation_randomized[6]) * coord.y,
                          sinf(translation_rotation_randomized[6]) * coord.x +
                              cosf(translation_rotation_randomized[6]) * coord.y,
                          coord.z,
                          coord.w);
    }

    coord = make_float4(scale_randomized[0],
                        scale_randomized[1],
                        scale_randomized[2],
                        scale_randomized[3]) *
            coord;
  }

  return coord;
}

/* Noise Texture fBM optimized for Raiko Texture. */
ccl_device float raiko_noise_fbm(float4 coord, float detail, float roughness, float lacunarity)
{
  float octave_scale = 1.0f;
  float amplitude = 1.0f;
  float max_amplitude = 1.0f;
  float sum = 0.0f;

  for (int i = 0; i <= int(detail); ++i) {
    sum += amplitude * snoise_4d(octave_scale * coord);
    max_amplitude += amplitude;
    amplitude *= roughness;
    octave_scale *= lacunarity;
  }

  float remainder = detail - floorf(detail);
  return (remainder != 0.0f) ? ((sum + remainder * amplitude * snoise_4d(octave_scale * coord)) /
                                (max_amplitude + remainder * amplitude)) :
                               (sum / max_amplitude);
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_1(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0f */
ccl_device float4 raiko_noise_fbm_layer_1(ccl_private const DeterministicVariables &dv,
                                          float4 coord)
{
  return make_float4(
      raiko_noise_fbm(dv.noise_scale_1 * coord +
                          make_float4(178.498459f, 183.790161f, 114.143784f, 163.889908f),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord +
                          make_float4(147.634079f, 195.179962f, 158.144135f, 128.116669f),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord +
                          make_float4(195.063629f, 144.612671f, 155.014709f, 165.883881f),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord +
                          make_float4(115.671997f, 104.330322f, 135.032425f, 120.330460f),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1));
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_2(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0f */
ccl_device float4 raiko_noise_fbm_layer_2(ccl_private const DeterministicVariables &dv,
                                          float4 coord)
{
  return make_float4(
      raiko_noise_fbm(dv.noise_scale_2 * coord +
                          make_float4(115.225372f, 181.849701f, 148.865616f, 148.047165f),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord +
                          make_float4(132.636856f, 169.415527f, 110.008087f, 130.162735f),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord +
                          make_float4(187.223145f, 167.974121f, 156.358246f, 121.253998f),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord +
                          make_float4(119.618362f, 126.933167f, 161.577881f, 147.723999f),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2));
}

ccl_device float4 rotate_noise(float4 noise_vector, float noise_fragmentation, float3 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU_F *
                              (floored_modulo(index.x + 0.0625f, 7.0f) +
                               3.0f * floored_modulo(index.x + 0.0625f, 2.0f) +
                               13.0f * (floored_modulo(index.y + 0.0625f, 7.0f) +
                                        3.0f * floored_modulo(index.y + 0.0625f, 2.0f)) +
                               143.0f * (floored_modulo(index.z + 0.0625f, 7.0f) +
                                         3.0f * floored_modulo(index.z + 0.0625f, 2.0f)));
  float4 noise_vector_rotated = make_float4(
      cosf(deterministic_angle) * noise_vector.x - sinf(deterministic_angle) * noise_vector.y,
      sinf(deterministic_angle) * noise_vector.x + cosf(deterministic_angle) * noise_vector.y,
      cosf(deterministic_angle) * noise_vector.z - sinf(deterministic_angle) * noise_vector.w,
      sinf(deterministic_angle) * noise_vector.z + cosf(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU_F * 5.0f * hash_float3_to_float(index);
  return make_float4(
      cosf(random_angle) * noise_vector_rotated.x - sinf(random_angle) * noise_vector_rotated.z,
      cosf(random_angle) * noise_vector_rotated.y - sinf(random_angle) * noise_vector_rotated.w,
      sinf(random_angle) * noise_vector_rotated.x + cosf(random_angle) * noise_vector_rotated.z,
      sinf(random_angle) * noise_vector_rotated.y + cosf(random_angle) * noise_vector_rotated.w);
}

ccl_device float4 rotate_noise(float4 noise_vector, float noise_fragmentation, float4 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU_F *
                              (floored_modulo(index.x + 0.0625f, 7.0f) +
                               3.0f * floored_modulo(index.x + 0.0625f, 2.0f) +
                               13.0f * (floored_modulo(index.y + 0.0625f, 7.0f) +
                                        3.0f * floored_modulo(index.y + 0.0625f, 2.0f)) +
                               143.0f * (floored_modulo(index.z + 0.0625f, 7.0f) +
                                         3.0f * floored_modulo(index.z + 0.0625f, 2.0f)) +
                               2431.0f * (floored_modulo(index.w + 0.0625f, 7.0f) +
                                          3.0f * floored_modulo(index.w + 0.0625f, 2.0f)));
  float4 noise_vector_rotated = make_float4(
      cosf(deterministic_angle) * noise_vector.x - sinf(deterministic_angle) * noise_vector.y,
      sinf(deterministic_angle) * noise_vector.x + cosf(deterministic_angle) * noise_vector.y,
      cosf(deterministic_angle) * noise_vector.z - sinf(deterministic_angle) * noise_vector.w,
      sinf(deterministic_angle) * noise_vector.z + cosf(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU_F * 5.0f * hash_float4_to_float(index);
  return make_float4(
      cosf(random_angle) * noise_vector_rotated.x - sinf(random_angle) * noise_vector_rotated.z,
      cosf(random_angle) * noise_vector_rotated.y - sinf(random_angle) * noise_vector_rotated.w,
      sinf(random_angle) * noise_vector_rotated.x + cosf(random_angle) * noise_vector_rotated.z,
      sinf(random_angle) * noise_vector_rotated.y + cosf(random_angle) * noise_vector_rotated.w);
}

ccl_device OutVariables raiko_select_mode_0d(ccl_private const DeterministicVariables &dv,
                                             float r_sphere[],
                                             float r_sphere_min[],
                                             float r_sphere_max[],
                                             int r_sphere_index_list[],
                                             int r_sphere_index_count,
                                             float translation_rotation[],
                                             float translation_rotation_min[],
                                             float translation_rotation_max[],
                                             int translation_rotation_index_list[],
                                             int translation_rotation_index_count,
                                             float scale[],
                                             float scale_randomness[],
                                             int scale_index_list[],
                                             int scale_index_count,
                                             float remap[],
                                             float remap_min[],
                                             float remap_max[],
                                             int remap_index_list[],
                                             int remap_index_count)
{
  OutVariables ov;
  float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                    raiko_noise_fbm_layer_1(
                                        dv,
                                        dv.transform_fields_noise ?
                                            rotate_scale(dv.coord,
                                                         translation_rotation,
                                                         scale,
                                                         dv.invert_order_of_transformation) :
                                            dv.coord) :
                                    make_float4(0.0f, 0.0f, 0.0f, 0.0f);
  float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                    raiko_noise_fbm_layer_2(
                                        dv,
                                        dv.transform_fields_noise ?
                                            rotate_scale(dv.coord,
                                                         translation_rotation,
                                                         scale,
                                                         dv.invert_order_of_transformation) :
                                            dv.coord) :
                                    make_float4(0.0f, 0.0f, 0.0f, 0.0f);
  float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                               dv.noise_fields_strength_2 * fields_noise_layer_2;

  float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
  randomize_float_array(r_sphere_randomized,
                        r_sphere_min,
                        r_sphere_max,
                        r_sphere_index_list,
                        r_sphere_index_count,
                        make_float3(0.0f, 0.0f, 0.0f));
  float translation_rotation_randomized[7];
  for (int n = 0; n < 7; ++n) {
    translation_rotation_randomized[n] = translation_rotation[n];
  }
  randomize_float_array(translation_rotation_randomized,
                        translation_rotation_min,
                        translation_rotation_max,
                        translation_rotation_index_list,
                        translation_rotation_index_count,
                        make_float3(0.0f, 0.0f, 0.0f));
  float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
  randomize_scale(scale_randomized,
                  scale_randomness,
                  scale_index_list,
                  scale_index_count,
                  dv.uniform_scale_randomness,
                  make_float3(0.0f, 0.0f, 0.0f));

  float4 iteration_position = make_float4(translation_rotation_randomized[0],
                                          translation_rotation_randomized[1],
                                          translation_rotation_randomized[2],
                                          translation_rotation_randomized[3]);
  float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                        translation_rotation_randomized,
                                        scale_randomized,
                                        dv.invert_order_of_transformation);
  float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                  rotate_noise(fields_noise_vector,
                                                               dv.noise_fragmentation,
                                                               make_float3(0.0f, 0.0f, 0.0f)) :
                                                  fields_noise_vector);

  if (dv.mode == NODE_RAIKO_ADDITIVE) {
    ov.out_r_sphere_field = chained_elliptical_remap_select_steps(
        dv.step_count,
        remap,
        remap_min,
        remap_max,
        remap_index_list,
        remap_index_count,
        make_float3(0.0f, 0.0f, 0.0f),
        calculate_l_angle_bisector_4d(dv.integer_sides,
                                      dv.elliptical_corners,
                                      r_sphere_randomized[0],
                                      r_sphere_randomized[1],
                                      r_sphere_randomized[2],
                                      r_sphere_randomized[3],
                                      iteration_coord));
  }
  else {
    float4 coordinates_noise_layer_1 =
        dv.calculate_coordinates_noise_1 ?
            raiko_noise_fbm_layer_1(
                dv,
                dv.transform_coordinates_noise ?
                    rotate_scale(
                        dv.coord, translation_rotation, scale, dv.invert_order_of_transformation) :
                    dv.coord) :
            fields_noise_layer_1;
    float4 coordinates_noise_layer_2 =
        dv.calculate_coordinates_noise_2 ?
            raiko_noise_fbm_layer_2(
                dv,
                dv.transform_coordinates_noise ?
                    rotate_scale(
                        dv.coord, translation_rotation, scale, dv.invert_order_of_transformation) :
                    dv.coord) :
            fields_noise_layer_2;
    float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

    float4 iteration_r_sphere_coordinates = noiseless_coord +
                                            (dv.noise_fragmentation_non_zero ?
                                                 rotate_noise(coordinates_noise_vector,
                                                              dv.noise_fragmentation,
                                                              make_float3(0.0f, 0.0f, 0.0f)) :
                                                 coordinates_noise_vector);

    float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                dv.calculate_r_gon_parameter_field,
                                                dv.calculate_max_unit_parameter_field,
                                                dv.normalize_r_gon_parameter,
                                                dv.integer_sides,
                                                dv.elliptical_corners,
                                                r_sphere_randomized[0],
                                                r_sphere_randomized[1],
                                                r_sphere_randomized[2],
                                                r_sphere_randomized[3],
                                                iteration_coord);

    ov.out_r_sphere_field = out_fields.x;
    ov.r_gon_parameter_field = out_fields.y;
    ov.max_unit_parameter_field = out_fields.z;
    ov.segment_id_field = out_fields.w;
    ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
  }
  return ov;
}

ccl_device OutVariables raiko_select_mode_1d(ccl_private const DeterministicVariables &dv,
                                             float r_sphere[],
                                             float r_sphere_min[],
                                             float r_sphere_max[],
                                             int r_sphere_index_list[],
                                             int r_sphere_index_count,
                                             float translation_rotation[],
                                             float translation_rotation_min[],
                                             float translation_rotation_max[],
                                             int translation_rotation_index_list[],
                                             int translation_rotation_index_count,
                                             float scale[],
                                             float scale_randomness[],
                                             int scale_index_list[],
                                             int scale_index_count,
                                             float remap[],
                                             float remap_min[],
                                             float remap_max[],
                                             int remap_index_list[],
                                             int remap_index_count,
                                             float4 initial_index,
                                             float4 initial_position)
{
  float scanning_window_size = fceilf(4.0f * dv.accuracy);
  OutVariables ov;
  switch (dv.mode) {
    case NODE_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = make_float3(i, 0.0f, 0.0f) +
                                 make_float3(initial_index.x, 0.0f, 0.0f);
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0f * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    make_float4(translation_rotation_randomized[0],
                                                translation_rotation_randomized[1],
                                                translation_rotation_randomized[2],
                                                translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(fields_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        fields_noise_vector);

        ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
            dv.step_count,
            remap,
            remap_min,
            remap_max,
            remap_index_list,
            remap_index_count,
            27.0f * iteration_index,
            calculate_l_angle_bisector_4d(dv.integer_sides,
                                          dv.elliptical_corners,
                                          r_sphere_randomized[0],
                                          r_sphere_randomized[1],
                                          r_sphere_randomized[2],
                                          r_sphere_randomized[3],
                                          iteration_coord));
      }
      break;
    }
    case NODE_RAIKO_CLOSEST: {
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];

      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = make_float3(i, 0.0f, 0.0f) +
                                 make_float3(initial_index.x, 0.0f, 0.0f);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    make_float4(translation_rotation_randomized[0],
                                                translation_rotation_randomized[1],
                                                translation_rotation_randomized[2],
                                                translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(index_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        index_noise_vector);

        float l_iteration_coord = euclidean_norm(iteration_coord);
        if (l_iteration_coord < min_distance) {
          min_distance = l_iteration_coord;
          ov.out_index_field.x = iteration_index.x;
          ov.out_position_field = iteration_position;
          /* Translation data not needed for subsequent computations. */
          closest_rotation_randomized[4] = translation_rotation_randomized[4];
          closest_rotation_randomized[5] = translation_rotation_randomized[5];
          closest_rotation_randomized[6] = translation_rotation_randomized[6];
          closest_scale_randomized[0] = scale_randomized[0];
          closest_scale_randomized[1] = scale_randomized[1];
          closest_scale_randomized[2] = scale_randomized[2];
          closest_scale_randomized[3] = scale_randomized[3];
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates =
          noiseless_coord +
          (dv.noise_fragmentation_non_zero ?
               rotate_noise(closest_coordinates_noise_vector,
                            dv.noise_fragmentation,
                            7.0f * make_float3(ov.out_index_field.x, 0.0f, 0.0f)) :
               closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * make_float3(ov.out_index_field.x, 0.0f, 0.0f));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord +
              (dv.noise_fragmentation_non_zero ?
                   rotate_noise(closest_fields_noise_vector,
                                dv.noise_fragmentation,
                                7.0f * make_float3(ov.out_index_field.x, 0.0f, 0.0f)) :
                   closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case NODE_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float3 iteration_index = make_float3(i, 0.0f, 0.0f) +
                                 make_float3(initial_index.x, 0.0f, 0.0f);
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0f * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0f * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0f * iteration_index);

        float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    make_float4(translation_rotation_randomized[0],
                                                translation_rotation_randomized[1],
                                                translation_rotation_randomized[2],
                                                translation_rotation_randomized[3]);
        float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
        float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(fields_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        fields_noise_vector);
        float4 iteration_r_sphere_coordinates =
            noiseless_coord +
            (dv.noise_fragmentation_non_zero ?
                 rotate_noise(coordinates_noise_vector, dv.noise_fragmentation, iteration_index) :
                 coordinates_noise_vector);

        float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                    dv.calculate_r_gon_parameter_field,
                                                    dv.calculate_max_unit_parameter_field,
                                                    dv.normalize_r_gon_parameter,
                                                    dv.integer_sides,
                                                    dv.elliptical_corners,
                                                    r_sphere_randomized[0],
                                                    r_sphere_randomized[1],
                                                    r_sphere_randomized[2],
                                                    r_sphere_randomized[3],
                                                    iteration_coord);
        float iteration_l_angle_bisector_4d = out_fields.x;
        float iteration_r_gon_parameter = out_fields.y;
        float iteration_max_unit_parameter = out_fields.z;
        float iteration_segment_id = out_fields.w;

        if (dv.smoothness_non_zero) {
          float interpolation_factor = first_iteration ?
                                           1.0f :
                                           smoothstep(0.0f,
                                                      1.0f,
                                                      0.5f + 0.5f *
                                                                 (ov.out_r_sphere_field -
                                                                  iteration_l_angle_bisector_4d) /
                                                                 dv.smoothness);
          float substraction_factor = dv.smoothness * interpolation_factor *
                                      (1.0f - interpolation_factor);
          ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                      iteration_l_angle_bisector_4d,
                                      interpolation_factor) -
                                  substraction_factor;

          ov.r_gon_parameter_field = mix(
              ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
          ov.max_unit_parameter_field = mix(
              ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
          ov.segment_id_field = mix(
              ov.segment_id_field, iteration_segment_id, interpolation_factor);
          ov.out_index_field.x = mix(
              ov.out_index_field.x, iteration_index.x, interpolation_factor);
          ov.out_position_field = mix(
              ov.out_position_field, iteration_position, interpolation_factor);
          ov.out_r_sphere_coordinates = mix(
              ov.out_r_sphere_coordinates, iteration_r_sphere_coordinates, interpolation_factor);
        }
        else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
          ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
          ov.r_gon_parameter_field = iteration_r_gon_parameter;
          ov.max_unit_parameter_field = iteration_max_unit_parameter;
          ov.segment_id_field = iteration_segment_id;
          ov.out_index_field.x = iteration_index.x;
          ov.out_position_field = iteration_position;
          ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
        }
        first_iteration = false;
      }
      break;
    }
  }
  return ov;
}

ccl_device OutVariables raiko_select_mode_2d(ccl_private const DeterministicVariables &dv,
                                             float r_sphere[],
                                             float r_sphere_min[],
                                             float r_sphere_max[],
                                             int r_sphere_index_list[],
                                             int r_sphere_index_count,
                                             float translation_rotation[],
                                             float translation_rotation_min[],
                                             float translation_rotation_max[],
                                             int translation_rotation_index_list[],
                                             int translation_rotation_index_count,
                                             float scale[],
                                             float scale_randomness[],
                                             int scale_index_list[],
                                             int scale_index_count,
                                             float remap[],
                                             float remap_min[],
                                             float remap_max[],
                                             int remap_index_list[],
                                             int remap_index_count,
                                             float4 initial_index,
                                             float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_shortest_grid_vector = float_min(l_grid_vector1, l_grid_vector2);
  float2 scanning_window_size = fceilf(4.0f * dv.accuracy *
                                       make_float2(l_shortest_grid_vector / l_grid_vector1,
                                                   l_shortest_grid_vector / l_grid_vector2));
  OutVariables ov;
  switch (dv.mode) {
    case NODE_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = make_float3(i, j, 0.0f) +
                                   make_float3(initial_index.x, initial_index.y, 0.0f);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0f * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      make_float4(translation_rotation_randomized[0],
                                                  translation_rotation_randomized[1],
                                                  translation_rotation_randomized[2],
                                                  translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(fields_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          fields_noise_vector);

          ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
              dv.step_count,
              remap,
              remap_min,
              remap_max,
              remap_index_list,
              remap_index_count,
              27.0f * iteration_index,
              calculate_l_angle_bisector_4d(dv.integer_sides,
                                            dv.elliptical_corners,
                                            r_sphere_randomized[0],
                                            r_sphere_randomized[1],
                                            r_sphere_randomized[2],
                                            r_sphere_randomized[3],
                                            iteration_coord));
        }
      }
      break;
    }
    case NODE_RAIKO_CLOSEST: {
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = make_float3(i, j, 0.0f) +
                                   make_float3(initial_index.x, initial_index.y, 0.0f);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      make_float4(translation_rotation_randomized[0],
                                                  translation_rotation_randomized[1],
                                                  translation_rotation_randomized[2],
                                                  translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(index_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          index_noise_vector);

          float l_iteration_coord = euclidean_norm(iteration_coord);
          if (l_iteration_coord < min_distance) {
            min_distance = l_iteration_coord;
            ov.out_index_field.x = iteration_index.x;
            ov.out_index_field.y = iteration_index.y;
            ov.out_position_field = iteration_position;
            /* Translation data not needed for subsequent computations. */
            closest_rotation_randomized[4] = translation_rotation_randomized[4];
            closest_rotation_randomized[5] = translation_rotation_randomized[5];
            closest_rotation_randomized[6] = translation_rotation_randomized[6];
            closest_scale_randomized[0] = scale_randomized[0];
            closest_scale_randomized[1] = scale_randomized[1];
            closest_scale_randomized[2] = scale_randomized[2];
            closest_scale_randomized[3] = scale_randomized[3];
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates =
          noiseless_coord +
          (dv.noise_fragmentation_non_zero ?
               rotate_noise(closest_coordinates_noise_vector,
                            dv.noise_fragmentation,
                            7.0f * make_float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f)) :
               closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * make_float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord +
              (dv.noise_fragmentation_non_zero ?
                   rotate_noise(
                       closest_fields_noise_vector,
                       dv.noise_fragmentation,
                       7.0f * make_float3(ov.out_index_field.x, ov.out_index_field.y, 0.0f)) :
                   closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case NODE_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          float3 iteration_index = make_float3(i, j, 0.0f) +
                                   make_float3(initial_index.x, initial_index.y, 0.0f);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0f * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0f * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0f * iteration_index);

          float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 +
                                      make_float4(translation_rotation_randomized[0],
                                                  translation_rotation_randomized[1],
                                                  translation_rotation_randomized[2],
                                                  translation_rotation_randomized[3]);
          float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
          float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(fields_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          fields_noise_vector);
          float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                  (dv.noise_fragmentation_non_zero ?
                                                       rotate_noise(coordinates_noise_vector,
                                                                    dv.noise_fragmentation,
                                                                    iteration_index) :
                                                       coordinates_noise_vector);

          float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                      dv.calculate_r_gon_parameter_field,
                                                      dv.calculate_max_unit_parameter_field,
                                                      dv.normalize_r_gon_parameter,
                                                      dv.integer_sides,
                                                      dv.elliptical_corners,
                                                      r_sphere_randomized[0],
                                                      r_sphere_randomized[1],
                                                      r_sphere_randomized[2],
                                                      r_sphere_randomized[3],
                                                      iteration_coord);
          float iteration_l_angle_bisector_4d = out_fields.x;
          float iteration_r_gon_parameter = out_fields.y;
          float iteration_max_unit_parameter = out_fields.z;
          float iteration_segment_id = out_fields.w;

          if (dv.smoothness_non_zero) {
            float interpolation_factor =
                first_iteration ?
                    1.0f :
                    smoothstep(0.0f,
                               1.0f,
                               0.5f + 0.5f *
                                          (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                          dv.smoothness);
            float substraction_factor = dv.smoothness * interpolation_factor *
                                        (1.0f - interpolation_factor);
            ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                        iteration_l_angle_bisector_4d,
                                        interpolation_factor) -
                                    substraction_factor;

            ov.r_gon_parameter_field = mix(
                ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
            ov.max_unit_parameter_field = mix(
                ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
            ov.segment_id_field = mix(
                ov.segment_id_field, iteration_segment_id, interpolation_factor);
            ov.out_index_field.x = mix(
                ov.out_index_field.x, iteration_index.x, interpolation_factor);
            ov.out_index_field.y = mix(
                ov.out_index_field.y, iteration_index.y, interpolation_factor);
            ov.out_position_field = mix(
                ov.out_position_field, iteration_position, interpolation_factor);
            ov.out_r_sphere_coordinates = mix(
                ov.out_r_sphere_coordinates, iteration_r_sphere_coordinates, interpolation_factor);
          }
          else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
            ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
            ov.r_gon_parameter_field = iteration_r_gon_parameter;
            ov.max_unit_parameter_field = iteration_max_unit_parameter;
            ov.segment_id_field = iteration_segment_id;
            ov.out_index_field.x = iteration_index.x;
            ov.out_index_field.y = iteration_index.y;
            ov.out_position_field = iteration_position;
            ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
          }
          first_iteration = false;
        }
      }
      break;
    }
  }
  return ov;
}

ccl_device OutVariables raiko_select_mode_3d(ccl_private const DeterministicVariables &dv,
                                             float r_sphere[],
                                             float r_sphere_min[],
                                             float r_sphere_max[],
                                             int r_sphere_index_list[],
                                             int r_sphere_index_count,
                                             float translation_rotation[],
                                             float translation_rotation_min[],
                                             float translation_rotation_max[],
                                             int translation_rotation_index_list[],
                                             int translation_rotation_index_count,
                                             float scale[],
                                             float scale_randomness[],
                                             int scale_index_list[],
                                             int scale_index_count,
                                             float remap[],
                                             float remap_min[],
                                             float remap_max[],
                                             int remap_index_list[],
                                             int remap_index_count,
                                             float4 initial_index,
                                             float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_shortest_grid_vector = float_min(l_grid_vector1,
                                           float_min(l_grid_vector2, l_grid_vector3));
  float3 scanning_window_size = fceilf(4.0f * dv.accuracy *
                                       make_float3(l_shortest_grid_vector / l_grid_vector1,
                                                   l_shortest_grid_vector / l_grid_vector2,
                                                   l_shortest_grid_vector / l_grid_vector3));
  OutVariables ov;
  switch (dv.mode) {
    case NODE_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = make_float3(i, j, k) + make_float3(initial_index.x,
                                                                        initial_index.y,
                                                                        initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0f * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        make_float4(translation_rotation_randomized[0],
                                                    translation_rotation_randomized[1],
                                                    translation_rotation_randomized[2],
                                                    translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(fields_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            fields_noise_vector);

            ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
                dv.step_count,
                remap,
                remap_min,
                remap_max,
                remap_index_list,
                remap_index_count,
                27.0f * iteration_index,
                calculate_l_angle_bisector_4d(dv.integer_sides,
                                              dv.elliptical_corners,
                                              r_sphere_randomized[0],
                                              r_sphere_randomized[1],
                                              r_sphere_randomized[2],
                                              r_sphere_randomized[3],
                                              iteration_coord));
          }
        }
      }
      break;
    }
    case NODE_RAIKO_CLOSEST: {
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = make_float3(i, j, k) + make_float3(initial_index.x,
                                                                        initial_index.y,
                                                                        initial_index.z);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        make_float4(translation_rotation_randomized[0],
                                                    translation_rotation_randomized[1],
                                                    translation_rotation_randomized[2],
                                                    translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(index_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            index_noise_vector);

            float l_iteration_coord = euclidean_norm(iteration_coord);
            if (l_iteration_coord < min_distance) {
              min_distance = l_iteration_coord;
              ov.out_index_field.x = iteration_index.x;
              ov.out_index_field.y = iteration_index.y;
              ov.out_index_field.z = iteration_index.z;
              ov.out_position_field = iteration_position;
              /* Translation data not needed for subsequent computations. */
              closest_rotation_randomized[4] = translation_rotation_randomized[4];
              closest_rotation_randomized[5] = translation_rotation_randomized[5];
              closest_rotation_randomized[6] = translation_rotation_randomized[6];
              closest_scale_randomized[0] = scale_randomized[0];
              closest_scale_randomized[1] = scale_randomized[1];
              closest_scale_randomized[2] = scale_randomized[2];
              closest_scale_randomized[3] = scale_randomized[3];
            }
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0f * make_float3(ov.out_index_field.x,
                                                                         ov.out_index_field.y,
                                                                         ov.out_index_field.z)) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(
          r_sphere_randomized,
          r_sphere_min,
          r_sphere_max,
          r_sphere_index_list,
          r_sphere_index_count,
          5.0f * make_float3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z));

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_fields_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * make_float3(ov.out_index_field.x,
                                                                 ov.out_index_field.y,
                                                                 ov.out_index_field.z)) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case NODE_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            float3 iteration_index = make_float3(i, j, k) + make_float3(initial_index.x,
                                                                        initial_index.y,
                                                                        initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0f * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0f * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0f * iteration_index);

            float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        make_float4(translation_rotation_randomized[0],
                                                    translation_rotation_randomized[1],
                                                    translation_rotation_randomized[2],
                                                    translation_rotation_randomized[3]);
            float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
            float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(fields_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            fields_noise_vector);
            float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                    (dv.noise_fragmentation_non_zero ?
                                                         rotate_noise(coordinates_noise_vector,
                                                                      dv.noise_fragmentation,
                                                                      iteration_index) :
                                                         coordinates_noise_vector);

            float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                        dv.calculate_r_gon_parameter_field,
                                                        dv.calculate_max_unit_parameter_field,
                                                        dv.normalize_r_gon_parameter,
                                                        dv.integer_sides,
                                                        dv.elliptical_corners,
                                                        r_sphere_randomized[0],
                                                        r_sphere_randomized[1],
                                                        r_sphere_randomized[2],
                                                        r_sphere_randomized[3],
                                                        iteration_coord);
            float iteration_l_angle_bisector_4d = out_fields.x;
            float iteration_r_gon_parameter = out_fields.y;
            float iteration_max_unit_parameter = out_fields.z;
            float iteration_segment_id = out_fields.w;

            if (dv.smoothness_non_zero) {
              float interpolation_factor =
                  first_iteration ?
                      1.0f :
                      smoothstep(
                          0.0f,
                          1.0f,
                          0.5f + 0.5f * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                     dv.smoothness);
              float substraction_factor = dv.smoothness * interpolation_factor *
                                          (1.0f - interpolation_factor);
              ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                          iteration_l_angle_bisector_4d,
                                          interpolation_factor) -
                                      substraction_factor;

              ov.r_gon_parameter_field = mix(
                  ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
              ov.max_unit_parameter_field = mix(
                  ov.max_unit_parameter_field, iteration_max_unit_parameter, interpolation_factor);
              ov.segment_id_field = mix(
                  ov.segment_id_field, iteration_segment_id, interpolation_factor);
              ov.out_index_field.x = mix(
                  ov.out_index_field.x, iteration_index.x, interpolation_factor);
              ov.out_index_field.y = mix(
                  ov.out_index_field.y, iteration_index.y, interpolation_factor);
              ov.out_index_field.z = mix(
                  ov.out_index_field.z, iteration_index.z, interpolation_factor);
              ov.out_position_field = mix(
                  ov.out_position_field, iteration_position, interpolation_factor);
              ov.out_r_sphere_coordinates = mix(ov.out_r_sphere_coordinates,
                                                iteration_r_sphere_coordinates,
                                                interpolation_factor);
            }
            else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration) {
              ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
              ov.r_gon_parameter_field = iteration_r_gon_parameter;
              ov.max_unit_parameter_field = iteration_max_unit_parameter;
              ov.segment_id_field = iteration_segment_id;
              ov.out_index_field.x = iteration_index.x;
              ov.out_index_field.y = iteration_index.y;
              ov.out_index_field.z = iteration_index.z;
              ov.out_position_field = iteration_position;
              ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
            }
            first_iteration = false;
          }
        }
      }
      break;
    }
  }
  return ov;
}

ccl_device OutVariables raiko_select_mode_4d(ccl_private const DeterministicVariables &dv,
                                             float r_sphere[],
                                             float r_sphere_min[],
                                             float r_sphere_max[],
                                             int r_sphere_index_list[],
                                             int r_sphere_index_count,
                                             float translation_rotation[],
                                             float translation_rotation_min[],
                                             float translation_rotation_max[],
                                             int translation_rotation_index_list[],
                                             int translation_rotation_index_count,
                                             float scale[],
                                             float scale_randomness[],
                                             int scale_index_list[],
                                             int scale_index_count,
                                             float remap[],
                                             float remap_min[],
                                             float remap_max[],
                                             int remap_index_list[],
                                             int remap_index_count,
                                             float4 initial_index,
                                             float4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_grid_vector4 = euclidean_norm(dv.grid_vector_4);
  float l_shortest_grid_vector = float_min(
      l_grid_vector1, float_min(l_grid_vector2, float_min(l_grid_vector3, l_grid_vector4)));
  float4 scanning_window_size = fceilf(4.0f * dv.accuracy *
                                       make_float4(l_shortest_grid_vector / l_grid_vector1,
                                                   l_shortest_grid_vector / l_grid_vector2,
                                                   l_shortest_grid_vector / l_grid_vector3,
                                                   l_shortest_grid_vector / l_grid_vector4));
  OutVariables ov;
  switch (dv.mode) {
    case NODE_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0f;
      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              float4 iteration_index = make_float4(i, j, k, l) + initial_index;
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0f * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          make_float4(translation_rotation_randomized[0],
                                                      translation_rotation_randomized[1],
                                                      translation_rotation_randomized[2],
                                                      translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(fields_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              fields_noise_vector);

              ov.out_r_sphere_field += chained_elliptical_remap_select_steps(
                  dv.step_count,
                  remap,
                  remap_min,
                  remap_max,
                  remap_index_list,
                  remap_index_count,
                  27.0f * iteration_index,
                  calculate_l_angle_bisector_4d(dv.integer_sides,
                                                dv.elliptical_corners,
                                                r_sphere_randomized[0],
                                                r_sphere_randomized[1],
                                                r_sphere_randomized[2],
                                                r_sphere_randomized[3],
                                                iteration_coord));
            }
          }
        }
      }
      break;
    }
    case NODE_RAIKO_CLOSEST: {
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      float4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                       raiko_noise_fbm_layer_1(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                       raiko_noise_fbm_layer_2(
                                           dv,
                                           dv.transform_fields_noise ?
                                               rotate_scale(dv.coord,
                                                            translation_rotation,
                                                            scale,
                                                            dv.invert_order_of_transformation) :
                                               dv.coord) :
                                       make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                  dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float4 iteration_index = make_float4(i, j, k, l) + initial_index;
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          make_float4(translation_rotation_randomized[0],
                                                      translation_rotation_randomized[1],
                                                      translation_rotation_randomized[2],
                                                      translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(index_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              index_noise_vector);

              float l_iteration_coord = euclidean_norm(iteration_coord);
              if (l_iteration_coord < min_distance) {
                min_distance = l_iteration_coord;
                ov.out_index_field = iteration_index;
                ov.out_position_field = iteration_position;
                /* Translation data not needed for subsequent computations. */
                closest_rotation_randomized[4] = translation_rotation_randomized[4];
                closest_rotation_randomized[5] = translation_rotation_randomized[5];
                closest_rotation_randomized[6] = translation_rotation_randomized[6];
                closest_scale_randomized[0] = scale_randomized[0];
                closest_scale_randomized[1] = scale_randomized[1];
                closest_scale_randomized[2] = scale_randomized[2];
                closest_scale_randomized[3] = scale_randomized[3];
              }
            }
          }
        }
      }

      float4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                               closest_fields_noise_layer_1 +
                                           dv.noise_fields_strength_2 *
                                               closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_1;
      float4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field) :
              closest_fields_noise_layer_2;
      float4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                    closest_coordinates_noise_layer_1 +
                                                dv.noise_coordinates_strength_2 *
                                                    closest_coordinates_noise_layer_2;

      float4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                            closest_rotation_randomized,
                                            closest_scale_randomized,
                                            dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0f * ov.out_index_field) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0f * ov.out_index_field);

      float4 out_fields = calculate_out_fields_4d(
          dv.calculate_r_sphere_field,
          dv.calculate_r_gon_parameter_field,
          dv.calculate_max_unit_parameter_field,
          dv.normalize_r_gon_parameter,
          dv.integer_sides,
          dv.elliptical_corners,
          r_sphere_randomized[0],
          r_sphere_randomized[1],
          r_sphere_randomized[2],
          r_sphere_randomized[3],
          noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                 rotate_noise(closest_fields_noise_vector,
                                              dv.noise_fragmentation,
                                              7.0f * ov.out_index_field) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case NODE_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0f;
      ov.r_gon_parameter_field = 0.0f;
      ov.max_unit_parameter_field = 0.0f;
      ov.segment_id_field = 0.0f;
      ov.out_r_sphere_coordinates = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_index_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      ov.out_position_field = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      bool first_iteration = true;

      float4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                        raiko_noise_fbm_layer_1(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                        raiko_noise_fbm_layer_2(
                                            dv,
                                            dv.transform_fields_noise ?
                                                rotate_scale(dv.coord,
                                                             translation_rotation,
                                                             scale,
                                                             dv.invert_order_of_transformation) :
                                                dv.coord) :
                                        make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      float4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                   dv.noise_fields_strength_2 * fields_noise_layer_2;
      float4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      float4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      float4 coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                            coordinates_noise_layer_1 +
                                        dv.noise_coordinates_strength_2 *
                                            coordinates_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float4 iteration_index = make_float4(i, j, k, l) + initial_index;
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0f * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0f * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0f * iteration_index);

              float4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                          j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                          l * dv.grid_vector_4 +
                                          make_float4(translation_rotation_randomized[0],
                                                      translation_rotation_randomized[1],
                                                      translation_rotation_randomized[2],
                                                      translation_rotation_randomized[3]);
              float4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                    translation_rotation_randomized,
                                                    scale_randomized,
                                                    dv.invert_order_of_transformation);
              float4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                              rotate_noise(fields_noise_vector,
                                                                           dv.noise_fragmentation,
                                                                           iteration_index) :
                                                              fields_noise_vector);
              float4 iteration_r_sphere_coordinates = noiseless_coord +
                                                      (dv.noise_fragmentation_non_zero ?
                                                           rotate_noise(coordinates_noise_vector,
                                                                        dv.noise_fragmentation,
                                                                        iteration_index) :
                                                           coordinates_noise_vector);

              float4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
                                                          dv.calculate_r_gon_parameter_field,
                                                          dv.calculate_max_unit_parameter_field,
                                                          dv.normalize_r_gon_parameter,
                                                          dv.integer_sides,
                                                          dv.elliptical_corners,
                                                          r_sphere_randomized[0],
                                                          r_sphere_randomized[1],
                                                          r_sphere_randomized[2],
                                                          r_sphere_randomized[3],
                                                          iteration_coord);
              float iteration_l_angle_bisector_4d = out_fields.x;
              float iteration_r_gon_parameter = out_fields.y;
              float iteration_max_unit_parameter = out_fields.z;
              float iteration_segment_id = out_fields.w;

              if (dv.smoothness_non_zero) {
                float interpolation_factor =
                    first_iteration ?
                        1.0f :
                        smoothstep(
                            0.0f,
                            1.0f,
                            0.5f + 0.5f * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                       dv.smoothness);
                float substraction_factor = dv.smoothness * interpolation_factor *
                                            (1.0f - interpolation_factor);
                ov.out_r_sphere_field = mix(ov.out_r_sphere_field,
                                            iteration_l_angle_bisector_4d,
                                            interpolation_factor) -
                                        substraction_factor;

                ov.r_gon_parameter_field = mix(
                    ov.r_gon_parameter_field, iteration_r_gon_parameter, interpolation_factor);
                ov.max_unit_parameter_field = mix(ov.max_unit_parameter_field,
                                                  iteration_max_unit_parameter,
                                                  interpolation_factor);
                ov.segment_id_field = mix(
                    ov.segment_id_field, iteration_segment_id, interpolation_factor);
                ov.out_index_field = mix(
                    ov.out_index_field, iteration_index, interpolation_factor);
                ov.out_position_field = mix(
                    ov.out_position_field, iteration_position, interpolation_factor);
                ov.out_r_sphere_coordinates = mix(ov.out_r_sphere_coordinates,
                                                  iteration_r_sphere_coordinates,
                                                  interpolation_factor);
              }
              else if ((iteration_l_angle_bisector_4d < ov.out_r_sphere_field) || first_iteration)
              {
                ov.out_r_sphere_field = iteration_l_angle_bisector_4d;
                ov.r_gon_parameter_field = iteration_r_gon_parameter;
                ov.max_unit_parameter_field = iteration_max_unit_parameter;
                ov.segment_id_field = iteration_segment_id;
                ov.out_index_field = iteration_index;
                ov.out_position_field = iteration_position;
                ov.out_r_sphere_coordinates = iteration_r_sphere_coordinates;
              }
              first_iteration = false;
            }
          }
        }
      }
      break;
    }
  }
  return ov;
}

ccl_device OutVariables raiko_select_grid_dimensions(DeterministicVariables dv,
                                                     float r_sphere[],
                                                     float r_sphere_min[],
                                                     float r_sphere_max[],
                                                     int r_sphere_index_list[],
                                                     int r_sphere_index_count,
                                                     float translation_rotation[],
                                                     float translation_rotation_min[],
                                                     float translation_rotation_max[],
                                                     int translation_rotation_index_list[],
                                                     int translation_rotation_index_count,
                                                     float scale[],
                                                     float scale_randomness[],
                                                     int scale_index_list[],
                                                     int scale_index_count,
                                                     float remap[],
                                                     float remap_min[],
                                                     float remap_max[],
                                                     int remap_index_list[],
                                                     int remap_index_count)
{
  OutVariables ov;
  switch (dv.grid_dimensions) {
    case 0: {
      ov = raiko_select_mode_0d(dv,
                                r_sphere,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                translation_rotation,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                scale,
                                scale_randomness,
                                scale_index_list,
                                scale_index_count,
                                remap,
                                remap_min,
                                remap_max,
                                remap_index_list,
                                remap_index_count);
      break;
    }
    case 1: {
      if (dv.grid_vector_1.x != 0.0f) {
        dv.grid_vector_1 = make_float4(dv.grid_vector_1.x, 0.0f, 0.0f, 0.0f);

        float4 initial_index = froundf(
            make_float4(dv.coord.x / dv.grid_vector_1.x, 0.0f, 0.0f, 0.0f));
        float4 initial_position = initial_index.x * dv.grid_vector_1;

        ov = raiko_select_mode_1d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 2: {
      float M_det = dv.grid_vector_1.x * dv.grid_vector_2.y -
                    dv.grid_vector_2.x * dv.grid_vector_1.y;
      if (M_det != 0.0f) {
        dv.grid_vector_1 = make_float4(dv.grid_vector_1.x, dv.grid_vector_1.y, 0.0f, 0.0f);
        dv.grid_vector_2 = make_float4(dv.grid_vector_2.x, dv.grid_vector_2.y, 0.0f, 0.0f);

        float2 initial_index_xy = froundf(fc_linear_system_solve_non_singular_2x2(
            make_float2(dv.grid_vector_1.x, dv.grid_vector_1.y),
            make_float2(dv.grid_vector_2.x, dv.grid_vector_2.y),
            make_float2(dv.coord.x, dv.coord.y),
            M_det));
        float4 initial_index = make_float4(initial_index_xy.x, initial_index_xy.y, 0.0f, 0.0f);
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2;

        ov = raiko_select_mode_2d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 3: {
      Fcc_3x3 A_fcc = calculate_Fcc_3x3(
          make_float3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
          make_float3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
          make_float3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z));
      if (A_fcc.M_det != 0.0f) {
        dv.grid_vector_1 = make_float4(
            dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z, 0.0f);
        dv.grid_vector_2 = make_float4(
            dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z, 0.0f);
        dv.grid_vector_3 = make_float4(
            dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z, 0.0f);

        float3 initial_index_xyz = froundf(fc_linear_system_solve_non_singular_3x3(
            make_float3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
            make_float3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
            make_float3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z),
            make_float3(dv.coord.x, dv.coord.y, dv.coord.z),
            A_fcc));
        float4 initial_index = make_float4(
            initial_index_xyz.x, initial_index_xyz.y, initial_index_xyz.z, 0.0f);
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2 +
                                  initial_index.z * dv.grid_vector_3;

        ov = raiko_select_mode_3d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
    case 4: {
      Fcc_4x4 A_fcc = calculate_Fcc_4x4(
          dv.grid_vector_1, dv.grid_vector_2, dv.grid_vector_3, dv.grid_vector_4);
      if (A_fcc.M_det != 0.0f) {

        float4 initial_index = froundf(fc_linear_system_solve_non_singular_4x4(dv.grid_vector_1,
                                                                               dv.grid_vector_2,
                                                                               dv.grid_vector_3,
                                                                               dv.grid_vector_4,
                                                                               dv.coord,
                                                                               A_fcc));
        float4 initial_position = initial_index.x * dv.grid_vector_1 +
                                  initial_index.y * dv.grid_vector_2 +
                                  initial_index.z * dv.grid_vector_3 +
                                  initial_index.w * dv.grid_vector_4;

        ov = raiko_select_mode_4d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count,
                                  initial_index,
                                  initial_position);
      }
      else {
        ov = raiko_select_mode_0d(dv,
                                  r_sphere,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  translation_rotation,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  scale,
                                  scale_randomness,
                                  scale_index_list,
                                  scale_index_count,
                                  remap,
                                  remap_min,
                                  remap_max,
                                  remap_index_list,
                                  remap_index_count);
      }
      break;
    }
  }
  return ov;
}

template<uint node_feature_mask>
ccl_device_noinline int svm_node_tex_raiko(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  StackOffsets so;

  uint in_mode, in_normalize_r_gon_parameter, in_integer_sides, in_elliptical_corners,
      in_transform_fields_noise, in_transform_coordinates_noise, in_uniform_scale_randomness,
      in_grid_dimensions, in_step_count;

  svm_unpack_node_uchar4(node.y,
                         &(in_mode),
                         &(in_normalize_r_gon_parameter),
                         &(in_integer_sides),
                         &(in_elliptical_corners));
  svm_unpack_node_uchar4(node.z,
                         &(in_transform_fields_noise),
                         &(in_transform_coordinates_noise),
                         &(in_uniform_scale_randomness),
                         &(in_grid_dimensions));
  svm_unpack_node_uchar4(node.w, &(in_step_count), &(so.vector), &(so.w), &(so.accuracy));

  uint4 stack_offsets_1 = read_node(kg, &offset);
  svm_unpack_node_uchar4(
      stack_offsets_1.x, &(so.scale), &(so.smoothness), &(so.r_gon_sides), &(so.r_gon_roundness));
  svm_unpack_node_uchar4(stack_offsets_1.y,
                         &(so.r_gon_exponent),
                         &(so.sphere_exponent),
                         &(so.r_gon_sides_randomness),
                         &(so.r_gon_roundness_randomness));
  svm_unpack_node_uchar4(stack_offsets_1.z,
                         &(so.r_gon_exponent_randomness),
                         &(so.sphere_exponent_randomness),
                         &(so.transform_rotation),
                         &(so.transform_scale));
  svm_unpack_node_uchar4(stack_offsets_1.w,
                         &(so.transform_scale_w),
                         &(so.transform_rotation_randomness),
                         &(so.transform_scale_randomness),
                         &(so.transform_scale_w_randomness));
  uint4 stack_offsets_2 = read_node(kg, &offset);
  svm_unpack_node_uchar4(stack_offsets_2.x,
                         &(so.noise_fragmentation),
                         &(so.noise_fields_strength_1),
                         &(so.noise_coordinates_strength_1),
                         &(so.noise_scale_1));
  svm_unpack_node_uchar4(stack_offsets_2.y,
                         &(so.noise_detail_1),
                         &(so.noise_roughness_1),
                         &(so.noise_lacunarity_1),
                         &(so.noise_fields_strength_2));
  svm_unpack_node_uchar4(stack_offsets_2.z,
                         &(so.noise_coordinates_strength_2),
                         &(so.noise_scale_2),
                         &(so.noise_detail_2),
                         &(so.noise_roughness_2));
  svm_unpack_node_uchar4(stack_offsets_2.w,
                         &(so.noise_lacunarity_2),
                         &(so.grid_vector_1),
                         &(so.grid_vector_w_1),
                         &(so.grid_vector_2));
  uint4 stack_offsets_3 = read_node(kg, &offset);
  svm_unpack_node_uchar4(stack_offsets_3.x,
                         &(so.grid_vector_w_2),
                         &(so.grid_vector_3),
                         &(so.grid_vector_w_3),
                         &(so.grid_vector_4));
  svm_unpack_node_uchar4(stack_offsets_3.y,
                         &(so.grid_vector_w_4),
                         &(so.grid_points_translation_randomness),
                         &(so.grid_points_translation_w_randomness),
                         &(so.step_center_1));
  svm_unpack_node_uchar4(stack_offsets_3.z,
                         &(so.step_width_1),
                         &(so.step_value_1),
                         &(so.ellipse_height_1),
                         &(so.ellipse_width_1));
  svm_unpack_node_uchar4(stack_offsets_3.w,
                         &(so.inflection_point_1),
                         &(so.step_center_2),
                         &(so.step_width_2),
                         &(so.step_value_2));
  if (NodeRaikoMode(in_mode) == NODE_RAIKO_ADDITIVE) {
    uint4 stack_offsets_4 = read_node(kg, &offset);
    svm_unpack_node_uchar4(stack_offsets_4.x,
                           &(so.ellipse_height_2),
                           &(so.ellipse_width_2),
                           &(so.inflection_point_2),
                           &(so.step_center_3));
    svm_unpack_node_uchar4(stack_offsets_4.y,
                           &(so.step_width_3),
                           &(so.step_value_3),
                           &(so.ellipse_height_3),
                           &(so.ellipse_width_3));
    svm_unpack_node_uchar4(stack_offsets_4.z,
                           &(so.inflection_point_3),
                           &(so.step_center_4),
                           &(so.step_width_4),
                           &(so.step_value_4));
    svm_unpack_node_uchar4(stack_offsets_4.w,
                           &(so.ellipse_height_4),
                           &(so.ellipse_width_4),
                           &(so.inflection_point_4),
                           &(so.step_center_randomness_1));
    uint4 stack_offsets_5 = read_node(kg, &offset);
    svm_unpack_node_uchar4(stack_offsets_5.x,
                           &(so.step_width_randomness_1),
                           &(so.step_value_randomness_1),
                           &(so.ellipse_height_randomness_1),
                           &(so.ellipse_width_randomness_1));
    svm_unpack_node_uchar4(stack_offsets_5.y,
                           &(so.inflection_point_randomness_1),
                           &(so.step_center_randomness_2),
                           &(so.step_width_randomness_2),
                           &(so.step_value_randomness_2));
    svm_unpack_node_uchar4(stack_offsets_5.z,
                           &(so.ellipse_height_randomness_2),
                           &(so.ellipse_width_randomness_2),
                           &(so.inflection_point_randomness_2),
                           &(so.step_center_randomness_3));
    svm_unpack_node_uchar4(stack_offsets_5.w,
                           &(so.step_width_randomness_3),
                           &(so.step_value_randomness_3),
                           &(so.ellipse_height_randomness_3),
                           &(so.ellipse_width_randomness_3));
  }
  else {
    offset += 2;
  }
  uint4 stack_offsets_6 = read_node(kg, &offset);
  svm_unpack_node_uchar4(stack_offsets_6.x,
                         &(so.inflection_point_randomness_3),
                         &(so.step_center_randomness_4),
                         &(so.step_width_randomness_4),
                         &(so.step_value_randomness_4));
  svm_unpack_node_uchar4(stack_offsets_6.y,
                         &(so.ellipse_height_randomness_4),
                         &(so.ellipse_width_randomness_4),
                         &(so.inflection_point_randomness_4),
                         &(so.r_sphere_field));
  svm_unpack_node_uchar4(stack_offsets_6.z,
                         &(so.r_gon_parameter_field),
                         &(so.max_unit_parameter_field),
                         &(so.segment_id_field),
                         &(so.index_field));
  svm_unpack_node_uchar4(stack_offsets_6.w,
                         &(so.index_field_w),
                         &(so.position_field),
                         &(so.position_field_w),
                         &(so.r_sphere_coordinates));
  uint4 defaults_1 = read_node(kg, &offset);
  so.r_sphere_coordinates_w = defaults_1.x;
  uint4 defaults_2 = read_node(kg, &offset);
  uint4 defaults_3 = read_node(kg, &offset);
  uint4 defaults_4 = read_node(kg, &offset);
  uint4 defaults_5 = read_node(kg, &offset);
  uint4 defaults_6 = read_node(kg, &offset);
  uint4 defaults_7 = read_node(kg, &offset);
  uint4 defaults_8 = read_node(kg, &offset);
  uint4 defaults_9 = read_node(kg, &offset);
  uint4 defaults_10 = read_node(kg, &offset);
  uint4 defaults_11 = read_node(kg, &offset);
  uint4 defaults_12 = read_node(kg, &offset);
  uint4 defaults_13 = read_node(kg, &offset);
  uint4 defaults_14 = read_node(kg, &offset);
  uint4 defaults_15 = read_node(kg, &offset);
  uint4 defaults_16;
  uint4 defaults_17;
  uint4 defaults_18;
  uint4 defaults_19;
  uint4 defaults_20;
  uint4 defaults_21;
  uint4 defaults_22;
  uint4 defaults_23;
  uint4 defaults_24;
  uint4 defaults_25;
  uint4 defaults_26;
  uint4 defaults_27;
  if (NodeRaikoMode(in_mode) == NODE_RAIKO_ADDITIVE) {
    defaults_16 = read_node(kg, &offset);
    defaults_17 = read_node(kg, &offset);
    defaults_18 = read_node(kg, &offset);
    defaults_19 = read_node(kg, &offset);
    defaults_20 = read_node(kg, &offset);
    defaults_21 = read_node(kg, &offset);
    defaults_22 = read_node(kg, &offset);
    defaults_23 = read_node(kg, &offset);
    defaults_24 = read_node(kg, &offset);
    defaults_25 = read_node(kg, &offset);
    defaults_26 = read_node(kg, &offset);
    defaults_27 = read_node(kg, &offset);
  }
  else {
    offset += 12;
  }
  uint4 appendix = read_node(kg, &offset);

  DeterministicVariables dv;

  dv.calculate_r_sphere_field = stack_valid(so.r_sphere_field) ||
                                stack_valid(so.r_gon_parameter_field) ||
                                stack_valid(so.max_unit_parameter_field) ||
                                (NodeRaikoMode(in_mode) != NODE_RAIKO_CLOSEST);
  dv.calculate_r_gon_parameter_field = stack_valid(so.r_gon_parameter_field);
  dv.calculate_max_unit_parameter_field = stack_valid(so.max_unit_parameter_field);
  dv.calculate_coordinates_outputs = (stack_valid(so.r_sphere_coordinates) ||
                                      stack_valid(so.r_sphere_coordinates_w)) &&
                                     (NodeRaikoMode(in_mode) != NODE_RAIKO_ADDITIVE);

  dv.mode = NodeRaikoMode(in_mode);
  dv.normalize_r_gon_parameter = stack_valid(so.r_gon_parameter_field) &&
                                 bool(in_normalize_r_gon_parameter);
  dv.integer_sides = bool(in_integer_sides);
  dv.elliptical_corners = bool(in_elliptical_corners);
  dv.invert_order_of_transformation = bool(appendix.x);
  dv.transform_fields_noise = bool(in_transform_fields_noise);
  dv.transform_coordinates_noise = bool(in_transform_coordinates_noise);
  dv.uniform_scale_randomness = bool(in_uniform_scale_randomness);
  dv.grid_dimensions = int(in_grid_dimensions);
  dv.step_count = int(in_step_count);

  dv.accuracy = stack_load_float_default(stack, so.accuracy, defaults_1.z);
  dv.scale = stack_load_float_default(stack, so.scale, defaults_1.w);
  float3 in_vector = stack_load_float3(stack, so.vector);
  float in_w = stack_load_float_default(stack, so.w, defaults_1.y);
  dv.coord = dv.scale * make_float4(in_vector.x, in_vector.y, in_vector.z, in_w);
  dv.smoothness = stack_load_float_default(stack, so.smoothness, defaults_2.x);
  dv.smoothness_non_zero = dv.smoothness != 0.0f;
  /*r_sphere[0] == r_gon_sides; r_sphere[1] == r_gon_roundness; r_sphere[2] == r_gon_exponent;
   * r_sphere[3] == sphere_exponent;*/
  float r_sphere[4] = {stack_load_float_default(stack, so.r_gon_sides, defaults_2.y),
                       stack_load_float_default(stack, so.r_gon_roundness, defaults_2.z),
                       stack_load_float_default(stack, so.r_gon_exponent, defaults_2.w),
                       stack_load_float_default(stack, so.sphere_exponent, defaults_3.x)};
  float4 in_r_sphere_randomness = make_float4(
      stack_load_float_default(stack, so.r_gon_sides_randomness, defaults_3.y),
      stack_load_float_default(stack, so.r_gon_roundness_randomness, defaults_3.z),
      stack_load_float_default(stack, so.r_gon_exponent_randomness, defaults_3.w),
      stack_load_float_default(stack, so.sphere_exponent_randomness, defaults_4.x));
  float r_sphere_min[4];
  float r_sphere_max[4];
  int r_sphere_index_list[4];
  int r_sphere_index_count;
  int index_count = 0;
  if (in_r_sphere_randomness.x != 0.0f) {
    r_sphere_min[index_count] = float_max(r_sphere[0] - in_r_sphere_randomness.x, 2.0f);
    r_sphere_max[index_count] = float_max(r_sphere[0] + in_r_sphere_randomness.x, 2.0f);
    r_sphere_index_list[index_count] = 0;
    ++index_count;
  }
  if (in_r_sphere_randomness.y != 0.0f) {
    r_sphere_min[index_count] = clamp(r_sphere[1] - in_r_sphere_randomness.y, 0.0f, 1.0f);
    r_sphere_max[index_count] = clamp(r_sphere[1] + in_r_sphere_randomness.y, 0.0f, 1.0f);
    r_sphere_index_list[index_count] = 1;
    ++index_count;
  }
  if (in_r_sphere_randomness.z != 0.0f) {
    r_sphere_min[index_count] = float_max(r_sphere[2] - in_r_sphere_randomness.z, 0.0f);
    r_sphere_max[index_count] = float_max(r_sphere[2] + in_r_sphere_randomness.z, 0.0f);
    r_sphere_index_list[index_count] = 2;
    ++index_count;
  }
  if (in_r_sphere_randomness.w != 0.0f) {
    r_sphere_min[index_count] = float_max(r_sphere[3] - in_r_sphere_randomness.w, 0.0f);
    r_sphere_max[index_count] = float_max(r_sphere[3] + in_r_sphere_randomness.w, 0.0f);
    r_sphere_index_list[index_count] = 3;
    ++index_count;
  }
  r_sphere_index_count = index_count;
  float3 in_transform_rotation = stack_load_float3_default(
      stack, so.transform_rotation, make_uint3(defaults_4.y, defaults_4.z, defaults_4.w));
  float translation_rotation[7] = {0.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   in_transform_rotation.x,
                                   in_transform_rotation.y,
                                   in_transform_rotation.z};
  float3 in_grid_points_translation_randomness = stack_load_float3_default(
      stack,
      so.grid_points_translation_randomness,
      make_uint3(defaults_15.x, defaults_15.y, defaults_15.z));
  float in_grid_points_translation_w_randomness = stack_load_float_default(
      stack, so.grid_points_translation_w_randomness, defaults_15.w);
  float3 in_transform_rotation_randomness = stack_load_float3_default(
      stack,
      so.transform_rotation_randomness,
      make_uint3(defaults_6.x, defaults_6.y, defaults_6.z));
  float translation_rotation_min[7];
  float translation_rotation_max[7];
  int translation_rotation_index_list[7];
  int translation_rotation_index_count;
  index_count = 0;
  if (in_grid_points_translation_randomness.x != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[0] -
                                            in_grid_points_translation_randomness.x;
    translation_rotation_max[index_count] = translation_rotation[0] +
                                            in_grid_points_translation_randomness.x;
    translation_rotation_index_list[index_count] = 0;
    ++index_count;
  }
  if (in_grid_points_translation_randomness.y != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[1] -
                                            in_grid_points_translation_randomness.y;
    translation_rotation_max[index_count] = translation_rotation[1] +
                                            in_grid_points_translation_randomness.y;
    translation_rotation_index_list[index_count] = 1;
    ++index_count;
  }
  if (in_grid_points_translation_randomness.z != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[2] -
                                            in_grid_points_translation_randomness.z;
    translation_rotation_max[index_count] = translation_rotation[2] +
                                            in_grid_points_translation_randomness.z;
    translation_rotation_index_list[index_count] = 2;
    ++index_count;
  }
  if (in_grid_points_translation_w_randomness != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[3] -
                                            in_grid_points_translation_w_randomness;
    translation_rotation_max[index_count] = translation_rotation[3] +
                                            in_grid_points_translation_w_randomness;
    translation_rotation_index_list[index_count] = 3;
    ++index_count;
  }
  if (in_transform_rotation_randomness.x != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[4] -
                                            in_transform_rotation_randomness.x;
    translation_rotation_max[index_count] = translation_rotation[4] +
                                            in_transform_rotation_randomness.x;
    translation_rotation_index_list[index_count] = 4;
    ++index_count;
  }
  if (in_transform_rotation_randomness.y != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[5] -
                                            in_transform_rotation_randomness.y;
    translation_rotation_max[index_count] = translation_rotation[5] +
                                            in_transform_rotation_randomness.y;
    translation_rotation_index_list[index_count] = 5;
    ++index_count;
  }
  if (in_transform_rotation_randomness.z != 0.0f) {
    translation_rotation_min[index_count] = translation_rotation[6] -
                                            in_transform_rotation_randomness.z;
    translation_rotation_max[index_count] = translation_rotation[6] +
                                            in_transform_rotation_randomness.z;
    translation_rotation_index_list[index_count] = 6;
    ++index_count;
  }
  translation_rotation_index_count = index_count;
  float3 in_transform_scale = stack_load_float3_default(
      stack, so.transform_scale, make_uint3(defaults_5.x, defaults_5.y, defaults_5.z));
  float in_transform_scale_w = stack_load_float_default(stack, so.transform_scale_w, defaults_5.w);
  float scale[4] = {
      in_transform_scale.x, in_transform_scale.y, in_transform_scale.z, in_transform_scale_w};
  float3 in_transform_scale_randomness = stack_load_float3_default(
      stack, so.transform_scale_randomness, make_uint3(defaults_6.w, defaults_7.x, defaults_7.y));
  float in_transform_scale_w_randomness = stack_load_float_default(
      stack, so.transform_scale_w_randomness, defaults_7.z);
  float scale_randomness[4];
  int scale_index_list[4];
  int scale_index_count;
  index_count = 0;
  if (!dv.uniform_scale_randomness) {
    if (in_transform_scale_randomness.x != 0.0f) {
      scale_randomness[index_count] = in_transform_scale_randomness.x;
      scale_index_list[index_count] = 0;
      ++index_count;
    }
    if (in_transform_scale_randomness.y != 0.0f) {
      scale_randomness[index_count] = in_transform_scale_randomness.y;
      scale_index_list[index_count] = 1;
      ++index_count;
    }
    if (in_transform_scale_randomness.z != 0.0f) {
      scale_randomness[index_count] = in_transform_scale_randomness.z;
      scale_index_list[index_count] = 2;
      ++index_count;
    }
  }
  if (in_transform_scale_w_randomness != 0.0f) {
    scale_randomness[index_count] = in_transform_scale_w_randomness;
    scale_index_list[index_count] = 3;
    ++index_count;
  }
  scale_index_count = index_count;
  dv.noise_fragmentation = stack_load_float_default(stack, so.noise_fragmentation, defaults_7.w);
  dv.noise_fields_strength_1 = stack_load_float_default(
      stack, so.noise_fields_strength_1, defaults_8.x);
  dv.noise_coordinates_strength_1 = stack_load_float_default(
                                        stack, so.noise_coordinates_strength_1, defaults_8.y) *
                                    float(dv.calculate_coordinates_outputs);
  dv.noise_scale_1 = stack_load_float_default(stack, so.noise_scale_1, defaults_8.z);
  dv.noise_detail_1 = stack_load_float_default(stack, so.noise_detail_1, defaults_8.w);
  dv.noise_roughness_1 = stack_load_float_default(stack, so.noise_roughness_1, defaults_9.x);
  dv.noise_lacunarity_1 = stack_load_float_default(stack, so.noise_lacunarity_1, defaults_9.y);
  dv.noise_fields_strength_2 = stack_load_float_default(
      stack, so.noise_fields_strength_2, defaults_9.z);
  dv.noise_coordinates_strength_2 = stack_load_float_default(
                                        stack, so.noise_coordinates_strength_2, defaults_9.w) *
                                    float(dv.calculate_coordinates_outputs);
  dv.noise_scale_2 = stack_load_float_default(stack, so.noise_scale_2, defaults_10.x);
  dv.noise_detail_2 = stack_load_float_default(stack, so.noise_detail_2, defaults_10.y);
  dv.noise_roughness_2 = stack_load_float_default(stack, so.noise_roughness_2, defaults_10.z);
  dv.noise_lacunarity_2 = stack_load_float_default(stack, so.noise_lacunarity_2, defaults_10.w);
  dv.noise_fragmentation_non_zero = dv.noise_fragmentation != 0.0f;
  dv.calculate_fields_noise_1 = dv.noise_fields_strength_1 != 0.0f;
  dv.calculate_fields_noise_2 = dv.noise_fields_strength_2 != 0.0f;
  dv.calculate_coordinates_noise_1 = (dv.noise_coordinates_strength_1 != 0.0f) &&
                                     (!(dv.calculate_fields_noise_1 &&
                                        (dv.transform_fields_noise ==
                                         dv.transform_coordinates_noise)));
  dv.calculate_coordinates_noise_2 = (dv.noise_coordinates_strength_2 != 0.0f) &&
                                     (!(dv.calculate_fields_noise_2 &&
                                        (dv.transform_fields_noise ==
                                         dv.transform_coordinates_noise)));
  if (dv.grid_dimensions == 1) {
    dv.grid_vector_1.x = stack_load_float_default(stack, so.grid_vector_w_1, defaults_11.w);
  }
  else {
    float3 in_grid_vector_1 = stack_load_float3_default(
        stack, so.grid_vector_1, make_uint3(defaults_11.x, defaults_11.y, defaults_11.z));
    float in_grid_vector_w_1 = stack_load_float_default(stack, so.grid_vector_w_1, defaults_11.w);
    float3 in_grid_vector_2 = stack_load_float3_default(
        stack, so.grid_vector_2, make_uint3(defaults_12.x, defaults_12.y, defaults_12.z));
    float in_grid_vector_w_2 = stack_load_float_default(stack, so.grid_vector_w_2, defaults_12.w);
    float3 in_grid_vector_3 = stack_load_float3_default(
        stack, so.grid_vector_3, make_uint3(defaults_13.x, defaults_13.y, defaults_13.z));
    float in_grid_vector_w_3 = stack_load_float_default(stack, so.grid_vector_w_3, defaults_13.w);
    float3 in_grid_vector_4 = stack_load_float3_default(
        stack, so.grid_vector_4, make_uint3(defaults_14.x, defaults_14.y, defaults_14.z));
    float in_grid_vector_w_4 = stack_load_float_default(stack, so.grid_vector_w_4, defaults_14.w);
    dv.grid_vector_1 = make_float4(
        in_grid_vector_1.x, in_grid_vector_1.y, in_grid_vector_1.z, in_grid_vector_w_1);
    dv.grid_vector_2 = make_float4(
        in_grid_vector_2.x, in_grid_vector_2.y, in_grid_vector_2.z, in_grid_vector_w_2);
    dv.grid_vector_3 = make_float4(
        in_grid_vector_3.x, in_grid_vector_3.y, in_grid_vector_3.z, in_grid_vector_w_3);
    dv.grid_vector_4 = make_float4(
        in_grid_vector_4.x, in_grid_vector_4.y, in_grid_vector_4.z, in_grid_vector_w_4);
  }
  float remap[24];
  float remap_randomness[24];
  float remap_min[24];
  float remap_max[24];
  int remap_index_list[24];
  int remap_index_count;
  index_count = 0;
  if (dv.mode == NODE_RAIKO_ADDITIVE) {
    switch (dv.step_count) {
      case 4: {
        ASSIGN_REMAP_INPUTS_4
        ATTR_FALLTHROUGH;
      }
      case 3: {
        ASSIGN_REMAP_INPUTS_3
        ATTR_FALLTHROUGH;
      }
      case 2: {
        ASSIGN_REMAP_INPUTS_2
        ATTR_FALLTHROUGH;
      }
      case 1: {
        ASSIGN_REMAP_INPUTS_1
        break;
      }
    }
  }
  remap_index_count = index_count;

  OutVariables ov = raiko_select_grid_dimensions(dv,
                                                 r_sphere,
                                                 r_sphere_min,
                                                 r_sphere_max,
                                                 r_sphere_index_list,
                                                 r_sphere_index_count,
                                                 translation_rotation,
                                                 translation_rotation_min,
                                                 translation_rotation_max,
                                                 translation_rotation_index_list,
                                                 translation_rotation_index_count,
                                                 scale,
                                                 scale_randomness,
                                                 scale_index_list,
                                                 scale_index_count,
                                                 remap,
                                                 remap_min,
                                                 remap_max,
                                                 remap_index_list,
                                                 remap_index_count);

  if (stack_valid(so.r_sphere_field)) {
    stack_store_float(stack, so.r_sphere_field, ov.out_r_sphere_field);
  }
  if (stack_valid(so.r_gon_parameter_field)) {
    stack_store_float(stack, so.r_gon_parameter_field, ov.r_gon_parameter_field);
  }
  if (stack_valid(so.max_unit_parameter_field)) {
    stack_store_float(stack, so.max_unit_parameter_field, ov.max_unit_parameter_field);
  }
  if (stack_valid(so.segment_id_field)) {
    stack_store_float(stack, so.segment_id_field, ov.segment_id_field);
  }
  if (dv.grid_dimensions == 1) {
    if (stack_valid(so.index_field_w)) {
      stack_store_float(stack, so.index_field_w, ov.out_index_field.x);
    }
  }
  else {
    if (stack_valid(so.index_field)) {
      stack_store_float3(
          stack,
          so.index_field,
          make_float3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z));
    }
    if (stack_valid(so.index_field_w)) {
      stack_store_float(stack, so.index_field_w, ov.out_index_field.w);
    }
  }
  if (stack_valid(so.position_field)) {
    stack_store_float3(
        stack,
        so.position_field,
        make_float3(ov.out_position_field.x, ov.out_position_field.y, ov.out_position_field.z));
  }
  if (stack_valid(so.position_field_w)) {
    stack_store_float(stack, so.position_field_w, ov.out_position_field.w);
  }
  if (stack_valid(so.r_sphere_coordinates)) {
    stack_store_float3(stack,
                       so.r_sphere_coordinates,
                       make_float3(ov.out_r_sphere_coordinates.x,
                                   ov.out_r_sphere_coordinates.y,
                                   ov.out_r_sphere_coordinates.z));
  }
  if (stack_valid(so.r_sphere_coordinates_w)) {
    stack_store_float(stack, so.r_sphere_coordinates_w, ov.out_r_sphere_coordinates.w);
  }

  return offset;
}

CCL_NAMESPACE_END
