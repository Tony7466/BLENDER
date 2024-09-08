/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma BLENDER_REQUIRE(gpu_shader_common_hash.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_material_noise.glsl)

#define do_not_parse_in_lib void

#define ATTR_FALLTHROUGH

#define SHD_RAIKO_ADDITIVE 0
#define SHD_RAIKO_CLOSEST 1
#define SHD_RAIKO_SMOOTH_MINIMUM 2

#define ASSIGN_REMAP_INPUTS(X, A, B, C, D, E, F) \
  remap[A] = in_step_center.X; \
  remap[B] = in_step_width.X; \
  remap[C] = in_step_value.X; \
  remap[D] = in_ellipse_height.X; \
  remap[E] = in_ellipse_width.X; \
  remap[F] = in_inflection_point.X; \
  if (in_step_center_randomness.X != 0.0) { \
    remap_min[index_count] = max(remap[A] - in_step_center_randomness.X, 0.0); \
    remap_max[index_count] = max(remap[A] + in_step_center_randomness.X, 0.0); \
    remap_index_list[index_count] = A; \
    ++index_count; \
  } \
  if (in_step_width_randomness.X != 0.0) { \
    remap_min[index_count] = max(remap[B] - in_step_width_randomness.X, 0.0); \
    remap_max[index_count] = max(remap[B] + in_step_width_randomness.X, 0.0); \
    remap_index_list[index_count] = B; \
    ++index_count; \
  } \
  if (in_step_value_randomness.X != 0.0) { \
    remap_min[index_count] = remap[C] - in_step_value_randomness.X; \
    remap_max[index_count] = remap[C] + in_step_value_randomness.X; \
    remap_index_list[index_count] = C; \
    ++index_count; \
  } \
  if (in_ellipse_height_randomness.X != 0.0) { \
    remap_min[index_count] = clamp(remap[D] - in_ellipse_height_randomness.X, 0.0, 1.0); \
    remap_max[index_count] = clamp(remap[D] + in_ellipse_height_randomness.X, 0.0, 1.0); \
    remap_index_list[index_count] = D; \
    ++index_count; \
  } \
  if (in_ellipse_width_randomness.X != 0.0) { \
    remap_min[index_count] = clamp(remap[E] - in_ellipse_width_randomness.X, 0.0, 1.0); \
    remap_max[index_count] = clamp(remap[E] + in_ellipse_width_randomness.X, 0.0, 1.0); \
    remap_index_list[index_count] = E; \
    ++index_count; \
  } \
  if (in_inflection_point_randomness.X != 0.0) { \
    remap_min[index_count] = clamp(remap[F] - in_inflection_point_randomness.X, 0.0, 1.0); \
    remap_max[index_count] = clamp(remap[F] + in_inflection_point_randomness.X, 0.0, 1.0); \
    remap_index_list[index_count] = F; \
    ++index_count; \
  }

struct DeterministicVariables {
  int mode;
  bool normalize_r_gon_parameter;
  vec4 coord;
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
  vec4 grid_vector_1;
  vec4 grid_vector_2;
  vec4 grid_vector_3;
  vec4 grid_vector_4;
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
  vec4 out_index_field;
  vec4 out_position_field;
  vec4 out_r_sphere_coordinates;
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

Fcc_3x3 calculate_Fcc_3x3(vec3 a_1, vec3 a_2, vec3 a_3)
{
  Fcc_3x3 A_fcc;

  A_fcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_fcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_fcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_det - a_1.y * A_fcc.M_2_1_det + a_1.z * A_fcc.M_3_1_det;

  return A_fcc;
}

Fcc_4x4 calculate_Fcc_4x4(vec4 a_1, vec4 a_2, vec4 a_3, vec4 a_4)
{
  Fcc_4x4 A_fcc;

  A_fcc.M_1_1_fcc = calculate_Fcc_3x3(
      vec3(a_2.y, a_2.z, a_2.w), vec3(a_3.y, a_3.z, a_3.w), vec3(a_4.y, a_4.z, a_4.w));
  A_fcc.M_2_1_fcc = calculate_Fcc_3x3(
      vec3(a_2.x, a_2.z, a_2.w), vec3(a_3.x, a_3.z, a_3.w), vec3(a_4.x, a_4.z, a_4.w));
  A_fcc.M_3_1_fcc = calculate_Fcc_3x3(
      vec3(a_2.x, a_2.y, a_2.w), vec3(a_3.x, a_3.y, a_3.w), vec3(a_4.x, a_4.y, a_4.w));
  A_fcc.M_4_1_fcc = calculate_Fcc_3x3(
      vec3(a_2.x, a_2.y, a_2.z), vec3(a_3.x, a_3.y, a_3.z), vec3(a_4.x, a_4.y, a_4.z));

  A_fcc.M_det = a_1.x * A_fcc.M_1_1_fcc.M_det - a_1.y * A_fcc.M_2_1_fcc.M_det +
                a_1.z * A_fcc.M_3_1_fcc.M_det - a_1.w * A_fcc.M_4_1_fcc.M_det;

  return A_fcc;
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 2x2 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec2 fc_linear_system_solve_non_singular_2x2(vec2 a_1, vec2 a_2, vec2 b, float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is faster. */
  return vec2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 3x3 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec3 fc_linear_system_solve_non_singular_3x3(vec3 a_1, vec3 a_2, vec3 a_3, vec3 b, Fcc_3x3 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_det - b.y * A_fcc.M_2_1_det + b.z * A_fcc.M_3_1_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_det != 0.0) {
    vec2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        vec2(a_2.y, a_2.z),
        vec2(a_3.y, a_3.z),
        vec2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_1_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_fcc.M_2_1_det != 0.0) {
    vec2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        vec2(a_2.x, a_2.z),
        vec2(a_3.x, a_3.z),
        vec2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_2_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    vec2 solution_yz = fc_linear_system_solve_non_singular_2x2(
        vec2(a_2.x, a_2.y),
        vec2(a_3.x, a_3.y),
        vec2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_fcc.M_3_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Fast Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 4x4 matrix A. fc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec4 fc_linear_system_solve_non_singular_4x4(
    vec4 a_1, vec4 a_2, vec4 a_3, vec4 a_4, vec4 b, Fcc_4x4 A_fcc)
{
  float solution_x = (b.x * A_fcc.M_1_1_fcc.M_det - b.y * A_fcc.M_2_1_fcc.M_det +
                      b.z * A_fcc.M_3_1_fcc.M_det - b.w * A_fcc.M_4_1_fcc.M_det) /
                     A_fcc.M_det;

  if (A_fcc.M_1_1_fcc.M_det != 0.0) {
    vec3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        vec3(a_2.y, a_2.z, a_2.w),
        vec3(a_3.y, a_3.z, a_3.w),
        vec3(a_4.y, a_4.z, a_4.w),
        vec3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_1_1_fcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_2_1_fcc.M_det != 0.0) {
    vec3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.z, a_2.w),
        vec3(a_3.x, a_3.z, a_3.w),
        vec3(a_4.x, a_4.z, a_4.w),
        vec3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_2_1_fcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_fcc.M_3_1_fcc.M_det != 0.0) {
    vec3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.y, a_2.w),
        vec3(a_3.x, a_3.y, a_3.w),
        vec3(a_4.x, a_4.y, a_4.w),
        vec3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_fcc.M_3_1_fcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    vec3 solution_yzw = fc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.y, a_2.z),
        vec3(a_3.x, a_3.y, a_3.z),
        vec3(a_4.x, a_4.y, a_4.z),
        vec3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_fcc.M_4_1_fcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

float chebychev_norm(float coord)
{
  return abs(coord);
}

float chebychev_norm(vec2 coord)
{
  return max(abs(coord.x), abs(coord.y));
}

float chebychev_norm(vec3 coord)
{
  return max(abs(coord.x), max(abs(coord.y), abs(coord.z)));
}

float p_norm(float coord)
{
  return abs(coord);
}

float p_norm(vec2 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0) ?
             chebychev_norm(coord) :
             pow(pow(abs(coord.x), exponent) + pow(abs(coord.y), exponent), 1.0 / exponent);
}

float p_norm(vec3 coord, float exponent)
{
  /* Use Chebychev norm instead of p-norm for high exponent values to avoid going out of the
   * floating point representable range. */
  return (exponent > 32.0) ? chebychev_norm(coord) :
                             pow(pow(abs(coord.x), exponent) + pow(abs(coord.y), exponent) +
                                     pow(abs(coord.z), exponent),
                                 1.0 / exponent);
}

float euclidean_norm(vec4 coord)
{
  return sqrt(square(coord.x) + square(coord.y) + square(coord.z) + square(coord.w));
}

float calculate_l_angle_bisector_2d_full_roundness_irregular_elliptical(float r_gon_sides,
                                                                        vec2 coord,
                                                                        float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    return l_projection_2d;
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_basis_vector_1 = tan(ref_A_angle_bisector);
    /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
     * precision issues. Change to double if necessary */
    float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                             sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
    vec2 ellipse_center =
        vec2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
             sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
        l_basis_vector_2 *
            vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
    vec2 transformed_direction_vector = vec2(
        cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    vec2 transformed_origin = vec2(
        (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
         ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
            (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
        -(ellipse_center.y * sin(ref_A_angle_bisector) +
          ellipse_center.x * cos(ref_A_angle_bisector)) /
            (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
    float l_coord_R_l_angle_bisector_2d =
        (-(transformed_direction_vector.x * transformed_origin.x +
           transformed_direction_vector.y * transformed_origin.y) +
         sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                     transformed_direction_vector.y * transformed_origin.y) -
              (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                  (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
        (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
    return l_projection_2d / l_coord_R_l_angle_bisector_2d;
  }
}

float calculate_l_angle_bisector_2d_irregular_elliptical(float r_gon_sides,
                                                         float r_gon_roundness,
                                                         vec2 coord,
                                                         float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                      sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                           square(l_circle_radius) - square(l_circle_center));
      return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
      float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                               sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
      vec2 ellipse_center =
          vec2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
               sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
          l_basis_vector_2 *
              vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      vec2 transformed_direction_vector = vec2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      vec2 transformed_origin = vec2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                       transformed_direction_vector.y * transformed_origin.y) -
                (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                    (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      return l_projection_2d / l_coord_R_l_angle_bisector_2d;
    }
  }
}

float calculate_l_angle_bisector_2d_full_roundness_irregular_circular(float r_gon_sides,
                                                                      vec2 coord,
                                                                      float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(cos(last_angle_bisector_A_x_axis) -
                                     l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                 l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                     sin(last_angle_bisector_A_x_axis));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      return l_projection_2d;
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_coord_R_l_last_angle_bisector_2d =
        sin(nearest_ref_MSA_coord) * last_circle_center.y +
        cos(nearest_ref_MSA_coord) * last_circle_center.x +
        sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                    cos(nearest_ref_MSA_coord) * last_circle_center.x) +
             square(l_last_circle_radius) - square(last_circle_center.x) -
             square(last_circle_center.y));
    return (cos(ref_A_angle_bisector) * l_projection_2d) /
           (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
  }
}

float calculate_l_angle_bisector_2d_irregular_circular(float r_gon_sides,
                                                       float r_gon_roundness,
                                                       vec2 coord,
                                                       float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
      float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                              sin(ref_A_angle_bisector);
      float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                      sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                           square(l_circle_radius) - square(l_circle_center));
      return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
             l_coord_R_l_bevel_start;
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                             cos(last_angle_bisector_A_x_axis);
      return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
             l_angle_bisector_2d_R_l_last_angle_bisector_2d;
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                      cos(nearest_ref_MSA_coord) * last_circle_center.x) +
               square(l_last_circle_radius) - square(last_circle_center.x) -
               square(last_circle_center.y));
      return (cos(ref_A_angle_bisector) * l_projection_2d) /
             (cos(last_angle_bisector_A_x_axis) * l_coord_R_l_last_angle_bisector_2d);
    }
  }
}

float calculate_l_angle_bisector_2d(bool integer_sides,
                                    bool elliptical_corners,
                                    float r_gon_sides,
                                    float r_gon_roundness,
                                    float r_gon_exponent,
                                    vec2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fract(r_gon_sides) == 0.0)) {
    if (r_gon_roundness == 0.0) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
      float ref_A_angle_bisector = M_PI / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
    }
    if (r_gon_roundness == 1.0) {
      return l_projection_2d;
    }
    else {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
      float ref_A_angle_bisector = M_PI / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                        sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                             square(l_circle_radius) - square(l_circle_center));
        return cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
               l_coord_R_l_bevel_start;
      }
      else {
        return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
      float ref_A_angle_bisector = M_PI / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU - last_ref_A_x_axis) {
        return l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
      }
      else {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        return l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
      }
    }
    if (r_gon_roundness == 1.0) {
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

float calculate_l_angle_bisector_4d(bool integer_sides,
                                    bool elliptical_corners,
                                    float r_gon_sides,
                                    float r_gon_roundness,
                                    float r_gon_exponent,
                                    float sphere_exponent,
                                    vec4 coord)
{
  return p_norm(vec3(calculate_l_angle_bisector_2d(integer_sides,
                                                   elliptical_corners,
                                                   integer_sides ? ceil(r_gon_sides) : r_gon_sides,
                                                   r_gon_roundness,
                                                   r_gon_exponent,
                                                   vec2(coord.x, coord.y)),
                     coord.z,
                     coord.w),
                sphere_exponent);
}

vec4 calculate_out_fields_2d_full_roundness_irregular_elliptical(
    bool calculate_r_sphere_field,
    bool calculate_r_gon_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    vec2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

  if ((x_axis_A_coord >= ref_A_angle_bisector) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_angle_bisector))
  {
    float r_gon_parameter_2d = 0.0;
    if (calculate_r_gon_parameter_field) {
      r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
      if (ref_A_coord < ref_A_angle_bisector) {
        r_gon_parameter_2d *= -1.0;
      }
      if (normalize_r_gon_parameter) {
        r_gon_parameter_2d /= ref_A_angle_bisector;
      }
    }
    return vec4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0;
    float r_gon_parameter_2d = 0.0;
    float max_unit_parameter_2d = 0.0;
    if (calculate_r_sphere_field) {
      float l_basis_vector_1 = tan(ref_A_angle_bisector);
      /* When the fractional part of r_gon_sides is very small division by l_basis_vector_2 causes
       * precision issues. Change to double if necessary */
      float l_basis_vector_2 = sin(last_angle_bisector_A_x_axis) *
                               sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
      vec2 ellipse_center =
          vec2(cos(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector),
               sin(ref_A_angle_bisector) / cos(ref_A_angle_bisector - ref_A_angle_bisector)) -
          l_basis_vector_2 *
              vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
      vec2 transformed_direction_vector = vec2(
          cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      vec2 transformed_origin = vec2(
          (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
           ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
              (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
          -(ellipse_center.y * sin(ref_A_angle_bisector) +
            ellipse_center.x * cos(ref_A_angle_bisector)) /
              (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
      float l_coord_R_l_angle_bisector_2d =
          (-(transformed_direction_vector.x * transformed_origin.x +
             transformed_direction_vector.y * transformed_origin.y) +
           sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                       transformed_direction_vector.y * transformed_origin.y) -
                (square(transformed_direction_vector.x) + square(transformed_direction_vector.y)) *
                    (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
          (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
      l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
      if (nearest_ref_MSA_coord < 0.0) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
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
          r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter) {
            r_gon_parameter_2d /= ref_A_angle_bisector;
          }
        }
        max_unit_parameter_2d = ref_A_angle_bisector;
      }
    }
    return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

vec4 calculate_out_fields_2d_irregular_elliptical(bool calculate_r_sphere_field,
                                                  bool calculate_r_gon_parameter_field,
                                                  bool calculate_max_unit_parameter_field,
                                                  bool normalize_r_gon_parameter,
                                                  float r_gon_sides,
                                                  float r_gon_roundness,
                                                  vec2 coord,
                                                  float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));

  if ((x_axis_A_coord >= ref_A_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - ref_A_bevel_start))
  {
    if ((ref_A_coord >= ref_A_bevel_start) && (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                        sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                             square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - abs(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_basis_vector_1 = r_gon_roundness * tan(ref_A_angle_bisector);
        float l_basis_vector_2 = r_gon_roundness * sin(last_angle_bisector_A_x_axis) *
                                 sqrt(square(tan(ref_A_angle_bisector)) + 1.0);
        vec2 ellipse_center =
            vec2(cos(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start),
                 sin(ref_A_bevel_start) / cos(ref_A_angle_bisector - ref_A_bevel_start)) -
            l_basis_vector_2 *
                vec2(sin(last_angle_bisector_A_x_axis), cos(last_angle_bisector_A_x_axis));
        vec2 transformed_direction_vector = vec2(
            cos(last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) /
                (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            cos(ref_A_angle_bisector - nearest_ref_MSA_coord) /
                (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        vec2 transformed_origin = vec2(
            (ellipse_center.y * sin(last_angle_bisector_A_x_axis) -
             ellipse_center.x * cos(last_angle_bisector_A_x_axis)) /
                (l_basis_vector_1 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)),
            -(ellipse_center.y * sin(ref_A_angle_bisector) +
              ellipse_center.x * cos(ref_A_angle_bisector)) /
                (l_basis_vector_2 * sin(ref_A_angle_bisector + last_angle_bisector_A_x_axis)));
        float l_coord_R_l_angle_bisector_2d =
            (-(transformed_direction_vector.x * transformed_origin.x +
               transformed_direction_vector.y * transformed_origin.y) +
             sqrt(square(transformed_direction_vector.x * transformed_origin.x +
                         transformed_direction_vector.y * transformed_origin.y) -
                  (square(transformed_direction_vector.x) +
                   square(transformed_direction_vector.y)) *
                      (square(transformed_origin.x) + square(transformed_origin.y) - 1.0))) /
            (square(transformed_direction_vector.x) + square(transformed_direction_vector.y));
        l_angle_bisector_2d = l_projection_2d / l_coord_R_l_angle_bisector_2d;
        if (nearest_ref_MSA_coord < 0.0) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0;
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
              r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

vec4 calculate_out_fields_2d_full_roundness_irregular_circular(
    bool calculate_r_sphere_field,
    bool calculate_r_gon_parameter_field,
    bool calculate_max_unit_parameter_field,
    bool normalize_r_gon_parameter,
    float r_gon_sides,
    vec2 coord,
    float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float l_last_circle_radius = tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(cos(last_angle_bisector_A_x_axis) -
                                     l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
                                 l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
                                     sin(last_angle_bisector_A_x_axis));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) ||
        (x_axis_A_coord < ref_A_angle_bisector))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      float r_gon_parameter_2d = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return vec4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
  }
  else {
    /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
     * to avoid a case distinction. */
    float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
        (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
    {
      nearest_ref_MSA_coord += last_ref_A_x_axis;
      nearest_ref_MSA_coord *= -1;
    }
    float l_angle_bisector_2d = 0.0;
    float r_gon_parameter_2d = 0.0;
    float max_unit_parameter_2d = 0.0;
    if (calculate_r_sphere_field) {
      float l_coord_R_l_last_angle_bisector_2d =
          sin(nearest_ref_MSA_coord) * last_circle_center.y +
          cos(nearest_ref_MSA_coord) * last_circle_center.x +
          sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                      cos(nearest_ref_MSA_coord) * last_circle_center.x) +
               square(l_last_circle_radius) - square(last_circle_center.x) -
               square(last_circle_center.y));
      l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                            (cos(last_angle_bisector_A_x_axis) *
                             l_coord_R_l_last_angle_bisector_2d);
      if (nearest_ref_MSA_coord < 0.0) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = (last_angle_bisector_A_x_axis + nearest_ref_MSA_coord) *
                               l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
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
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector -
                                                             x_axis_A_outer_last_bevel_start)) +
                               x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
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
    return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
  }
}

vec4 calculate_out_fields_2d_irregular_circular(bool calculate_r_sphere_field,
                                                bool calculate_r_gon_parameter_field,
                                                bool calculate_max_unit_parameter_field,
                                                bool normalize_r_gon_parameter,
                                                float r_gon_sides,
                                                float r_gon_roundness,
                                                vec2 coord,
                                                float l_projection_2d)
{
  float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
  float ref_A_angle_bisector = M_PI / r_gon_sides;
  float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
  float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
  float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;
  float ref_A_bevel_start = ref_A_angle_bisector -
                            atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

  float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
  float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;
  float inner_last_bevel_start_A_x_axis = last_angle_bisector_A_x_axis -
                                          atan((1 - r_gon_roundness) *
                                               tan(last_angle_bisector_A_x_axis));
  float l_last_circle_radius = r_gon_roundness * tan(last_angle_bisector_A_x_axis) /
                               tan(0.5 * (ref_A_angle_bisector + last_angle_bisector_A_x_axis));
  vec2 last_circle_center = vec2(
      (cos(inner_last_bevel_start_A_x_axis) /
       cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)) -
          l_last_circle_radius * cos(last_angle_bisector_A_x_axis),
      l_last_circle_radius * sin(last_angle_bisector_A_x_axis) -
          (sin(inner_last_bevel_start_A_x_axis) /
           cos(last_angle_bisector_A_x_axis - inner_last_bevel_start_A_x_axis)));
  vec2 outer_last_bevel_start = last_circle_center +
                                l_last_circle_radius *
                                    vec2(cos(ref_A_angle_bisector), sin(ref_A_angle_bisector));
  float x_axis_A_outer_last_bevel_start = atan(outer_last_bevel_start.y /
                                               outer_last_bevel_start.x);

  if ((x_axis_A_coord >= x_axis_A_outer_last_bevel_start) &&
      (x_axis_A_coord < M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start))
  {
    if (((ref_A_coord >= ref_A_bevel_start) &&
         (ref_A_coord < ref_A_next_ref - ref_A_bevel_start)) ||
        (x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_bevel_start) ||
        (x_axis_A_coord < ref_A_bevel_start))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - ref_A_angle_bisector) ||
            (x_axis_A_coord < ref_A_angle_bisector))
        {
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(abs(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
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
                                 tan(abs(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
       * angles are negative.*/
      float nearest_ref_SA_coord = ref_A_coord -
                                   float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
        float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                sin(ref_A_angle_bisector);
        float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                        sqrt(square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                             square(l_circle_radius) - square(l_circle_center));
        l_angle_bisector_2d = cos(ref_A_angle_bisector - ref_A_bevel_start) * l_projection_2d /
                              l_coord_R_l_bevel_start;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                                   tan(ref_A_angle_bisector - ref_A_bevel_start) +
                               ref_A_bevel_start - abs(nearest_ref_SA_coord);
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
  else {
    if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis + inner_last_bevel_start_A_x_axis) &&
        (x_axis_A_coord < M_TAU - inner_last_bevel_start_A_x_axis))
    {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                               cos(last_angle_bisector_A_x_axis);
        l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                              l_angle_bisector_2d_R_l_last_angle_bisector_2d;
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d *
                               tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
          if (ref_A_coord < last_angle_bisector_A_x_axis) {
            r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    else {
      /* MSA == Mirrored Signed Angle. The values are mirrored around the last angle bisector
       * to avoid a case distinction. */
      float nearest_ref_MSA_coord = atan2(coord.y, coord.x);
      if ((x_axis_A_coord >= M_TAU - last_ref_A_x_axis - x_axis_A_outer_last_bevel_start) &&
          (x_axis_A_coord < M_TAU - last_angle_bisector_A_x_axis))
      {
        nearest_ref_MSA_coord += last_ref_A_x_axis;
        nearest_ref_MSA_coord *= -1;
      }
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        float l_coord_R_l_last_angle_bisector_2d =
            sin(nearest_ref_MSA_coord) * last_circle_center.y +
            cos(nearest_ref_MSA_coord) * last_circle_center.x +
            sqrt(square(sin(nearest_ref_MSA_coord) * last_circle_center.y +
                        cos(nearest_ref_MSA_coord) * last_circle_center.x) +
                 square(l_last_circle_radius) - square(last_circle_center.x) -
                 square(last_circle_center.y));
        l_angle_bisector_2d = (cos(ref_A_angle_bisector) * l_projection_2d) /
                              (cos(last_angle_bisector_A_x_axis) *
                               l_coord_R_l_last_angle_bisector_2d);
        if (nearest_ref_MSA_coord < 0.0) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(last_angle_bisector_A_x_axis -
                                                               inner_last_bevel_start_A_x_axis)) +
                                 (inner_last_bevel_start_A_x_axis + nearest_ref_MSA_coord) *
                                     l_angle_bisector_2d_R_l_last_angle_bisector_2d;
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0;
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
            r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector -
                                                               x_axis_A_outer_last_bevel_start)) +
                                 x_axis_A_outer_last_bevel_start - nearest_ref_MSA_coord;
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
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
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
  }
}

vec4 calculate_out_fields_2d(bool calculate_r_sphere_field,
                             bool calculate_r_gon_parameter_field,
                             bool calculate_max_unit_parameter_field,
                             bool normalize_r_gon_parameter,
                             bool integer_sides,
                             bool elliptical_corners,
                             float r_gon_sides,
                             float r_gon_roundness,
                             float r_gon_exponent,
                             vec2 coord)
{
  float l_projection_2d = p_norm(coord, r_gon_exponent);

  if (integer_sides || (fract(r_gon_sides) == 0.0)) {
    float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
    float ref_A_angle_bisector = M_PI / r_gon_sides;
    float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
    float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
    float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

    if (r_gon_roundness == 0.0) {
      float l_angle_bisector_2d = 0.0;
      float r_gon_parameter_2d = 0.0;
      float max_unit_parameter_2d = 0.0;
      if (calculate_r_sphere_field) {
        l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
        if (calculate_r_gon_parameter_field) {
          r_gon_parameter_2d = l_angle_bisector_2d * tan(abs(ref_A_angle_bisector - ref_A_coord));
          if (ref_A_coord < ref_A_angle_bisector) {
            r_gon_parameter_2d *= -1.0;
          }
          if (normalize_r_gon_parameter && (r_gon_sides != 2.0)) {
            r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
          }
        }
        if (calculate_max_unit_parameter_field) {
          max_unit_parameter_2d = (r_gon_sides != 2.0) ? tan(ref_A_angle_bisector) : 0.0;
        }
      }
      return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
    }
    if (r_gon_roundness == 1.0) {
      float r_gon_parameter_2d = 0.0;
      if (calculate_r_gon_parameter_field) {
        r_gon_parameter_2d = abs(ref_A_angle_bisector - ref_A_coord);
        if (ref_A_coord < ref_A_angle_bisector) {
          r_gon_parameter_2d *= -1.0;
        }
        if (normalize_r_gon_parameter) {
          r_gon_parameter_2d /= ref_A_angle_bisector;
        }
      }
      return vec4(l_projection_2d, r_gon_parameter_2d, ref_A_angle_bisector, segment_id);
    }
    else {
      float ref_A_bevel_start = ref_A_angle_bisector -
                                atan((1 - r_gon_roundness) * tan(ref_A_angle_bisector));

      if ((ref_A_coord >= ref_A_next_ref - ref_A_bevel_start) || (ref_A_coord < ref_A_bevel_start))
      {
        /* SA == Signed Angle in [-M_PI, M_PI]. Counterclockwise angles are positive, clockwise
         * angles are negative.*/
        float nearest_ref_SA_coord = ref_A_coord -
                                     float(ref_A_coord > ref_A_angle_bisector) * ref_A_next_ref;
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;
        if (calculate_r_sphere_field) {
          float l_circle_radius = sin(ref_A_bevel_start) / sin(ref_A_angle_bisector);
          float l_circle_center = sin(ref_A_angle_bisector - ref_A_bevel_start) /
                                  sin(ref_A_angle_bisector);
          float l_coord_R_l_bevel_start = cos(nearest_ref_SA_coord) * l_circle_center +
                                          sqrt(
                                              square(cos(nearest_ref_SA_coord) * l_circle_center) +
                                              square(l_circle_radius) - square(l_circle_center));
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_bevel_start) /
                                l_coord_R_l_bevel_start;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                     tan(ref_A_angle_bisector - ref_A_bevel_start) +
                                 ref_A_bevel_start - abs(nearest_ref_SA_coord);
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
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
        return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(abs(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
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
        return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
  }
  else {
    if (r_gon_roundness == 0.0) {
      float x_axis_A_coord = atan2(coord.y, coord.x) + float(coord.y < 0.0) * M_TAU;
      float ref_A_angle_bisector = M_PI / r_gon_sides;
      float ref_A_next_ref = 2.0 * ref_A_angle_bisector;
      float segment_id = floor(x_axis_A_coord / ref_A_next_ref);
      float ref_A_coord = x_axis_A_coord - segment_id * ref_A_next_ref;

      float last_angle_bisector_A_x_axis = M_PI - floor(r_gon_sides) * ref_A_angle_bisector;
      float last_ref_A_x_axis = 2.0 * last_angle_bisector_A_x_axis;

      if (x_axis_A_coord < M_TAU - last_ref_A_x_axis) {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;
        if (calculate_r_sphere_field) {
          l_angle_bisector_2d = l_projection_2d * cos(ref_A_angle_bisector - ref_A_coord);
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(abs(ref_A_angle_bisector - ref_A_coord));
            if (ref_A_coord < ref_A_angle_bisector) {
              r_gon_parameter_2d *= -1.0;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(ref_A_angle_bisector);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(ref_A_angle_bisector);
          }
        }
        return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
      else {
        float l_angle_bisector_2d = 0.0;
        float r_gon_parameter_2d = 0.0;
        float max_unit_parameter_2d = 0.0;
        if (calculate_r_sphere_field) {
          float l_angle_bisector_2d_R_l_last_angle_bisector_2d = cos(ref_A_angle_bisector) /
                                                                 cos(last_angle_bisector_A_x_axis);
          l_angle_bisector_2d = l_projection_2d * cos(last_angle_bisector_A_x_axis - ref_A_coord) *
                                l_angle_bisector_2d_R_l_last_angle_bisector_2d;
          if (calculate_r_gon_parameter_field) {
            r_gon_parameter_2d = l_angle_bisector_2d *
                                 tan(abs(last_angle_bisector_A_x_axis - ref_A_coord));
            if (ref_A_coord < last_angle_bisector_A_x_axis) {
              r_gon_parameter_2d *= -1.0;
            }
            if (normalize_r_gon_parameter) {
              r_gon_parameter_2d /= l_angle_bisector_2d * tan(last_angle_bisector_A_x_axis);
            }
          }
          if (calculate_max_unit_parameter_field) {
            max_unit_parameter_2d = tan(last_angle_bisector_A_x_axis);
          }
        }
        return vec4(l_angle_bisector_2d, r_gon_parameter_2d, max_unit_parameter_2d, segment_id);
      }
    }
    if (r_gon_roundness == 1.0) {
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

vec4 calculate_out_fields_4d(bool calculate_r_sphere_field,
                             bool calculate_r_gon_parameter_field,
                             bool calculate_max_unit_parameter_field,
                             bool normalize_r_gon_parameter,
                             bool integer_sides,
                             bool elliptical_corners,
                             float r_gon_sides,
                             float r_gon_roundness,
                             float r_gon_exponent,
                             float sphere_exponent,
                             vec4 coord)
{
  vec4 out_fields = calculate_out_fields_2d(calculate_r_sphere_field,
                                            calculate_r_gon_parameter_field,
                                            calculate_max_unit_parameter_field,
                                            normalize_r_gon_parameter,
                                            integer_sides,
                                            elliptical_corners,
                                            integer_sides ? ceil(r_gon_sides) : r_gon_sides,
                                            r_gon_roundness,
                                            r_gon_exponent,
                                            vec2(coord.x, coord.y));
  out_fields.x = p_norm(vec3(out_fields.x, coord.z, coord.w), sphere_exponent);
  return out_fields;
}

do_not_parse_in_lib randomize_scale(inout float scale_randomized[4],
                                    float scale_randomness[4],
                                    int scale_index_list[4],
                                    int scale_index_count,
                                    bool uniform_scale_randomness,
                                    float seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= pow(
          2.0,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_float_to_float(float(scale_index_list[i]) + seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = pow(
        2.0, mix(-scale_randomness[0], scale_randomness[0], hash_float_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

do_not_parse_in_lib randomize_scale(inout float scale_randomized[4],
                                    float scale_randomness[4],
                                    int scale_index_list[4],
                                    int scale_index_count,
                                    bool uniform_scale_randomness,
                                    vec2 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= pow(
          2.0,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_vec2_to_float(vec2(scale_index_list[i], scale_index_list[i]) + seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = pow(
        2.0, mix(-scale_randomness[0], scale_randomness[0], hash_vec2_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

do_not_parse_in_lib randomize_scale(inout float scale_randomized[4],
                                    float scale_randomness[4],
                                    int scale_index_list[4],
                                    int scale_index_count,
                                    bool uniform_scale_randomness,
                                    vec3 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= pow(
          2.0,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_vec3_to_float(
                  vec3(scale_index_list[i], scale_index_list[i], scale_index_list[i]) +
                  seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = pow(
        2.0, mix(-scale_randomness[0], scale_randomness[0], hash_vec3_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

do_not_parse_in_lib randomize_scale(inout float scale_randomized[4],
                                    float scale_randomness[4],
                                    int scale_index_list[4],
                                    int scale_index_count,
                                    bool uniform_scale_randomness,
                                    vec4 seed_offset)
{
  if (!uniform_scale_randomness) {
    for (int i = 0; i < scale_index_count; ++i) {
      scale_randomized[scale_index_list[i]] *= pow(
          2.0,
          mix(-scale_randomness[i],
              scale_randomness[i],
              hash_vec4_to_float(vec4(scale_index_list[i],
                                      scale_index_list[i],
                                      scale_index_list[i],
                                      scale_index_list[i]) +
                                 seed_offset)));
    }
  }
  else if (scale_index_count != 0) {
    float random_scale_factor = pow(
        2.0, mix(-scale_randomness[0], scale_randomness[0], hash_vec4_to_float(seed_offset)));
    for (int i = 0; i < 4; i++) {
      scale_randomized[i] *= random_scale_factor;
    }
  }
}

#define RANDOMIZE_FLOAT_ARRAY_FLOAT(X) \
  do_not_parse_in_lib randomize_float_array(inout float array[X], \
                                            float min[X], \
                                            float max[X], \
                                            int index_list[X], \
                                            int index_count, \
                                            float seed_offset) \
  { \
    for (int i = 0; i < index_count; ++i) { \
      array[index_list[i]] = mix( \
          min[i], max[i], hash_float_to_float(float(index_list[i]) + seed_offset)); \
    } \
  }

RANDOMIZE_FLOAT_ARRAY_FLOAT(4)
RANDOMIZE_FLOAT_ARRAY_FLOAT(7)
RANDOMIZE_FLOAT_ARRAY_FLOAT(24)

#define RANDOMIZE_FLOAT_ARRAY_VEC2(X) \
  do_not_parse_in_lib randomize_float_array(inout float array[X], \
                                            float min[X], \
                                            float max[X], \
                                            int index_list[X], \
                                            int index_count, \
                                            vec2 seed_offset) \
  { \
    for (int i = 0; i < index_count; ++i) { \
      array[index_list[i]] = mix( \
          min[i], max[i], hash_vec2_to_float(vec2(index_list[i], index_list[i]) + seed_offset)); \
    } \
  }

RANDOMIZE_FLOAT_ARRAY_VEC2(4)
RANDOMIZE_FLOAT_ARRAY_VEC2(7)
RANDOMIZE_FLOAT_ARRAY_VEC2(24)

#define RANDOMIZE_FLOAT_ARRAY_VEC3(X) \
  do_not_parse_in_lib randomize_float_array(inout float array[X], \
                                            float min[X], \
                                            float max[X], \
                                            int index_list[X], \
                                            int index_count, \
                                            vec3 seed_offset) \
  { \
    for (int i = 0; i < index_count; ++i) { \
      array[index_list[i]] = mix( \
          min[i], \
          max[i], \
          hash_vec3_to_float(vec3(index_list[i], index_list[i], index_list[i]) + seed_offset)); \
    } \
  }

RANDOMIZE_FLOAT_ARRAY_VEC3(4)
RANDOMIZE_FLOAT_ARRAY_VEC3(7)
RANDOMIZE_FLOAT_ARRAY_VEC3(24)

#define RANDOMIZE_FLOAT_ARRAY_VEC4(X) \
  do_not_parse_in_lib randomize_float_array(inout float array[X], \
                                            float min[X], \
                                            float max[X], \
                                            int index_list[X], \
                                            int index_count, \
                                            vec4 seed_offset) \
  { \
    for (int i = 0; i < index_count; ++i) { \
      array[index_list[i]] = mix( \
          min[i], \
          max[i], \
          hash_vec4_to_float(vec4(index_list[i], index_list[i], index_list[i], index_list[i]) + \
                             seed_offset)); \
    } \
  }

RANDOMIZE_FLOAT_ARRAY_VEC4(4)
RANDOMIZE_FLOAT_ARRAY_VEC4(7)
RANDOMIZE_FLOAT_ARRAY_VEC4(24)

float elliptical_ramp(float value, float ellipse_height, float ellipse_width)
{
  if (value < 0.0) {
    return 0.0;
  }
  else if (value < ellipse_width + ellipse_height * (1.0 - ellipse_width)) {
    return (ellipse_height *
            (value * ellipse_height * (1.0 - ellipse_width) + square(ellipse_width) -
             ellipse_width * sqrt(square(ellipse_width) - square(value) +
                                  2.0 * value * ellipse_height * (1.0 - ellipse_width)))) /
           (square(ellipse_height * (1.0 - ellipse_width)) + square(ellipse_width));
  }
  else {
    return (ellipse_width == 1.0) ? ellipse_height :
                                    (value - ellipse_width) / (1.0 - ellipse_width);
  }
}

float elliptical_unit_step(float value,
                           float ellipse_height,
                           float ellipse_width,
                           float inflection_point)
{
  if (inflection_point == 0.0) {
    return (value < 0.0) ? 0.0 : 1.0 - elliptical_ramp(1.0 - value, ellipse_height, ellipse_width);
  }
  else if (inflection_point == 1.0) {
    return (value < 1.0) ? elliptical_ramp(value, ellipse_height, ellipse_width) : 1.0;
  }
  else {
    return (value < inflection_point) ?
               inflection_point *
                   elliptical_ramp(value / inflection_point, ellipse_height, ellipse_width) :
               1.0 - (1.0 - inflection_point) *
                         elliptical_ramp((1.0 - value) / (1.0 - inflection_point),
                                         ellipse_height,
                                         ellipse_width);
  }
}

float inverse_mix(float value, float from_min, float from_max)
{
  return (value - from_min) / (from_max - from_min);
}

float elliptical_remap(float value,
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

float chained_elliptical_remap_1_step(float remap[24], float value)
{
  return elliptical_remap(value,
                          /* Step Center 1 - 0.5 * Step Width 1 */
                          remap[0] - 0.5 * remap[1],
                          /* Step Center 1 + 0.5 * Step Width 1 */
                          remap[0] + 0.5 * remap[1],
                          /* Step Value 1 */
                          remap[2],
                          0.0,
                          /* Ellipse Height 1 */
                          remap[3],
                          /* Ellipse Width 1 */
                          remap[4],
                          /* Inflection Point 1 */
                          remap[5]);
}

float chained_elliptical_remap_2_steps(float remap[24], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5 * Step Width 1 */
                                  remap[0] - 0.5 * remap[1],
                                  /* Step Center 1 + 0.5 * Step Width 1 */
                                  remap[0] + 0.5 * remap[1],
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
                          /* Step Center 2 - 0.5 * Step Width 2 */
                          remap[6] - 0.5 * remap[7],
                          /* Step Center 2 + 0.5 * Step Width 2 */
                          remap[6] + 0.5 * remap[7],
                          result,
                          0.0,
                          /* Ellipse Height 2 */
                          remap[9],
                          /* Ellipse Width 2 */
                          remap[10],
                          /* Inflection Point 2 */
                          remap[11]);
}

float chained_elliptical_remap_3_steps(float remap[24], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5 * Step Width 1 */
                                  remap[0] - 0.5 * remap[1],
                                  /* Step Center 1 + 0.5 * Step Width 1 */
                                  remap[0] + 0.5 * remap[1],
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
                            /* Step Center 2 - 0.5 * Step Width 2 */
                            remap[6] - 0.5 * remap[7],
                            /* Step Center 2 + 0.5 * Step Width 2 */
                            remap[6] + 0.5 * remap[7],
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
                          /* Step Center 3 - 0.5 * Step Width 3 */
                          remap[12] - 0.5 * remap[13],
                          /* Step Center 3 + 0.5 * Step Width 3 */
                          remap[12] + 0.5 * remap[13],
                          result,
                          0.0,
                          /* Ellipse Height 3 */
                          remap[15],
                          /* Ellipse Width 3 */
                          remap[16],
                          /* Inflection Point 3 */
                          remap[17]);
}

float chained_elliptical_remap_4_steps(float remap[24], float value)
{
  float result = elliptical_remap(value,
                                  /* Step Center 1 - 0.5 * Step Width 1 */
                                  remap[0] - 0.5 * remap[1],
                                  /* Step Center 1 + 0.5 * Step Width 1 */
                                  remap[0] + 0.5 * remap[1],
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
                            /* Step Center 2 - 0.5 * Step Width 2 */
                            remap[6] - 0.5 * remap[7],
                            /* Step Center 2 + 0.5 * Step Width 2 */
                            remap[6] + 0.5 * remap[7],
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
                            /* Step Center 3 - 0.5 * Step Width 3 */
                            remap[12] - 0.5 * remap[13],
                            /* Step Center 3 + 0.5 * Step Width 3 */
                            remap[12] + 0.5 * remap[13],
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
                          /* Step Center 4 - 0.5 * Step Width 4 */
                          remap[18] - 0.5 * remap[19],
                          /* Step Center 4 + 0.5 * Step Width 4 */
                          remap[18] + 0.5 * remap[19],
                          result,
                          0.0,
                          /* Ellipse Height 4 */
                          remap[21],
                          /* Ellipse Width 4 */
                          remap[22],
                          /* Inflection Point 4 */
                          remap[23]);
}

#define CHAINED_ELLIPTICAL_REMAP_SELECT_STEPS(T) \
  float chained_elliptical_remap_select_steps(int step_count, \
                                              float remap[24], \
                                              float remap_min[24], \
                                              float remap_max[24], \
                                              int remap_index_list[24], \
                                              int remap_index_count, \
                                              T seed_offset, \
                                              float value) \
  { \
    float remap_randomized[24]; \
    float result; \
    switch (step_count) { \
      case 1: { \
        for (int i = 0; i < 6; ++i) { \
          remap_randomized[i] = remap[i]; \
        } \
        randomize_float_array(remap_randomized, \
                              remap_min, \
                              remap_max, \
                              remap_index_list, \
                              remap_index_count, \
                              seed_offset); \
        result = chained_elliptical_remap_1_step(remap_randomized, value); \
        break; \
      } \
      case 2: { \
        for (int i = 0; i < 12; ++i) { \
          remap_randomized[i] = remap[i]; \
        } \
        randomize_float_array(remap_randomized, \
                              remap_min, \
                              remap_max, \
                              remap_index_list, \
                              remap_index_count, \
                              seed_offset); \
        result = chained_elliptical_remap_2_steps(remap_randomized, value); \
        break; \
      } \
      case 3: { \
        for (int i = 0; i < 18; ++i) { \
          remap_randomized[i] = remap[i]; \
        } \
        randomize_float_array(remap_randomized, \
                              remap_min, \
                              remap_max, \
                              remap_index_list, \
                              remap_index_count, \
                              seed_offset); \
        result = chained_elliptical_remap_3_steps(remap_randomized, value); \
        break; \
      } \
      case 4: { \
        for (int i = 0; i < 24; ++i) { \
          remap_randomized[i] = remap[i]; \
        } \
        randomize_float_array(remap_randomized, \
                              remap_min, \
                              remap_max, \
                              remap_index_list, \
                              remap_index_count, \
                              seed_offset); \
        result = chained_elliptical_remap_4_steps(remap_randomized, value); \
        break; \
      } \
    } \
    return result; \
  }

CHAINED_ELLIPTICAL_REMAP_SELECT_STEPS(float)
CHAINED_ELLIPTICAL_REMAP_SELECT_STEPS(vec2)
CHAINED_ELLIPTICAL_REMAP_SELECT_STEPS(vec3)
CHAINED_ELLIPTICAL_REMAP_SELECT_STEPS(vec4)

vec4 rotate_scale(vec4 coord,
                  float translation_rotation_randomized[7],
                  float scale_randomized[4],
                  bool invert_order_of_transformation)
{
  if (invert_order_of_transformation) {
    coord =
        vec4(scale_randomized[0], scale_randomized[1], scale_randomized[2], scale_randomized[3]) *
        coord;

    if (translation_rotation_randomized[4] != 0.0) {
      coord = vec4(coord.x,
                   cos(translation_rotation_randomized[4]) * coord.y -
                       sin(translation_rotation_randomized[4]) * coord.z,
                   sin(translation_rotation_randomized[4]) * coord.y +
                       cos(translation_rotation_randomized[4]) * coord.z,
                   coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0) {
      coord = vec4(cos(translation_rotation_randomized[5]) * coord.x +
                       sin(translation_rotation_randomized[5]) * coord.z,
                   coord.y,
                   cos(translation_rotation_randomized[5]) * coord.z -
                       sin(translation_rotation_randomized[5]) * coord.x,
                   coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0) {
      coord = vec4(cos(translation_rotation_randomized[6]) * coord.x -
                       sin(translation_rotation_randomized[6]) * coord.y,
                   sin(translation_rotation_randomized[6]) * coord.x +
                       cos(translation_rotation_randomized[6]) * coord.y,
                   coord.z,
                   coord.w);
    }
  }
  else {
    if (translation_rotation_randomized[4] != 0.0) {
      coord = vec4(coord.x,
                   cos(translation_rotation_randomized[4]) * coord.y -
                       sin(translation_rotation_randomized[4]) * coord.z,
                   sin(translation_rotation_randomized[4]) * coord.y +
                       cos(translation_rotation_randomized[4]) * coord.z,
                   coord.w);
    }
    if (translation_rotation_randomized[5] != 0.0) {
      coord = vec4(cos(translation_rotation_randomized[5]) * coord.x +
                       sin(translation_rotation_randomized[5]) * coord.z,
                   coord.y,
                   cos(translation_rotation_randomized[5]) * coord.z -
                       sin(translation_rotation_randomized[5]) * coord.x,
                   coord.w);
    }
    if (translation_rotation_randomized[6] != 0.0) {
      coord = vec4(cos(translation_rotation_randomized[6]) * coord.x -
                       sin(translation_rotation_randomized[6]) * coord.y,
                   sin(translation_rotation_randomized[6]) * coord.x +
                       cos(translation_rotation_randomized[6]) * coord.y,
                   coord.z,
                   coord.w);
    }

    coord =
        vec4(scale_randomized[0], scale_randomized[1], scale_randomized[2], scale_randomized[3]) *
        coord;
  }

  return coord;
}

vec4 random_vec4_offset(vec2 seed_offset)
{
  return vec4(100.0, 100.0, 100.0, 100.0) +
         100.0 * vec4(hash_vec3_to_float(vec3(seed_offset.x, seed_offset.y, 1.0)),
                      hash_vec3_to_float(vec3(seed_offset.x, seed_offset.y, 2.0)),
                      hash_vec3_to_float(vec3(seed_offset.x, seed_offset.y, 3.0)),
                      hash_vec3_to_float(vec3(seed_offset.x, seed_offset.y, 4.0)));
}

vec4 random_vec4_offset(vec3 seed_offset)
{
  return vec4(100.0, 100.0, 100.0, 100.0) +
         100.0 * vec4(hash_vec3_to_float(seed_offset + vec3(0.0, 0.0, 1.0)),
                      hash_vec3_to_float(seed_offset + vec3(0.0, 0.0, 2.0)),
                      hash_vec3_to_float(seed_offset + vec3(0.0, 0.0, 3.0)),
                      hash_vec3_to_float(seed_offset + vec3(0.0, 0.0, 4.0)));
}

vec4 random_vec4_offset(vec4 seed_offset)
{
  return vec4(100.0, 100.0, 100.0, 100.0) +
         100.0 * vec4(hash_vec4_to_float(seed_offset + vec4(0.0, 0.0, 1.0, 1.0)),
                      hash_vec4_to_float(seed_offset + vec4(0.0, 0.0, 2.0, 2.0)),
                      hash_vec4_to_float(seed_offset + vec4(0.0, 0.0, 3.0, 3.0)),
                      hash_vec4_to_float(seed_offset + vec4(0.0, 0.0, 4.0, 4.0)));
}

/* Noise Texture fBM optimized for Raiko Texture. */
float raiko_noise_fbm(vec4 coord, float detail, float roughness, float lacunarity)
{
  float octave_scale = 1.0;
  float amplitude = 1.0;
  float max_amplitude = 1.0;
  float sum = 0.0;

  for (int i = 0; i <= int(detail); ++i) {
    sum += amplitude * snoise(octave_scale * coord);
    max_amplitude += amplitude;
    amplitude *= roughness;
    octave_scale *= lacunarity;
  }

  float remainder = detail - floor(detail);
  return (remainder != 0.0) ? ((sum + remainder * amplitude * snoise(octave_scale * coord)) /
                               (max_amplitude + remainder * amplitude)) :
                              (sum / max_amplitude);
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_1(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0 */
vec4 raiko_noise_fbm_layer_1(DeterministicVariables dv, vec4 coord)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  vec4(178.498459, 183.790161, 114.143784, 163.889908),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  vec4(147.634079, 195.179962, 158.144135, 128.116669),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  vec4(195.063629, 144.612671, 155.014709, 165.883881),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  vec4(115.671997, 104.330322, 135.032425, 120.330460),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1));
}

vec4 raiko_noise_fbm_layer_1(DeterministicVariables dv, vec4 coord, float seed_offset)
{
  return vec4(
      raiko_noise_fbm(dv.noise_scale_1 * coord + random_vec4_offset(vec2(seed_offset + 1.0, 0.0)),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord + random_vec4_offset(vec2(seed_offset + 2.0, 0.0)),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord + random_vec4_offset(vec2(seed_offset + 3.0, 0.0)),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1),
      raiko_noise_fbm(dv.noise_scale_1 * coord + random_vec4_offset(vec2(seed_offset + 4.0, 0.0)),
                      dv.noise_detail_1,
                      dv.noise_roughness_1,
                      dv.noise_lacunarity_1));
}

vec4 raiko_noise_fbm_layer_1(DeterministicVariables dv, vec4 coord, vec2 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(vec2(seed_offset.x + 1.0, seed_offset.y)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(vec2(seed_offset.x + 2.0, seed_offset.y)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(vec2(seed_offset.x + 3.0, seed_offset.y)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(vec2(seed_offset.x + 4.0, seed_offset.y)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1));
}

vec4 raiko_noise_fbm_layer_1(DeterministicVariables dv, vec4 coord, vec3 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec3(1.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec3(2.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec3(3.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec3(4.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1));
}

vec4 raiko_noise_fbm_layer_1(DeterministicVariables dv, vec4 coord, vec4 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec4(1.0, 0.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec4(2.0, 0.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec4(3.0, 0.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1),
              raiko_noise_fbm(dv.noise_scale_1 * coord +
                                  random_vec4_offset(seed_offset + vec4(4.0, 0.0, 0.0, 0.0)),
                              dv.noise_detail_1,
                              dv.noise_roughness_1,
                              dv.noise_lacunarity_1));
}

/* Random offsets are the same as when calling raiko_noise_fbm_layer_2(DeterministicVariables dv,
 * vec4 coord, float seed_offset) with seed_offset == 0.0 */
vec4 raiko_noise_fbm_layer_2(DeterministicVariables dv, vec4 coord)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  vec4(115.225372, 181.849701, 148.865616, 148.047165),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  vec4(132.636856, 169.415527, 110.008087, 130.162735),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  vec4(187.223145, 167.974121, 156.358246, 121.253998),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  vec4(119.618362, 126.933167, 161.577881, 147.723999),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2));
}

vec4 raiko_noise_fbm_layer_2(DeterministicVariables dv, vec4 coord, float seed_offset)
{
  return vec4(
      raiko_noise_fbm(dv.noise_scale_2 * coord + random_vec4_offset(vec2(seed_offset, 1.0)),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord + random_vec4_offset(vec2(seed_offset, 2.0)),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord + random_vec4_offset(vec2(seed_offset, 3.0)),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2),
      raiko_noise_fbm(dv.noise_scale_2 * coord + random_vec4_offset(vec2(seed_offset, 4.0)),
                      dv.noise_detail_2,
                      dv.noise_roughness_2,
                      dv.noise_lacunarity_2));
}

vec4 raiko_noise_fbm_layer_2(DeterministicVariables dv, vec4 coord, vec2 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(vec2(seed_offset.x, seed_offset.y + 1.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(vec2(seed_offset.x, seed_offset.y + 2.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(vec2(seed_offset.x, seed_offset.y + 3.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(vec2(seed_offset.x, seed_offset.y + 4.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2));
}

vec4 raiko_noise_fbm_layer_2(DeterministicVariables dv, vec4 coord, vec3 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec3(0.0, 1.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec3(0.0, 2.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec3(0.0, 3.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec3(0.0, 4.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2));
}

vec4 raiko_noise_fbm_layer_2(DeterministicVariables dv, vec4 coord, vec4 seed_offset)
{
  return vec4(raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec4(0.0, 1.0, 0.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec4(0.0, 2.0, 0.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec4(0.0, 3.0, 0.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2),
              raiko_noise_fbm(dv.noise_scale_2 * coord +
                                  random_vec4_offset(seed_offset + vec4(0.0, 4.0, 0.0, 0.0)),
                              dv.noise_detail_2,
                              dv.noise_roughness_2,
                              dv.noise_lacunarity_2));
}

vec4 rotate_noise(vec4 noise_vector, float noise_fragmentation, float index)
{
  float deterministic_angle = noise_fragmentation * M_TAU *
                              (floored_modulo(index + 0.0625, 7.0) +
                               3.0 * floored_modulo(index + 0.0625, 2.0));
  vec4 noise_vector_rotated = vec4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU * 5.0 * hash_float_to_float(index);
  return vec4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

vec4 rotate_noise(vec4 noise_vector, float noise_fragmentation, vec2 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU *
                              (floored_modulo(index.x + 0.0625, 7.0) +
                               3.0 * floored_modulo(index.x + 0.0625, 2.0) +
                               13.0 * (floored_modulo(index.y + 0.0625, 7.0) +
                                       3.0 * floored_modulo(index.y + 0.0625, 2.0)));
  vec4 noise_vector_rotated = vec4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU * 5.0 * hash_vec2_to_float(index);
  return vec4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

vec4 rotate_noise(vec4 noise_vector, float noise_fragmentation, vec3 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU *
                              (floored_modulo(index.x + 0.0625, 7.0) +
                               3.0 * floored_modulo(index.x + 0.0625, 2.0) +
                               13.0 * (floored_modulo(index.y + 0.0625, 7.0) +
                                       3.0 * floored_modulo(index.y + 0.0625, 2.0)) +
                               143.0 * (floored_modulo(index.z + 0.0625, 7.0) +
                                        3.0 * floored_modulo(index.z + 0.0625, 2.0)));
  vec4 noise_vector_rotated = vec4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU * 5.0 * hash_vec3_to_float(index);
  return vec4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

vec4 rotate_noise(vec4 noise_vector, float noise_fragmentation, vec4 index)
{
  float deterministic_angle = noise_fragmentation * M_TAU *
                              (floored_modulo(index.x + 0.0625, 7.0) +
                               3.0 * floored_modulo(index.x + 0.0625, 2.0) +
                               13.0 * (floored_modulo(index.y + 0.0625, 7.0) +
                                       3.0 * floored_modulo(index.y + 0.0625, 2.0)) +
                               143.0 * (floored_modulo(index.z + 0.0625, 7.0) +
                                        3.0 * floored_modulo(index.z + 0.0625, 2.0)) +
                               2431.0 * (floored_modulo(index.w + 0.0625, 7.0) +
                                         3.0 * floored_modulo(index.w + 0.0625, 2.0)));
  vec4 noise_vector_rotated = vec4(
      cos(deterministic_angle) * noise_vector.x - sin(deterministic_angle) * noise_vector.y,
      sin(deterministic_angle) * noise_vector.x + cos(deterministic_angle) * noise_vector.y,
      cos(deterministic_angle) * noise_vector.z - sin(deterministic_angle) * noise_vector.w,
      sin(deterministic_angle) * noise_vector.z + cos(deterministic_angle) * noise_vector.w);
  float random_angle = noise_fragmentation * M_TAU * 5.0 * hash_vec4_to_float(index);
  return vec4(
      cos(random_angle) * noise_vector_rotated.x - sin(random_angle) * noise_vector_rotated.z,
      cos(random_angle) * noise_vector_rotated.y - sin(random_angle) * noise_vector_rotated.w,
      sin(random_angle) * noise_vector_rotated.x + cos(random_angle) * noise_vector_rotated.z,
      sin(random_angle) * noise_vector_rotated.y + cos(random_angle) * noise_vector_rotated.w);
}

OutVariables raiko_select_mode_0d(DeterministicVariables dv,
                                  float r_sphere[4],
                                  float r_sphere_min[4],
                                  float r_sphere_max[4],
                                  int r_sphere_index_list[4],
                                  int r_sphere_index_count,
                                  float translation_rotation[7],
                                  float translation_rotation_min[7],
                                  float translation_rotation_max[7],
                                  int translation_rotation_index_list[7],
                                  int translation_rotation_index_count,
                                  float scale[4],
                                  float scale_randomness[4],
                                  int scale_index_list[4],
                                  int scale_index_count,
                                  float remap[24],
                                  float remap_min[24],
                                  float remap_max[24],
                                  int remap_index_list[24],
                                  int remap_index_count)
{
  OutVariables ov;
  vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                  raiko_noise_fbm_layer_1(
                                      dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
                                  vec4(0.0, 0.0, 0.0, 0.0);
  vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                  raiko_noise_fbm_layer_2(
                                      dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
                                  vec4(0.0, 0.0, 0.0, 0.0);
  vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                             dv.noise_fields_strength_2 * fields_noise_layer_2;

  float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
  randomize_float_array(r_sphere_randomized,
                        r_sphere_min,
                        r_sphere_max,
                        r_sphere_index_list,
                        r_sphere_index_count,
                        0.0);
  float translation_rotation_randomized[7];
  for (int n = 0; n < 7; ++n) {
    translation_rotation_randomized[n] = translation_rotation[n];
  }
  randomize_float_array(translation_rotation_randomized,
                        translation_rotation_min,
                        translation_rotation_max,
                        translation_rotation_index_list,
                        translation_rotation_index_count,
                        0.0);
  float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
  randomize_scale(scale_randomized,
                  scale_randomness,
                  scale_index_list,
                  scale_index_count,
                  dv.uniform_scale_randomness,
                  0.0);

  vec4 iteration_position = vec4(translation_rotation_randomized[0],
                                 translation_rotation_randomized[1],
                                 translation_rotation_randomized[2],
                                 translation_rotation_randomized[3]);
  vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                      translation_rotation_randomized,
                                      scale_randomized,
                                      dv.invert_order_of_transformation);
  vec4 iteration_coord = noiseless_coord +
                         (dv.noise_fragmentation_non_zero ?
                              rotate_noise(fields_noise_vector, dv.noise_fragmentation, 0.0) :
                              fields_noise_vector);

  if (dv.mode == SHD_RAIKO_ADDITIVE) {
    ov.out_r_sphere_field = chained_elliptical_remap_select_steps(
        dv.step_count,
        remap,
        remap_min,
        remap_max,
        remap_index_list,
        remap_index_count,
        0.0,
        calculate_l_angle_bisector_4d(dv.integer_sides,
                                      dv.elliptical_corners,
                                      r_sphere_randomized[0],
                                      r_sphere_randomized[1],
                                      r_sphere_randomized[2],
                                      r_sphere_randomized[3],
                                      iteration_coord));
  }
  else {
    vec4 coordinates_noise_layer_1 = dv.calculate_coordinates_noise_1 ?
                                         raiko_noise_fbm_layer_1(
                                             dv,
                                             dv.transform_coordinates_noise ?
                                                 rotate_scale(dv.coord,
                                                              translation_rotation,
                                                              scale,
                                                              dv.invert_order_of_transformation) :
                                                 dv.coord) :
                                         fields_noise_layer_1;
    vec4 coordinates_noise_layer_2 = dv.calculate_coordinates_noise_2 ?
                                         raiko_noise_fbm_layer_2(
                                             dv,
                                             dv.transform_coordinates_noise ?
                                                 rotate_scale(dv.coord,
                                                              translation_rotation,
                                                              scale,
                                                              dv.invert_order_of_transformation) :
                                                 dv.coord) :
                                         fields_noise_layer_2;
    vec4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                    dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

    vec4 iteration_r_sphere_coordinates =
        noiseless_coord +
        (dv.noise_fragmentation_non_zero ?
             rotate_noise(coordinates_noise_vector, dv.noise_fragmentation, 0.0) :
             coordinates_noise_vector);

    vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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

OutVariables raiko_select_mode_1d(DeterministicVariables dv,
                                  float r_sphere[4],
                                  float r_sphere_min[4],
                                  float r_sphere_max[4],
                                  int r_sphere_index_list[4],
                                  int r_sphere_index_count,
                                  float translation_rotation[7],
                                  float translation_rotation_min[7],
                                  float translation_rotation_max[7],
                                  int translation_rotation_index_list[7],
                                  int translation_rotation_index_count,
                                  float scale[4],
                                  float scale_randomness[4],
                                  int scale_index_list[4],
                                  int scale_index_count,
                                  float remap[24],
                                  float remap_min[24],
                                  float remap_max[24],
                                  int remap_index_list[24],
                                  int remap_index_count,
                                  vec4 initial_index,
                                  vec4 initial_position)
{
  float scanning_window_size = ceil(4.0 * dv.accuracy);
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0;
      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float iteration_index = i + initial_index.x;
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0 * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0 * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0 * iteration_index);

        vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                  vec4(translation_rotation_randomized[0],
                                       translation_rotation_randomized[1],
                                       translation_rotation_randomized[2],
                                       translation_rotation_randomized[3]);
        vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                            translation_rotation_randomized,
                                            scale_randomized,
                                            dv.invert_order_of_transformation);
        vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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
            27.0 * iteration_index,
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
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];

      vec4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                     raiko_noise_fbm_layer_1(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                     raiko_noise_fbm_layer_2(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float iteration_index = i + initial_index.x;
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0 * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0 * iteration_index);

        vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                  vec4(translation_rotation_randomized[0],
                                       translation_rotation_randomized[1],
                                       translation_rotation_randomized[2],
                                       translation_rotation_randomized[3]);
        vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                            translation_rotation_randomized,
                                            scale_randomized,
                                            dv.invert_order_of_transformation);
        vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                      rotate_noise(index_noise_vector,
                                                                   dv.noise_fragmentation,
                                                                   iteration_index) :
                                                      index_noise_vector);

        float l_iteration_coord = euclidean_norm(iteration_coord);
        if (l_iteration_coord < min_distance) {
          min_distance = l_iteration_coord;
          ov.out_index_field.x = iteration_index;
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

      vec4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field.x) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field.x) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                             closest_fields_noise_layer_1 +
                                         dv.noise_fields_strength_2 * closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field.x) :
              closest_fields_noise_layer_1;
      vec4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field.x) :
              closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                  closest_coordinates_noise_layer_1 +
                                              dv.noise_coordinates_strength_2 *
                                                  closest_coordinates_noise_layer_2;

      vec4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                          closest_rotation_randomized,
                                          closest_scale_randomized,
                                          dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0 * ov.out_index_field.x) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0 * ov.out_index_field.x);

      vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                                                                      7.0 * ov.out_index_field.x) :
                                                         closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0;
      ov.r_gon_parameter_field = 0.0;
      ov.max_unit_parameter_field = 0.0;
      ov.segment_id_field = 0.0;
      ov.out_r_sphere_coordinates = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_position_field = vec4(0.0, 0.0, 0.0, 0.0);
      bool first_iteration = true;

      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;
      vec4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      vec4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      vec4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

      for (float i = -scanning_window_size; i <= scanning_window_size; ++i) {
        float iteration_index = i + initial_index.x;
        float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
        randomize_float_array(r_sphere_randomized,
                              r_sphere_min,
                              r_sphere_max,
                              r_sphere_index_list,
                              r_sphere_index_count,
                              5.0 * iteration_index);
        float translation_rotation_randomized[7];
        for (int n = 0; n < 7; ++n) {
          translation_rotation_randomized[n] = translation_rotation[n];
        }
        randomize_float_array(translation_rotation_randomized,
                              translation_rotation_min,
                              translation_rotation_max,
                              translation_rotation_index_list,
                              translation_rotation_index_count,
                              8.0 * iteration_index);
        float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
        randomize_scale(scale_randomized,
                        scale_randomness,
                        scale_index_list,
                        scale_index_count,
                        dv.uniform_scale_randomness,
                        9.0 * iteration_index);

        vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                  vec4(translation_rotation_randomized[0],
                                       translation_rotation_randomized[1],
                                       translation_rotation_randomized[2],
                                       translation_rotation_randomized[3]);
        vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                            translation_rotation_randomized,
                                            scale_randomized,
                                            dv.invert_order_of_transformation);
        vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                      rotate_noise(fields_noise_vector,
                                                                   dv.noise_fragmentation,
                                                                   iteration_index) :
                                                      fields_noise_vector);
        vec4 iteration_r_sphere_coordinates =
            noiseless_coord +
            (dv.noise_fragmentation_non_zero ?
                 rotate_noise(coordinates_noise_vector, dv.noise_fragmentation, iteration_index) :
                 coordinates_noise_vector);

        vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                  1.0 :
                  smoothstep(0.0,
                             1.0,
                             0.5 + 0.5 * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                       dv.smoothness);
          float substraction_factor = dv.smoothness * interpolation_factor *
                                      (1.0 - interpolation_factor);
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
          ov.out_index_field.x = mix(ov.out_index_field.x, iteration_index, interpolation_factor);
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
          ov.out_index_field.x = iteration_index;
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

OutVariables raiko_select_mode_2d(DeterministicVariables dv,
                                  float r_sphere[4],
                                  float r_sphere_min[4],
                                  float r_sphere_max[4],
                                  int r_sphere_index_list[4],
                                  int r_sphere_index_count,
                                  float translation_rotation[7],
                                  float translation_rotation_min[7],
                                  float translation_rotation_max[7],
                                  int translation_rotation_index_list[7],
                                  int translation_rotation_index_count,
                                  float scale[4],
                                  float scale_randomness[4],
                                  int scale_index_list[4],
                                  int scale_index_count,
                                  float remap[24],
                                  float remap_min[24],
                                  float remap_max[24],
                                  int remap_index_list[24],
                                  int remap_index_count,
                                  vec4 initial_index,
                                  vec4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_shortest_grid_vector = min(l_grid_vector1, l_grid_vector2);
  vec2 scanning_window_size = ceil(
      4.0 * dv.accuracy *
      vec2(l_shortest_grid_vector / l_grid_vector1, l_shortest_grid_vector / l_grid_vector2));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0;
      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          vec2 iteration_index = vec2(i, j) + vec2(initial_index.x, initial_index.y);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0 * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0 * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0 * iteration_index);

          vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    j * dv.grid_vector_2 +
                                    vec4(translation_rotation_randomized[0],
                                         translation_rotation_randomized[1],
                                         translation_rotation_randomized[2],
                                         translation_rotation_randomized[3]);
          vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
          vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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
              27.0 * iteration_index,
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
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      vec4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                     raiko_noise_fbm_layer_1(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                     raiko_noise_fbm_layer_2(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          vec2 iteration_index = vec2(i, j) + vec2(initial_index.x, initial_index.y);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0 * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0 * iteration_index);

          vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    j * dv.grid_vector_2 +
                                    vec4(translation_rotation_randomized[0],
                                         translation_rotation_randomized[1],
                                         translation_rotation_randomized[2],
                                         translation_rotation_randomized[3]);
          vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
          vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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

      vec4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * vec2(ov.out_index_field.x, ov.out_index_field.y)) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * vec2(ov.out_index_field.x, ov.out_index_field.y)) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                             closest_fields_noise_layer_1 +
                                         dv.noise_fields_strength_2 * closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * vec2(ov.out_index_field.x, ov.out_index_field.y)) :
              closest_fields_noise_layer_1;
      vec4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * vec2(ov.out_index_field.x, ov.out_index_field.y)) :
              closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                  closest_coordinates_noise_layer_1 +
                                              dv.noise_coordinates_strength_2 *
                                                  closest_coordinates_noise_layer_2;

      vec4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                          closest_rotation_randomized,
                                          closest_scale_randomized,
                                          dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0 * vec2(ov.out_index_field.x,
                                                                 ov.out_index_field.y)) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0 * vec2(ov.out_index_field.x, ov.out_index_field.y));

      vec4 out_fields = calculate_out_fields_4d(
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
                                7.0 * vec2(ov.out_index_field.x, ov.out_index_field.y)) :
                   closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0;
      ov.r_gon_parameter_field = 0.0;
      ov.max_unit_parameter_field = 0.0;
      ov.segment_id_field = 0.0;
      ov.out_r_sphere_coordinates = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_position_field = vec4(0.0, 0.0, 0.0, 0.0);
      bool first_iteration = true;

      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;
      vec4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      vec4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      vec4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

      for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
        for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
          vec2 iteration_index = vec2(i, j) + vec2(initial_index.x, initial_index.y);
          float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
          randomize_float_array(r_sphere_randomized,
                                r_sphere_min,
                                r_sphere_max,
                                r_sphere_index_list,
                                r_sphere_index_count,
                                5.0 * iteration_index);
          float translation_rotation_randomized[7];
          for (int n = 0; n < 7; ++n) {
            translation_rotation_randomized[n] = translation_rotation[n];
          }
          randomize_float_array(translation_rotation_randomized,
                                translation_rotation_min,
                                translation_rotation_max,
                                translation_rotation_index_list,
                                translation_rotation_index_count,
                                8.0 * iteration_index);
          float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
          randomize_scale(scale_randomized,
                          scale_randomness,
                          scale_index_list,
                          scale_index_count,
                          dv.uniform_scale_randomness,
                          9.0 * iteration_index);

          vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                    j * dv.grid_vector_2 +
                                    vec4(translation_rotation_randomized[0],
                                         translation_rotation_randomized[1],
                                         translation_rotation_randomized[2],
                                         translation_rotation_randomized[3]);
          vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                              translation_rotation_randomized,
                                              scale_randomized,
                                              dv.invert_order_of_transformation);
          vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                        rotate_noise(fields_noise_vector,
                                                                     dv.noise_fragmentation,
                                                                     iteration_index) :
                                                        fields_noise_vector);
          vec4 iteration_r_sphere_coordinates = noiseless_coord +
                                                (dv.noise_fragmentation_non_zero ?
                                                     rotate_noise(coordinates_noise_vector,
                                                                  dv.noise_fragmentation,
                                                                  iteration_index) :
                                                     coordinates_noise_vector);

          vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                                             1.0 :
                                             smoothstep(0.0,
                                                        1.0,
                                                        0.5 + 0.5 *
                                                                  (ov.out_r_sphere_field -
                                                                   iteration_l_angle_bisector_4d) /
                                                                  dv.smoothness);
            float substraction_factor = dv.smoothness * interpolation_factor *
                                        (1.0 - interpolation_factor);
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

OutVariables raiko_select_mode_3d(DeterministicVariables dv,
                                  float r_sphere[4],
                                  float r_sphere_min[4],
                                  float r_sphere_max[4],
                                  int r_sphere_index_list[4],
                                  int r_sphere_index_count,
                                  float translation_rotation[7],
                                  float translation_rotation_min[7],
                                  float translation_rotation_max[7],
                                  int translation_rotation_index_list[7],
                                  int translation_rotation_index_count,
                                  float scale[4],
                                  float scale_randomness[4],
                                  int scale_index_list[4],
                                  int scale_index_count,
                                  float remap[24],
                                  float remap_min[24],
                                  float remap_max[24],
                                  int remap_index_list[24],
                                  int remap_index_count,
                                  vec4 initial_index,
                                  vec4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_shortest_grid_vector = min(l_grid_vector1, min(l_grid_vector2, l_grid_vector3));
  vec3 scanning_window_size = ceil(4.0 * dv.accuracy *
                                   vec3(l_shortest_grid_vector / l_grid_vector1,
                                        l_shortest_grid_vector / l_grid_vector2,
                                        l_shortest_grid_vector / l_grid_vector3));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0;
      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            vec3 iteration_index = vec3(i, j, k) +
                                   vec3(initial_index.x, initial_index.y, initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0 * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0 * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0 * iteration_index);

            vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                      vec4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
            vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
            vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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
                27.0 * iteration_index,
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
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      vec4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                     raiko_noise_fbm_layer_1(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                     raiko_noise_fbm_layer_2(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            vec3 iteration_index = vec3(i, j, k) +
                                   vec3(initial_index.x, initial_index.y, initial_index.z);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0 * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0 * iteration_index);

            vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                      vec4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
            vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
            vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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

      vec4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(
                  dv,
                  dv.transform_fields_noise ? rotate_scale(dv.coord - ov.out_position_field,
                                                           closest_rotation_randomized,
                                                           closest_scale_randomized,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord - ov.out_position_field,
                  7.0 * vec3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z)) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(
                  dv,
                  dv.transform_fields_noise ? rotate_scale(dv.coord - ov.out_position_field,
                                                           closest_rotation_randomized,
                                                           closest_scale_randomized,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord - ov.out_position_field,
                  7.0 * vec3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z)) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                             closest_fields_noise_layer_1 +
                                         dv.noise_fields_strength_2 * closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(
                  dv,
                  dv.transform_coordinates_noise ?
                      rotate_scale(dv.coord - ov.out_position_field,
                                   closest_rotation_randomized,
                                   closest_scale_randomized,
                                   dv.invert_order_of_transformation) :
                      dv.coord - ov.out_position_field,
                  7.0 * vec3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z)) :
              closest_fields_noise_layer_1;
      vec4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(
                  dv,
                  dv.transform_coordinates_noise ?
                      rotate_scale(dv.coord - ov.out_position_field,
                                   closest_rotation_randomized,
                                   closest_scale_randomized,
                                   dv.invert_order_of_transformation) :
                      dv.coord - ov.out_position_field,
                  7.0 * vec3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z)) :
              closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                  closest_coordinates_noise_layer_1 +
                                              dv.noise_coordinates_strength_2 *
                                                  closest_coordinates_noise_layer_2;

      vec4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                          closest_rotation_randomized,
                                          closest_scale_randomized,
                                          dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0 * vec3(ov.out_index_field.x,
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
          5.0 * vec3(ov.out_index_field.x, ov.out_index_field.y, ov.out_index_field.z));

      vec4 out_fields = calculate_out_fields_4d(
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
                                              7.0 * vec3(ov.out_index_field.x,
                                                         ov.out_index_field.y,
                                                         ov.out_index_field.z)) :
                                 closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0;
      ov.r_gon_parameter_field = 0.0;
      ov.max_unit_parameter_field = 0.0;
      ov.segment_id_field = 0.0;
      ov.out_r_sphere_coordinates = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_position_field = vec4(0.0, 0.0, 0.0, 0.0);
      bool first_iteration = true;

      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;
      vec4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      vec4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      vec4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

      for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
        for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
          for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
            vec3 iteration_index = vec3(i, j, k) +
                                   vec3(initial_index.x, initial_index.y, initial_index.z);
            float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
            randomize_float_array(r_sphere_randomized,
                                  r_sphere_min,
                                  r_sphere_max,
                                  r_sphere_index_list,
                                  r_sphere_index_count,
                                  5.0 * iteration_index);
            float translation_rotation_randomized[7];
            for (int n = 0; n < 7; ++n) {
              translation_rotation_randomized[n] = translation_rotation[n];
            }
            randomize_float_array(translation_rotation_randomized,
                                  translation_rotation_min,
                                  translation_rotation_max,
                                  translation_rotation_index_list,
                                  translation_rotation_index_count,
                                  8.0 * iteration_index);
            float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
            randomize_scale(scale_randomized,
                            scale_randomness,
                            scale_index_list,
                            scale_index_count,
                            dv.uniform_scale_randomness,
                            9.0 * iteration_index);

            vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                      j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                      vec4(translation_rotation_randomized[0],
                                           translation_rotation_randomized[1],
                                           translation_rotation_randomized[2],
                                           translation_rotation_randomized[3]);
            vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                translation_rotation_randomized,
                                                scale_randomized,
                                                dv.invert_order_of_transformation);
            vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                          rotate_noise(fields_noise_vector,
                                                                       dv.noise_fragmentation,
                                                                       iteration_index) :
                                                          fields_noise_vector);
            vec4 iteration_r_sphere_coordinates = noiseless_coord +
                                                  (dv.noise_fragmentation_non_zero ?
                                                       rotate_noise(coordinates_noise_vector,
                                                                    dv.noise_fragmentation,
                                                                    iteration_index) :
                                                       coordinates_noise_vector);

            vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                      1.0 :
                      smoothstep(
                          0.0,
                          1.0,
                          0.5 + 0.5 * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                    dv.smoothness);
              float substraction_factor = dv.smoothness * interpolation_factor *
                                          (1.0 - interpolation_factor);
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

OutVariables raiko_select_mode_4d(DeterministicVariables dv,
                                  float r_sphere[4],
                                  float r_sphere_min[4],
                                  float r_sphere_max[4],
                                  int r_sphere_index_list[4],
                                  int r_sphere_index_count,
                                  float translation_rotation[7],
                                  float translation_rotation_min[7],
                                  float translation_rotation_max[7],
                                  int translation_rotation_index_list[7],
                                  int translation_rotation_index_count,
                                  float scale[4],
                                  float scale_randomness[4],
                                  int scale_index_list[4],
                                  int scale_index_count,
                                  float remap[24],
                                  float remap_min[24],
                                  float remap_max[24],
                                  int remap_index_list[24],
                                  int remap_index_count,
                                  vec4 initial_index,
                                  vec4 initial_position)
{
  float l_grid_vector1 = euclidean_norm(dv.grid_vector_1);
  float l_grid_vector2 = euclidean_norm(dv.grid_vector_2);
  float l_grid_vector3 = euclidean_norm(dv.grid_vector_3);
  float l_grid_vector4 = euclidean_norm(dv.grid_vector_4);
  float l_shortest_grid_vector = min(l_grid_vector1,
                                     min(l_grid_vector2, min(l_grid_vector3, l_grid_vector4)));
  vec4 scanning_window_size = ceil(4.0 * dv.accuracy *
                                   vec4(l_shortest_grid_vector / l_grid_vector1,
                                        l_shortest_grid_vector / l_grid_vector2,
                                        l_shortest_grid_vector / l_grid_vector3,
                                        l_shortest_grid_vector / l_grid_vector4));
  OutVariables ov;
  switch (dv.mode) {
    case SHD_RAIKO_ADDITIVE: {
      ov.out_r_sphere_field = 0.0;
      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              vec4 iteration_index = vec4(i, j, k, l) + initial_index;
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0 * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0 * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0 * iteration_index);

              vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        l * dv.grid_vector_4 +
                                        vec4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
              vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
              vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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
                  27.0 * iteration_index,
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
    case SHD_RAIKO_CLOSEST: {
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      float min_distance = FLT_MAX;
      float closest_rotation_randomized[7];
      float closest_scale_randomized[4];
      vec4 index_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                     raiko_noise_fbm_layer_1(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                     raiko_noise_fbm_layer_2(
                                         dv,
                                         dv.transform_fields_noise ?
                                             rotate_scale(dv.coord,
                                                          translation_rotation,
                                                          scale,
                                                          dv.invert_order_of_transformation) :
                                             dv.coord) :
                                     vec4(0.0, 0.0, 0.0, 0.0);
      vec4 index_noise_vector = dv.noise_fields_strength_1 * index_noise_layer_1 +
                                dv.noise_fields_strength_2 * index_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              vec4 iteration_index = vec4(i, j, k, l) + initial_index;
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0 * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0 * iteration_index);

              vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        l * dv.grid_vector_4 +
                                        vec4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
              vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
              vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
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

      vec4 closest_fields_noise_layer_1 =
          dv.calculate_fields_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_layer_2 =
          dv.calculate_fields_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_fields_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field) :
              vec4(0.0, 0.0, 0.0, 0.0);
      vec4 closest_fields_noise_vector = dv.noise_fields_strength_1 *
                                             closest_fields_noise_layer_1 +
                                         dv.noise_fields_strength_2 * closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field) :
              closest_fields_noise_layer_1;
      vec4 closest_coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord - ov.out_position_field,
                                                       closest_rotation_randomized,
                                                       closest_scale_randomized,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord - ov.out_position_field,
                                      7.0 * ov.out_index_field) :
              closest_fields_noise_layer_2;
      vec4 closest_coordinates_noise_vector = dv.noise_coordinates_strength_1 *
                                                  closest_coordinates_noise_layer_1 +
                                              dv.noise_coordinates_strength_2 *
                                                  closest_coordinates_noise_layer_2;

      vec4 noiseless_coord = rotate_scale(dv.coord - ov.out_position_field,
                                          closest_rotation_randomized,
                                          closest_scale_randomized,
                                          dv.invert_order_of_transformation);
      ov.out_r_sphere_coordinates = noiseless_coord +
                                    (dv.noise_fragmentation_non_zero ?
                                         rotate_noise(closest_coordinates_noise_vector,
                                                      dv.noise_fragmentation,
                                                      7.0 * ov.out_index_field) :
                                         closest_coordinates_noise_vector);

      float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
      randomize_float_array(r_sphere_randomized,
                            r_sphere_min,
                            r_sphere_max,
                            r_sphere_index_list,
                            r_sphere_index_count,
                            5.0 * ov.out_index_field);

      vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                                                                      7.0 * ov.out_index_field) :
                                                         closest_fields_noise_vector));
      ov.out_r_sphere_field = out_fields.x;
      ov.r_gon_parameter_field = out_fields.y;
      ov.max_unit_parameter_field = out_fields.z;
      ov.segment_id_field = out_fields.w;
      break;
    }
    case SHD_RAIKO_SMOOTH_MINIMUM: {
      ov.out_r_sphere_field = 0.0;
      ov.r_gon_parameter_field = 0.0;
      ov.max_unit_parameter_field = 0.0;
      ov.segment_id_field = 0.0;
      ov.out_r_sphere_coordinates = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_index_field = vec4(0.0, 0.0, 0.0, 0.0);
      ov.out_position_field = vec4(0.0, 0.0, 0.0, 0.0);
      bool first_iteration = true;

      vec4 fields_noise_layer_1 = dv.calculate_fields_noise_1 ?
                                      raiko_noise_fbm_layer_1(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_layer_2 = dv.calculate_fields_noise_2 ?
                                      raiko_noise_fbm_layer_2(
                                          dv,
                                          dv.transform_fields_noise ?
                                              rotate_scale(dv.coord,
                                                           translation_rotation,
                                                           scale,
                                                           dv.invert_order_of_transformation) :
                                              dv.coord) :
                                      vec4(0.0, 0.0, 0.0, 0.0);
      vec4 fields_noise_vector = dv.noise_fields_strength_1 * fields_noise_layer_1 +
                                 dv.noise_fields_strength_2 * fields_noise_layer_2;
      vec4 coordinates_noise_layer_1 =
          dv.calculate_coordinates_noise_1 ?
              raiko_noise_fbm_layer_1(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_1;
      vec4 coordinates_noise_layer_2 =
          dv.calculate_coordinates_noise_2 ?
              raiko_noise_fbm_layer_2(dv,
                                      dv.transform_coordinates_noise ?
                                          rotate_scale(dv.coord,
                                                       translation_rotation,
                                                       scale,
                                                       dv.invert_order_of_transformation) :
                                          dv.coord) :
              fields_noise_layer_2;
      vec4 coordinates_noise_vector = dv.noise_coordinates_strength_1 * coordinates_noise_layer_1 +
                                      dv.noise_coordinates_strength_2 * coordinates_noise_layer_2;

      for (float l = -scanning_window_size.w; l <= scanning_window_size.w; ++l) {
        for (float k = -scanning_window_size.z; k <= scanning_window_size.z; ++k) {
          for (float j = -scanning_window_size.y; j <= scanning_window_size.y; ++j) {
            for (float i = -scanning_window_size.x; i <= scanning_window_size.x; ++i) {
              vec4 iteration_index = vec4(i, j, k, l) + initial_index;
              float r_sphere_randomized[4] = {r_sphere[0], r_sphere[1], r_sphere[2], r_sphere[3]};
              randomize_float_array(r_sphere_randomized,
                                    r_sphere_min,
                                    r_sphere_max,
                                    r_sphere_index_list,
                                    r_sphere_index_count,
                                    5.0 * iteration_index);
              float translation_rotation_randomized[7];
              for (int n = 0; n < 7; ++n) {
                translation_rotation_randomized[n] = translation_rotation[n];
              }
              randomize_float_array(translation_rotation_randomized,
                                    translation_rotation_min,
                                    translation_rotation_max,
                                    translation_rotation_index_list,
                                    translation_rotation_index_count,
                                    8.0 * iteration_index);
              float scale_randomized[4] = {scale[0], scale[1], scale[2], scale[3]};
              randomize_scale(scale_randomized,
                              scale_randomness,
                              scale_index_list,
                              scale_index_count,
                              dv.uniform_scale_randomness,
                              9.0 * iteration_index);

              vec4 iteration_position = initial_position + i * dv.grid_vector_1 +
                                        j * dv.grid_vector_2 + k * dv.grid_vector_3 +
                                        l * dv.grid_vector_4 +
                                        vec4(translation_rotation_randomized[0],
                                             translation_rotation_randomized[1],
                                             translation_rotation_randomized[2],
                                             translation_rotation_randomized[3]);
              vec4 noiseless_coord = rotate_scale(dv.coord - iteration_position,
                                                  translation_rotation_randomized,
                                                  scale_randomized,
                                                  dv.invert_order_of_transformation);
              vec4 iteration_coord = noiseless_coord + (dv.noise_fragmentation_non_zero ?
                                                            rotate_noise(fields_noise_vector,
                                                                         dv.noise_fragmentation,
                                                                         iteration_index) :
                                                            fields_noise_vector);
              vec4 iteration_r_sphere_coordinates = noiseless_coord +
                                                    (dv.noise_fragmentation_non_zero ?
                                                         rotate_noise(coordinates_noise_vector,
                                                                      dv.noise_fragmentation,
                                                                      iteration_index) :
                                                         coordinates_noise_vector);

              vec4 out_fields = calculate_out_fields_4d(dv.calculate_r_sphere_field,
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
                        1.0 :
                        smoothstep(
                            0.0,
                            1.0,
                            0.5 + 0.5 * (ov.out_r_sphere_field - iteration_l_angle_bisector_4d) /
                                      dv.smoothness);
                float substraction_factor = dv.smoothness * interpolation_factor *
                                            (1.0 - interpolation_factor);
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

OutVariables raiko_select_grid_dimensions(DeterministicVariables dv,
                                          float r_sphere[4],
                                          float r_sphere_min[4],
                                          float r_sphere_max[4],
                                          int r_sphere_index_list[4],
                                          int r_sphere_index_count,
                                          float translation_rotation[7],
                                          float translation_rotation_min[7],
                                          float translation_rotation_max[7],
                                          int translation_rotation_index_list[7],
                                          int translation_rotation_index_count,
                                          float scale[4],
                                          float scale_randomness[4],
                                          int scale_index_list[4],
                                          int scale_index_count,
                                          float remap[24],
                                          float remap_min[24],
                                          float remap_max[24],
                                          int remap_index_list[24],
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
      if (dv.grid_vector_1.x != 0.0) {
        dv.grid_vector_1 = vec4(dv.grid_vector_1.x, 0.0, 0.0, 0.0);

        vec4 initial_index = round(vec4(dv.coord.x / dv.grid_vector_1.x, 0.0, 0.0, 0.0));
        vec4 initial_position = initial_index.x * dv.grid_vector_1;

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
      if (M_det != 0.0) {
        dv.grid_vector_1 = vec4(dv.grid_vector_1.x, dv.grid_vector_1.y, 0.0, 0.0);
        dv.grid_vector_2 = vec4(dv.grid_vector_2.x, dv.grid_vector_2.y, 0.0, 0.0);

        vec2 initial_index_xy = round(
            fc_linear_system_solve_non_singular_2x2(vec2(dv.grid_vector_1.x, dv.grid_vector_1.y),
                                                    vec2(dv.grid_vector_2.x, dv.grid_vector_2.y),
                                                    vec2(dv.coord.x, dv.coord.y),
                                                    M_det));
        vec4 initial_index = vec4(initial_index_xy.x, initial_index_xy.y, 0.0, 0.0);
        vec4 initial_position = initial_index.x * dv.grid_vector_1 +
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
          vec3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
          vec3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
          vec3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z));
      if (A_fcc.M_det != 0.0) {
        dv.grid_vector_1 = vec4(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z, 0.0);
        dv.grid_vector_2 = vec4(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z, 0.0);
        dv.grid_vector_3 = vec4(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z, 0.0);

        vec3 initial_index_xyz = round(fc_linear_system_solve_non_singular_3x3(
            vec3(dv.grid_vector_1.x, dv.grid_vector_1.y, dv.grid_vector_1.z),
            vec3(dv.grid_vector_2.x, dv.grid_vector_2.y, dv.grid_vector_2.z),
            vec3(dv.grid_vector_3.x, dv.grid_vector_3.y, dv.grid_vector_3.z),
            vec3(dv.coord.x, dv.coord.y, dv.coord.z),
            A_fcc));
        vec4 initial_index = vec4(
            initial_index_xyz.x, initial_index_xyz.y, initial_index_xyz.z, 0.0);
        vec4 initial_position = initial_index.x * dv.grid_vector_1 +
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
      if (A_fcc.M_det != 0.0) {

        vec4 initial_index = round(fc_linear_system_solve_non_singular_4x4(dv.grid_vector_1,
                                                                           dv.grid_vector_2,
                                                                           dv.grid_vector_3,
                                                                           dv.grid_vector_4,
                                                                           dv.coord,
                                                                           A_fcc));
        vec4 initial_position = initial_index.x * dv.grid_vector_1 +
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

void set_vec2_from_components(float a, float b, out vec2 out_vec2)
{
  out_vec2 = vec2(a, b);
}

void set_vec3_from_components(float a, float b, float c, out vec3 out_vec3)
{
  out_vec3 = vec3(a, b, c);
}

void set_rgba_from_components(float a, float b, float c, float d, out vec4 out_color)
{
  out_color = vec4(a, b, c, d);
}

void set_rgba_from_vec3_and_float(vec3 a, float b, out vec4 out_color)
{
  out_color = vec4(a.x, a.y, a.z, b);
}

void set_components_from_vec4(
    vec4 a, out float out_x, out float out_y, out float out_z, out float out_w)
{
  out_x = a.x;
  out_y = a.y;
  out_z = a.z;
  out_w = a.w;
}

void set_vec3_and_float_from_rgba(vec4 a, out vec3 out_vec3, out float out_w)
{
  out_vec3.x = a.x;
  out_vec3.y = a.y;
  out_vec3.z = a.z;
  out_w = a.w;
}

void node_tex_raiko(vec4 in_calculate,
                    vec4 in_properties_1,
                    vec4 in_properties_2,
                    vec4 in_coord,
                    vec3 in_accuracy_scale_smoothness,
                    vec4 in_r_sphere,
                    vec4 in_r_sphere_randomness,
                    vec3 in_transform_rotation,
                    vec4 in_transform_scale,
                    vec3 in_transform_rotation_randomness,
                    vec4 in_transform_scale_randomness,
                    vec4 in_noise_1,
                    vec4 in_noise_2,
                    vec4 in_noise_3,
                    vec3 in_noise_4,
                    vec4 in_grid_vector_1,
                    vec4 in_grid_vector_2,
                    vec4 in_grid_vector_3,
                    vec4 in_grid_vector_4,
                    vec4 in_grid_points_translation_randomness,
                    vec4 in_step_center,
                    vec4 in_step_width,
                    vec4 in_step_value,
                    vec4 in_ellipse_height,
                    vec4 in_ellipse_width,
                    vec4 in_inflection_point,
                    vec4 in_step_center_randomness,
                    vec4 in_step_width_randomness,
                    vec4 in_step_value_randomness,
                    vec4 in_ellipse_height_randomness,
                    vec4 in_ellipse_width_randomness,
                    vec4 in_inflection_point_randomness,
                    out vec4 out_fields,
                    out vec4 out_index_field,
                    out vec4 out_position_field,
                    out vec4 out_r_sphere_coordinates)
{
  DeterministicVariables dv;

  dv.calculate_r_sphere_field = bool(in_calculate.x);
  dv.calculate_r_gon_parameter_field = bool(in_calculate.y);
  dv.calculate_max_unit_parameter_field = bool(in_calculate.z);
  dv.calculate_coordinates_outputs = bool(in_calculate.w);

  dv.mode = int(in_properties_1.x);
  dv.normalize_r_gon_parameter = bool(in_properties_1.y);
  dv.integer_sides = bool(in_properties_1.z);
  dv.elliptical_corners = bool(in_properties_1.w);
  dv.invert_order_of_transformation = bool(in_properties_2.x);
  dv.transform_fields_noise = bool(in_noise_1.x);
  dv.transform_coordinates_noise = bool(in_noise_1.y);
  dv.uniform_scale_randomness = bool(in_properties_2.y);
  dv.grid_dimensions = int(in_properties_2.z);
  dv.step_count = int(in_properties_2.w);

  dv.accuracy = in_accuracy_scale_smoothness.x;
  dv.scale = in_accuracy_scale_smoothness.y;
  dv.coord = dv.scale * in_coord;
  dv.smoothness = in_accuracy_scale_smoothness.z;
  dv.smoothness_non_zero = dv.smoothness != 0.0;
  /*r_sphere[0] == r_gon_sides; r_sphere[1] == r_gon_roundness; r_sphere[2] == r_gon_exponent;
   * r_sphere[3] == sphere_exponent;*/
  float r_sphere[4] = {in_r_sphere.x, in_r_sphere.y, in_r_sphere.z, in_r_sphere.w};
  float r_sphere_min[4];
  float r_sphere_max[4];
  int r_sphere_index_list[4];
  int r_sphere_index_count;
  int index_count = 0;
  if (in_r_sphere_randomness.x != 0.0) {
    r_sphere_min[index_count] = max(r_sphere[0] - in_r_sphere_randomness.x, 2.0);
    r_sphere_max[index_count] = max(r_sphere[0] + in_r_sphere_randomness.x, 2.0);
    r_sphere_index_list[index_count] = 0;
    ++index_count;
  }
  if (in_r_sphere_randomness.y != 0.0) {
    r_sphere_min[index_count] = clamp(r_sphere[1] - in_r_sphere_randomness.y, 0.0, 1.0);
    r_sphere_max[index_count] = clamp(r_sphere[1] + in_r_sphere_randomness.y, 0.0, 1.0);
    r_sphere_index_list[index_count] = 1;
    ++index_count;
  }
  if (in_r_sphere_randomness.z != 0.0) {
    r_sphere_min[index_count] = max(r_sphere[2] - in_r_sphere_randomness.z, 0.0);
    r_sphere_max[index_count] = max(r_sphere[2] + in_r_sphere_randomness.z, 0.0);
    r_sphere_index_list[index_count] = 2;
    ++index_count;
  }
  if (in_r_sphere_randomness.w != 0.0) {
    r_sphere_min[index_count] = max(r_sphere[3] - in_r_sphere_randomness.w, 0.0);
    r_sphere_max[index_count] = max(r_sphere[3] + in_r_sphere_randomness.w, 0.0);
    r_sphere_index_list[index_count] = 3;
    ++index_count;
  }
  r_sphere_index_count = index_count;
  float translation_rotation[7] = {0.0,
                                   0.0,
                                   0.0,
                                   0.0,
                                   in_transform_rotation.x,
                                   in_transform_rotation.y,
                                   in_transform_rotation.z};
  float translation_rotation_min[7];
  float translation_rotation_max[7];
  int translation_rotation_index_list[7];
  int translation_rotation_index_count;
  index_count = 0;
  if (in_grid_points_translation_randomness.x != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[0] -
                                            in_grid_points_translation_randomness.x;
    translation_rotation_max[index_count] = translation_rotation[0] +
                                            in_grid_points_translation_randomness.x;
    translation_rotation_index_list[index_count] = 0;
    ++index_count;
  }
  if (in_grid_points_translation_randomness.y != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[1] -
                                            in_grid_points_translation_randomness.y;
    translation_rotation_max[index_count] = translation_rotation[1] +
                                            in_grid_points_translation_randomness.y;
    translation_rotation_index_list[index_count] = 1;
    ++index_count;
  }
  if (in_grid_points_translation_randomness.z != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[2] -
                                            in_grid_points_translation_randomness.z;
    translation_rotation_max[index_count] = translation_rotation[2] +
                                            in_grid_points_translation_randomness.z;
    translation_rotation_index_list[index_count] = 2;
    ++index_count;
  }
  if (in_grid_points_translation_randomness.w != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[3] -
                                            in_grid_points_translation_randomness.w;
    translation_rotation_max[index_count] = translation_rotation[3] +
                                            in_grid_points_translation_randomness.w;
    translation_rotation_index_list[index_count] = 3;
    ++index_count;
  }
  if (in_transform_rotation_randomness.x != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[4] -
                                            in_transform_rotation_randomness.x;
    translation_rotation_max[index_count] = translation_rotation[4] +
                                            in_transform_rotation_randomness.x;
    translation_rotation_index_list[index_count] = 4;
    ++index_count;
  }
  if (in_transform_rotation_randomness.y != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[5] -
                                            in_transform_rotation_randomness.y;
    translation_rotation_max[index_count] = translation_rotation[5] +
                                            in_transform_rotation_randomness.y;
    translation_rotation_index_list[index_count] = 5;
    ++index_count;
  }
  if (in_transform_rotation_randomness.z != 0.0) {
    translation_rotation_min[index_count] = translation_rotation[6] -
                                            in_transform_rotation_randomness.z;
    translation_rotation_max[index_count] = translation_rotation[6] +
                                            in_transform_rotation_randomness.z;
    translation_rotation_index_list[index_count] = 6;
    ++index_count;
  }
  translation_rotation_index_count = index_count;
  float scale[4] = {
      in_transform_scale.x, in_transform_scale.y, in_transform_scale.z, in_transform_scale.w};
  float scale_randomness[4];
  int scale_index_list[4];
  int scale_index_count;
  index_count = 0;
  if (!dv.uniform_scale_randomness) {
    if (in_transform_scale_randomness.x != 0.0) {
      scale_randomness[index_count] = in_transform_scale_randomness.x;
      scale_index_list[index_count] = 0;
      ++index_count;
    }
    if (in_transform_scale_randomness.y != 0.0) {
      scale_randomness[index_count] = in_transform_scale_randomness.y;
      scale_index_list[index_count] = 1;
      ++index_count;
    }
    if (in_transform_scale_randomness.z != 0.0) {
      scale_randomness[index_count] = in_transform_scale_randomness.z;
      scale_index_list[index_count] = 2;
      ++index_count;
    }
  }
  if (in_transform_scale_randomness.w != 0.0) {
    scale_randomness[index_count] = in_transform_scale_randomness.w;
    scale_index_list[index_count] = 3;
    ++index_count;
  }
  scale_index_count = index_count;
  dv.noise_fragmentation = in_noise_1.z;
  dv.noise_fields_strength_1 = in_noise_1.w;
  dv.noise_coordinates_strength_1 = in_noise_2.x * float(dv.calculate_coordinates_outputs);
  dv.noise_scale_1 = in_noise_2.y;
  dv.noise_detail_1 = in_noise_2.z;
  dv.noise_roughness_1 = in_noise_2.w;
  dv.noise_lacunarity_1 = in_noise_3.x;
  dv.noise_fields_strength_2 = in_noise_3.y;
  dv.noise_coordinates_strength_2 = in_noise_3.z * float(dv.calculate_coordinates_outputs);
  dv.noise_scale_2 = in_noise_3.w;
  dv.noise_detail_2 = in_noise_4.x;
  dv.noise_roughness_2 = in_noise_4.y;
  dv.noise_lacunarity_2 = in_noise_4.z;
  dv.noise_fragmentation_non_zero = dv.noise_fragmentation != 0.0;
  dv.calculate_fields_noise_1 = dv.noise_fields_strength_1 != 0.0;
  dv.calculate_fields_noise_2 = dv.noise_fields_strength_2 != 0.0;
  dv.calculate_coordinates_noise_1 = (dv.noise_coordinates_strength_1 != 0.0) &&
                                     (!(dv.calculate_fields_noise_1 &&
                                        (dv.transform_fields_noise ==
                                         dv.transform_coordinates_noise)));
  dv.calculate_coordinates_noise_2 = (dv.noise_coordinates_strength_2 != 0.0) &&
                                     (!(dv.calculate_fields_noise_2 &&
                                        (dv.transform_fields_noise ==
                                         dv.transform_coordinates_noise)));
  if (dv.grid_dimensions == 1) {
    dv.grid_vector_1.x = in_grid_vector_1.w;
  }
  else {
    dv.grid_vector_1 = in_grid_vector_1;
    dv.grid_vector_2 = in_grid_vector_2;
    dv.grid_vector_3 = in_grid_vector_3;
    dv.grid_vector_4 = in_grid_vector_4;
  }
  float remap[24];
  float remap_min[24];
  float remap_max[24];
  int remap_index_list[24];
  int remap_index_count;
  index_count = 0;
  if (dv.mode == SHD_RAIKO_ADDITIVE) {
    switch (dv.step_count) {
      case 4: {
        ASSIGN_REMAP_INPUTS(w, 18, 19, 20, 21, 22, 23)
        ATTR_FALLTHROUGH;
      }
      case 3: {
        ASSIGN_REMAP_INPUTS(z, 12, 13, 14, 15, 16, 17)
        ATTR_FALLTHROUGH;
      }
      case 2: {
        ASSIGN_REMAP_INPUTS(y, 6, 7, 8, 9, 10, 11)
        ATTR_FALLTHROUGH;
      }
      case 1: {
        ASSIGN_REMAP_INPUTS(x, 0, 1, 2, 3, 4, 5)
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

  out_fields = vec4(ov.out_r_sphere_field,
                    ov.r_gon_parameter_field,
                    ov.max_unit_parameter_field,
                    ov.segment_id_field);
  if (dv.grid_dimensions == 1) {
    out_index_field.w = ov.out_index_field.x;
  }
  else {
    out_index_field = ov.out_index_field;
  }
  out_position_field = ov.out_position_field;
  out_r_sphere_coordinates = ov.out_r_sphere_coordinates;
}
