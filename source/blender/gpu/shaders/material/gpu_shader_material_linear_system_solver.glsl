/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma BLENDER_REQUIRE(gpu_shader_common_math_utils.glsl)

/* Reduced Cramer Coefficients of the 3x3 matrix M. */
struct rcc_3x3 {
  /* Determinant of the 3x3 matrix M. */
  float M_det;
  /* Determinant of the 2x2 submatrices M_i_j. The first index i denotes the row, the second index
   * j the coloumn being removed from M. */
  float M_1_1_det;
  float M_2_1_det;
  float M_3_1_det;
};

/* Reduced Cramer Coefficients of the 4x4 matrix M. */
struct rcc_4x4 {
  /* Determinant of the 4x4 matrix M. */
  float M_det;
  /* Reduced Cramer Coefficients of the 3x3 submatrices M_i_j. The first index i denotes the row, the
   * second index j the coloumn being removed from M. */
  rcc_3x3 M_1_1_rcc;
  rcc_3x3 M_2_1_rcc;
  rcc_3x3 M_3_1_rcc;
  rcc_3x3 M_4_1_rcc;
};

rcc_3x3 calculate_rcc_3x3(vec3 a_1, vec3 a_2, vec3 a_3)
{
  rcc_3x3 A_rcc;

  A_rcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_rcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_rcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_det - a_1.y * A_rcc.M_2_1_det + a_1.z * A_rcc.M_3_1_det;

  return A_rcc;
}

rcc_4x4 calculate_rcc_4x4(vec4 a_1, vec4 a_2, vec4 a_3, vec4 a_4)
{
  rcc_4x4 A_rcc;

  A_rcc.M_1_1_rcc = calculate_rcc_3x3(
      vec3(a_2.y, a_2.z, a_2.w), vec3(a_3.y, a_3.z, a_3.w), vec3(a_4.y, a_4.z, a_4.w));
  A_rcc.M_2_1_rcc = calculate_rcc_3x3(
      vec3(a_2.x, a_2.z, a_2.w), vec3(a_3.x, a_3.z, a_3.w), vec3(a_4.x, a_4.z, a_4.w));
  A_rcc.M_3_1_rcc = calculate_rcc_3x3(
      vec3(a_2.x, a_2.y, a_2.w), vec3(a_3.x, a_3.y, a_3.w), vec3(a_4.x, a_4.y, a_4.w));
  A_rcc.M_4_1_rcc = calculate_rcc_3x3(
      vec3(a_2.x, a_2.y, a_2.z), vec3(a_3.x, a_3.y, a_3.z), vec3(a_4.x, a_4.y, a_4.z));

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_rcc.M_det - a_1.y * A_rcc.M_2_1_rcc.M_det +
                a_1.z * A_rcc.M_3_1_rcc.M_det - a_1.w * A_rcc.M_4_1_rcc.M_det;

  return A_rcc;
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 2x2 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec2 rc_linear_system_solve_non_singular_2x2(vec2 a_1, vec2 a_2, vec2 b, float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is Reduceder. */
  return vec2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 3x3 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec3 rc_linear_system_solve_non_singular_3x3(vec3 a_1, vec3 a_2, vec3 a_3, vec3 b, rcc_3x3 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_det - b.y * A_rcc.M_2_1_det + b.z * A_rcc.M_3_1_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_det != 0.0) {
    vec2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        vec2(a_2.y, a_2.z),
        vec2(a_3.y, a_3.z),
        vec2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_1_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_rcc.M_2_1_det != 0.0) {
    vec2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        vec2(a_2.x, a_2.z),
        vec2(a_3.x, a_3.z),
        vec2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_2_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    vec2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        vec2(a_2.x, a_2.y),
        vec2(a_3.x, a_3.y),
        vec2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_rcc.M_3_1_det);
    return vec3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of the
 * invertible 4x4 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or not A
 * is invertible. Calling it on a singular matrix leads to division by 0. */
vec4 rc_linear_system_solve_non_singular_4x4(
    vec4 a_1, vec4 a_2, vec4 a_3, vec4 a_4, vec4 b, rcc_4x4 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_rcc.M_det - b.y * A_rcc.M_2_1_rcc.M_det +
                      b.z * A_rcc.M_3_1_rcc.M_det - b.w * A_rcc.M_4_1_rcc.M_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_rcc.M_det != 0.0) {
    vec3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        vec3(a_2.y, a_2.z, a_2.w),
        vec3(a_3.y, a_3.z, a_3.w),
        vec3(a_4.y, a_4.z, a_4.w),
        vec3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_1_1_rcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_2_1_rcc.M_det != 0.0) {
    vec3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.z, a_2.w),
        vec3(a_3.x, a_3.z, a_3.w),
        vec3(a_4.x, a_4.z, a_4.w),
        vec3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_2_1_rcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_3_1_rcc.M_det != 0.0) {
    vec3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.y, a_2.w),
        vec3(a_3.x, a_3.y, a_3.w),
        vec3(a_4.x, a_4.y, a_4.w),
        vec3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_3_1_rcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    vec3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        vec3(a_2.x, a_2.y, a_2.z),
        vec3(a_3.x, a_3.y, a_3.z),
        vec3(a_4.x, a_4.y, a_4.z),
        vec3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_4_1_rcc);
    return vec4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

void node_linear_system_solver(vec3 b_vector,
                                   float b_vector_w,
                                   vec3 column_1,
                                   float column_1_w,
                                   vec3 column_2,
                                   float column_2_w,
                                   vec3 column_3,
                                   float column_3_w,
                                   vec3 column_4,
                                   float column_4_w,
                                   float matrix_dimension,
                                   out vec3 out_solution,
                                   out float out_solution_w)
{
  switch (int(matrix_dimension)) {
    case 1: {
        if (column_1_w != 0.0) {
          out_solution_w = b_vector_w / column_1_w;
        }
        else {
          out_solution_w = 0.0;
        }
      break;
    }
    case 2: {
        float M_det = column_1.x * column_2.y - column_2.x * column_1.y;
        if (M_det != 0.0) {
          vec2 solution_xy = rc_linear_system_solve_non_singular_2x2(
              vec2(column_1.x, column_1.y),
              vec2(column_2.x, column_2.y),
              vec2(b_vector.x, b_vector.y),
              M_det);
          out_solution = vec3(solution_xy.x, solution_xy.y, 0.0);
        }
        else {
          out_solution = vec3(0.0, 0.0, 0.0);
        }
      break;
    }
    case 3: {
        rcc_3x3 A_rcc = calculate_rcc_3x3(column_1, column_2, column_3);
        if (A_rcc.M_det != 0.0) {
          out_solution = rc_linear_system_solve_non_singular_3x3(
              column_1, column_2, column_3, b_vector, A_rcc);
        }
        else {
          out_solution = vec3(0.0, 0.0, 0.0);
        }
      break;
    }
    case 4: {
        rcc_4x4 A_rcc = calculate_rcc_4x4(
            vec4(column_1.x, column_1.y, column_1.z, column_1_w),
            vec4(column_2.x, column_2.y, column_2.z, column_2_w),
            vec4(column_3.x, column_3.y, column_3.z, column_3_w),
            vec4(column_4.x, column_4.y, column_4.z, column_4_w));
        if (A_rcc.M_det != 0.0) {
          vec4 solution_xyzw = rc_linear_system_solve_non_singular_4x4(
              vec4(column_1.x, column_1.y, column_1.z, column_1_w),
              vec4(column_2.x, column_2.y, column_2.z, column_2_w),
              vec4(column_3.x, column_3.y, column_3.z, column_3_w),
              vec4(column_4.x, column_4.y, column_4.z, column_4_w),
              vec4(b_vector.x, b_vector.y, b_vector.z, b_vector_w),
              A_rcc);
            out_solution = vec3(solution_xyzw.x, solution_xyzw.y, solution_xyzw.z);
            out_solution_w = solution_xyzw.w;
        }
        else {
            out_solution = vec3(0.0, 0.0, 0.0);
            out_solution_w = 0.0;
        }
      break;
    }
  }
}
