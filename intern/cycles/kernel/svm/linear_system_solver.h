/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

CCL_NAMESPACE_BEGIN

struct LinearSystemSolverStackOffsets {
  uint b_vector;
  uint b_vector_w;
  uint column_1;
  uint column_1_w;
  uint column_2;
  uint column_2_w;
  uint column_3;
  uint column_3_w;
  uint column_4;
  uint column_4_w;
  uint solution;
  uint solution_w;
};

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
  /* Reduced Cramer Coefficients of the 3x3 submatrices M_i_j. The first index i denotes the row,
   * the second index j the coloumn being removed from M. */
  rcc_3x3 M_1_1_rcc;
  rcc_3x3 M_2_1_rcc;
  rcc_3x3 M_3_1_rcc;
  rcc_3x3 M_4_1_rcc;
};

ccl_device rcc_3x3 calculate_rcc_3x3(float3 a_1, float3 a_2, float3 a_3)
{
  rcc_3x3 A_rcc;

  A_rcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_rcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_rcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_det - a_1.y * A_rcc.M_2_1_det + a_1.z * A_rcc.M_3_1_det;

  return A_rcc;
}

ccl_device rcc_4x4 calculate_rcc_4x4(float4 a_1, float4 a_2, float4 a_3, float4 a_4)
{
  rcc_4x4 A_rcc;

  A_rcc.M_1_1_rcc = calculate_rcc_3x3(make_float3(a_2.y, a_2.z, a_2.w),
                                      make_float3(a_3.y, a_3.z, a_3.w),
                                      make_float3(a_4.y, a_4.z, a_4.w));
  A_rcc.M_2_1_rcc = calculate_rcc_3x3(make_float3(a_2.x, a_2.z, a_2.w),
                                      make_float3(a_3.x, a_3.z, a_3.w),
                                      make_float3(a_4.x, a_4.z, a_4.w));
  A_rcc.M_3_1_rcc = calculate_rcc_3x3(make_float3(a_2.x, a_2.y, a_2.w),
                                      make_float3(a_3.x, a_3.y, a_3.w),
                                      make_float3(a_4.x, a_4.y, a_4.w));
  A_rcc.M_4_1_rcc = calculate_rcc_3x3(make_float3(a_2.x, a_2.y, a_2.z),
                                      make_float3(a_3.x, a_3.y, a_3.z),
                                      make_float3(a_4.x, a_4.y, a_4.z));

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_rcc.M_det - a_1.y * A_rcc.M_2_1_rcc.M_det +
                a_1.z * A_rcc.M_3_1_rcc.M_det - a_1.w * A_rcc.M_4_1_rcc.M_det;

  return A_rcc;
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 2x2 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float2 rc_linear_system_solve_non_singular_2x2(float2 a_1,
                                                          float2 a_2,
                                                          float2 b,
                                                          float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is Reduceder. */
  return make_float2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 3x3 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float3 rc_linear_system_solve_non_singular_3x3(
    float3 a_1, float3 a_2, float3 a_3, float3 b, rcc_3x3 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_det - b.y * A_rcc.M_2_1_det + b.z * A_rcc.M_3_1_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_det != 0.0f) {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.y, a_2.z),
        make_float2(a_3.y, a_3.z),
        make_float2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_1_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_rcc.M_2_1_det != 0.0f) {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.x, a_2.z),
        make_float2(a_3.x, a_3.z),
        make_float2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_2_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        make_float2(a_2.x, a_2.y),
        make_float2(a_3.x, a_3.y),
        make_float2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_rcc.M_3_1_det);
    return make_float3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 4x4 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
ccl_device float4 rc_linear_system_solve_non_singular_4x4(
    float4 a_1, float4 a_2, float4 a_3, float4 a_4, float4 b, rcc_4x4 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_rcc.M_det - b.y * A_rcc.M_2_1_rcc.M_det +
                      b.z * A_rcc.M_3_1_rcc.M_det - b.w * A_rcc.M_4_1_rcc.M_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.y, a_2.z, a_2.w),
        make_float3(a_3.y, a_3.z, a_3.w),
        make_float3(a_4.y, a_4.z, a_4.w),
        make_float3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_1_1_rcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_2_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.z, a_2.w),
        make_float3(a_3.x, a_3.z, a_3.w),
        make_float3(a_4.x, a_4.z, a_4.w),
        make_float3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_2_1_rcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_3_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.y, a_2.w),
        make_float3(a_3.x, a_3.y, a_3.w),
        make_float3(a_4.x, a_4.y, a_4.w),
        make_float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_3_1_rcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        make_float3(a_2.x, a_2.y, a_2.z),
        make_float3(a_3.x, a_3.y, a_3.z),
        make_float3(a_4.x, a_4.y, a_4.z),
        make_float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_4_1_rcc);
    return make_float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

ccl_device_noinline int svm_node_linear_system_solver(
    KernelGlobals kg, ccl_private ShaderData *sd, ccl_private float *stack, uint4 node, int offset)
{
  LinearSystemSolverStackOffsets so;

  svm_unpack_node_uchar4(
      node.y, &(so.b_vector), &(so.b_vector_w), &(so.column_1), &(so.column_1_w));
  svm_unpack_node_uchar4(
      node.z, &(so.column_2), &(so.column_2_w), &(so.column_3), &(so.column_3_w));
  svm_unpack_node_uchar4(
      node.w, &(so.column_4), &(so.column_4_w), &(so.solution), &(so.solution_w));

  uint4 defaults_1 = read_node(kg, &offset);
  float3 b_vector = stack_load_float3_default(
      stack, so.b_vector, make_uint3(defaults_1.x, defaults_1.y, defaults_1.z));
  float b_vector_w = stack_load_float_default(stack, so.b_vector_w, defaults_1.w);

  uint4 defaults_2 = read_node(kg, &offset);
  float3 column_1 = stack_load_float3_default(
      stack, so.column_1, make_uint3(defaults_2.x, defaults_2.y, defaults_2.z));
  float column_1_w = stack_load_float_default(stack, so.column_1_w, defaults_2.w);

  uint4 defaults_3 = read_node(kg, &offset);
  float3 column_2 = stack_load_float3_default(
      stack, so.column_2, make_uint3(defaults_3.x, defaults_3.y, defaults_3.z));
  float column_2_w = stack_load_float_default(stack, so.column_2_w, defaults_3.w);

  uint4 defaults_4 = read_node(kg, &offset);
  float3 column_3 = stack_load_float3_default(
      stack, so.column_3, make_uint3(defaults_4.x, defaults_4.y, defaults_4.z));
  float column_3_w = stack_load_float_default(stack, so.column_3_w, defaults_4.w);

  uint4 defaults_5 = read_node(kg, &offset);
  float3 column_4 = stack_load_float3_default(
      stack, so.column_4, make_uint3(defaults_5.x, defaults_5.y, defaults_5.z));
  float column_4_w = stack_load_float_default(stack, so.column_4_w, defaults_5.w);

  uint4 properties = read_node(kg, &offset);
  uint matrix_dimension = properties.x;

  switch (matrix_dimension) {
    case 1: {
      if (stack_valid(so.solution_w)) {
        if (column_1_w != 0.0f) {
          stack_store_float(stack, so.solution_w, b_vector_w / column_1_w);
        }
        else {
          stack_store_float(stack, so.solution_w, 0.0f);
        }
      }
      break;
    }
    case 2: {
      if (stack_valid(so.solution)) {
        float M_det = column_1.x * column_2.y - column_2.x * column_1.y;
        if (M_det != 0.0f) {
          float2 solution_xy = rc_linear_system_solve_non_singular_2x2(
              make_float2(column_1.x, column_1.y),
              make_float2(column_2.x, column_2.y),
              make_float2(b_vector.x, b_vector.y),
              M_det);
          stack_store_float3(stack, so.solution, make_float3(solution_xy.x, solution_xy.y, 0.0f));
        }
        else {
          stack_store_float3(stack, so.solution, make_float3(0.0f, 0.0f, 0.0f));
        }
      }
      break;
    }
    case 3: {
      if (stack_valid(so.solution)) {
        rcc_3x3 A_rcc = calculate_rcc_3x3(column_1, column_2, column_3);
        if (A_rcc.M_det != 0.0f) {
          stack_store_float3(stack,
                             so.solution,
                             rc_linear_system_solve_non_singular_3x3(
                                 column_1, column_2, column_3, b_vector, A_rcc));
        }
        else {
          stack_store_float3(stack, so.solution, make_float3(0.0f, 0.0f, 0.0f));
        }
      }
      break;
    }
    case 4: {
      if (stack_valid(so.solution) || stack_valid(so.solution_w)) {
        rcc_4x4 A_rcc = calculate_rcc_4x4(
            make_float4(column_1.x, column_1.y, column_1.z, column_1_w),
            make_float4(column_2.x, column_2.y, column_2.z, column_2_w),
            make_float4(column_3.x, column_3.y, column_3.z, column_3_w),
            make_float4(column_4.x, column_4.y, column_4.z, column_4_w));
        if (A_rcc.M_det != 0.0f) {
          float4 solution_xyzw = rc_linear_system_solve_non_singular_4x4(
              make_float4(column_1.x, column_1.y, column_1.z, column_1_w),
              make_float4(column_2.x, column_2.y, column_2.z, column_2_w),
              make_float4(column_3.x, column_3.y, column_3.z, column_3_w),
              make_float4(column_4.x, column_4.y, column_4.z, column_4_w),
              make_float4(b_vector.x, b_vector.y, b_vector.z, b_vector_w),
              A_rcc);
          if (stack_valid(so.solution)) {
            stack_store_float3(stack,
                               so.solution,
                               make_float3(solution_xyzw.x, solution_xyzw.y, solution_xyzw.z));
          }
          if (stack_valid(so.solution_w)) {
            if (stack_valid(so.solution_w)) {
              stack_store_float(stack, so.solution_w, solution_xyzw.w);
            }
          }
          else {
            if (stack_valid(so.solution)) {
              stack_store_float3(stack, so.solution, make_float3(0.0f, 0.0f, 0.0f));
            }
            if (stack_valid(so.solution_w)) {
              stack_store_float(stack, so.solution_w, 0.0f);
            }
          }
        }
        break;
      }
    }
  }

  return offset;
}

CCL_NAMESPACE_END
