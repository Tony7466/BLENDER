/* SPDX-FileCopyrightText: 2024 Tenkai Raiko
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "node_shader_util.hh"
#include "node_util.hh"

#include "BKE_texture.h"

#include "BLI_noise.hh"

#include "NOD_multi_function.hh"

#include "RNA_access.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_shader_linear_system_solver_cc {

NODE_STORAGE_FUNCS(NodeLinearSystemSolver)

static void sh_node_linear_system_solver_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.use_custom_socket_order();

  b.add_output<decl::Vector>("Solution").no_muted_links().make_available([](bNode &node) {
    node_storage(node).matrix_dimension = 2;
  });
  b.add_output<decl::Float>("Solution W").no_muted_links().make_available([](bNode &node) {
    node_storage(node).matrix_dimension = 1;
  });

  PanelDeclarationBuilder &b_vector = b.add_panel("b Vector").default_closed(false);
  b_vector.add_input<decl::Vector>("b Vector")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 2; })
      .description("b Vector in Ax=b");
  b_vector.add_input<decl::Float>("b Vector W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 1; })
      .description("b Vector in Ax=b");

  PanelDeclarationBuilder &a_matrix = b.add_panel("A Matrix").default_closed(false);
  a_matrix.add_input<decl::Vector>("Column 1")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{1.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 2; })
      .description("1st column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Float>("Column 1 W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 1; })
      .description("1st column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Vector>("Column 2")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 1.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 2; })
      .description("2nd column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Float>("Column 2 W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 4; })
      .description("2nd column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Vector>("Column 3")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 1.0f})
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 3; })
      .description("3rd column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Float>("Column 3 W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(0.0f)
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 4; })
      .description("3rd column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Vector>("Column 4")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(float3{0.0f, 0.0f, 0.0f})
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 4; })
      .description("4th column vector of the A matrix in Ax=b");
  a_matrix.add_input<decl::Float>("Column 4 W")
      .min(-1000.0f)
      .max(1000.0f)
      .default_value(1.0f)
      .make_available([](bNode &node) { node_storage(node).matrix_dimension = 4; })
      .description("4th column vector of the A matrix in Ax=b");
}

static void node_shader_buts_linear_system_solver(uiLayout *layout,
                                                  bContext * /*C*/,
                                                  PointerRNA *ptr)
{
  uiItemR(layout, ptr, "matrix_dimension", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_shader_init_linear_system_solver(bNodeTree * /*ntree*/, bNode *node)
{
  NodeLinearSystemSolver *data = MEM_cnew<NodeLinearSystemSolver>(__func__);
  data->matrix_dimension = 3;

  node->storage = data;
}

static const char *gpu_shader_get_name()
{
  return "node_linear_system_solver";
}

static int node_shader_gpu_linear_system_solver(GPUMaterial *mat,
                                                bNode *node,
                                                bNodeExecData * /*execdata*/,
                                                GPUNodeStack *in,
                                                GPUNodeStack *out)
{
  const NodeLinearSystemSolver &storage = node_storage(*node);
  float matrix_dimension = storage.matrix_dimension;

  const char *name = gpu_shader_get_name();

  return GPU_stack_link(mat, node, name, in, out, GPU_constant(&matrix_dimension));
}

static void node_shader_update_linear_system_solver(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *inbVectorSock = bke::node_find_socket(node, SOCK_IN, "b Vector");
  bNodeSocket *inbVectorWSock = bke::node_find_socket(node, SOCK_IN, "b Vector W");
  bNodeSocket *inColumn1Sock = bke::node_find_socket(node, SOCK_IN, "Column 1");
  bNodeSocket *inColumn1WSock = bke::node_find_socket(node, SOCK_IN, "Column 1 W");
  bNodeSocket *inColumn2Sock = bke::node_find_socket(node, SOCK_IN, "Column 2");
  bNodeSocket *inColumn2WSock = bke::node_find_socket(node, SOCK_IN, "Column 2 W");
  bNodeSocket *inColumn3Sock = bke::node_find_socket(node, SOCK_IN, "Column 3");
  bNodeSocket *inColumn3WSock = bke::node_find_socket(node, SOCK_IN, "Column 3 W");
  bNodeSocket *inColumn4Sock = bke::node_find_socket(node, SOCK_IN, "Column 4");
  bNodeSocket *inColumn4WSock = bke::node_find_socket(node, SOCK_IN, "Column 4 W");

  bNodeSocket *outSolutionSock = bke::node_find_socket(node, SOCK_OUT, "Solution");
  bNodeSocket *outSolutionWSock = bke::node_find_socket(node, SOCK_OUT, "Solution W");

  const NodeLinearSystemSolver &storage = node_storage(*node);

  bke::node_set_socket_availability(ntree, inbVectorSock, storage.matrix_dimension >= 2);
  bke::node_set_socket_availability(
      ntree, inbVectorWSock, (storage.matrix_dimension == 1) || (storage.matrix_dimension == 4));
  if (storage.matrix_dimension == 1) {
    node_sock_label(inbVectorWSock, "b Vector");
  }
  else {
    node_sock_label_clear(inbVectorWSock);
  }
  bke::node_set_socket_availability(ntree, inColumn1Sock, storage.matrix_dimension >= 2);
  bke::node_set_socket_availability(
      ntree, inColumn1WSock, (storage.matrix_dimension == 1) || (storage.matrix_dimension == 4));
  if (storage.matrix_dimension == 1) {
    node_sock_label(inColumn1WSock, "Column 1");
  }
  else {
    node_sock_label_clear(inColumn1WSock);
  }
  bke::node_set_socket_availability(ntree, inColumn2Sock, storage.matrix_dimension >= 2);
  bke::node_set_socket_availability(ntree, inColumn2WSock, storage.matrix_dimension == 4);
  bke::node_set_socket_availability(ntree, inColumn3Sock, storage.matrix_dimension >= 3);
  bke::node_set_socket_availability(ntree, inColumn3WSock, storage.matrix_dimension == 4);
  bke::node_set_socket_availability(ntree, inColumn4Sock, storage.matrix_dimension == 4);
  bke::node_set_socket_availability(ntree, inColumn4WSock, storage.matrix_dimension == 4);

  bke::node_set_socket_availability(ntree, outSolutionSock, storage.matrix_dimension >= 2);
  bke::node_set_socket_availability(
      ntree, outSolutionWSock, (storage.matrix_dimension == 1) || (storage.matrix_dimension == 4));
  if (storage.matrix_dimension == 1) {
    node_sock_label(outSolutionWSock, "Solution");
  }
  else {
    node_sock_label_clear(outSolutionWSock);
  }
}

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

rcc_3x3 calculate_rcc_3x3(float3 a_1, float3 a_2, float3 a_3)
{
  rcc_3x3 A_rcc;

  A_rcc.M_1_1_det = a_2.y * a_3.z - a_3.y * a_2.z;
  A_rcc.M_2_1_det = a_2.x * a_3.z - a_3.x * a_2.z;
  A_rcc.M_3_1_det = a_2.x * a_3.y - a_3.x * a_2.y;

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_det - a_1.y * A_rcc.M_2_1_det + a_1.z * A_rcc.M_3_1_det;

  return A_rcc;
}

rcc_4x4 calculate_rcc_4x4(float4 a_1, float4 a_2, float4 a_3, float4 a_4)
{
  rcc_4x4 A_rcc;

  A_rcc.M_1_1_rcc = calculate_rcc_3x3(
      float3(a_2.y, a_2.z, a_2.w), float3(a_3.y, a_3.z, a_3.w), float3(a_4.y, a_4.z, a_4.w));
  A_rcc.M_2_1_rcc = calculate_rcc_3x3(
      float3(a_2.x, a_2.z, a_2.w), float3(a_3.x, a_3.z, a_3.w), float3(a_4.x, a_4.z, a_4.w));
  A_rcc.M_3_1_rcc = calculate_rcc_3x3(
      float3(a_2.x, a_2.y, a_2.w), float3(a_3.x, a_3.y, a_3.w), float3(a_4.x, a_4.y, a_4.w));
  A_rcc.M_4_1_rcc = calculate_rcc_3x3(
      float3(a_2.x, a_2.y, a_2.z), float3(a_3.x, a_3.y, a_3.z), float3(a_4.x, a_4.y, a_4.z));

  A_rcc.M_det = a_1.x * A_rcc.M_1_1_rcc.M_det - a_1.y * A_rcc.M_2_1_rcc.M_det +
                a_1.z * A_rcc.M_3_1_rcc.M_det - a_1.w * A_rcc.M_4_1_rcc.M_det;

  return A_rcc;
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 2x2 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
float2 rc_linear_system_solve_non_singular_2x2(float2 a_1, float2 a_2, float2 b, float M_det)
{
  /* Use Cramer's rule on both components instead of further recursion because it is faster. */
  return float2((b.x * a_2.y - a_2.x * b.y) / M_det, (a_1.x * b.y - b.x * a_1.y) / M_det);
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 3x3 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
float3 rc_linear_system_solve_non_singular_3x3(
    float3 a_1, float3 a_2, float3 a_3, float3 b, rcc_3x3 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_det - b.y * A_rcc.M_2_1_det + b.z * A_rcc.M_3_1_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_det != 0.0f) {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        float2(a_2.y, a_2.z),
        float2(a_3.y, a_3.z),
        float2(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_1_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else if (A_rcc.M_2_1_det != 0.0f) {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        float2(a_2.x, a_2.z),
        float2(a_3.x, a_3.z),
        float2(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_2_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
  else {
    float2 solution_yz = rc_linear_system_solve_non_singular_2x2(
        float2(a_2.x, a_2.y),
        float2(a_3.x, a_3.y),
        float2(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x),
        A_rcc.M_3_1_det);
    return float3(solution_x, solution_yz.x, solution_yz.y);
  }
}

/* Solves Ax=b for x using the Reduced Cramer algorithm, with a_n being the nth coloumn vector of
 * the invertible 4x4 matrix A. rc_linear_system_solve_non_singular_4x4 doesn't check whether or
 * not A is invertible. Calling it on a singular matrix leads to division by 0. */
float4 rc_linear_system_solve_non_singular_4x4(
    float4 a_1, float4 a_2, float4 a_3, float4 a_4, float4 b, rcc_4x4 A_rcc)
{
  float solution_x = (b.x * A_rcc.M_1_1_rcc.M_det - b.y * A_rcc.M_2_1_rcc.M_det +
                      b.z * A_rcc.M_3_1_rcc.M_det - b.w * A_rcc.M_4_1_rcc.M_det) /
                     A_rcc.M_det;

  if (A_rcc.M_1_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        float3(a_2.y, a_2.z, a_2.w),
        float3(a_3.y, a_3.z, a_3.w),
        float3(a_4.y, a_4.z, a_4.w),
        float3(b.y - a_1.y * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_1_1_rcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_2_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.z, a_2.w),
        float3(a_3.x, a_3.z, a_3.w),
        float3(a_4.x, a_4.z, a_4.w),
        float3(b.x - a_1.x * solution_x, b.z - a_1.z * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_2_1_rcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else if (A_rcc.M_3_1_rcc.M_det != 0.0f) {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.y, a_2.w),
        float3(a_3.x, a_3.y, a_3.w),
        float3(a_4.x, a_4.y, a_4.w),
        float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.w - a_1.w * solution_x),
        A_rcc.M_3_1_rcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
  else {
    float3 solution_yzw = rc_linear_system_solve_non_singular_3x3(
        float3(a_2.x, a_2.y, a_2.z),
        float3(a_3.x, a_3.y, a_3.z),
        float3(a_4.x, a_4.y, a_4.z),
        float3(b.x - a_1.x * solution_x, b.y - a_1.y * solution_x, b.z - a_1.z * solution_x),
        A_rcc.M_4_1_rcc);
    return float4(solution_x, solution_yzw.x, solution_yzw.y, solution_yzw.z);
  }
}

class LinearSystemSolverFunction : public mf::MultiFunction {
 private:
  int matrix_dimension_;

  mf::Signature signature_;

 public:
  LinearSystemSolverFunction(int matrix_dimension) : matrix_dimension_(matrix_dimension)
  {
    BLI_assert((matrix_dimension >= 1) && (matrix_dimension <= 4));

    signature_ = create_signature(matrix_dimension);
    this->set_signature(&signature_);
  }

  static mf::Signature create_signature(int matrix_dimension)
  {
    mf::Signature signature;
    mf::SignatureBuilder builder{"linear_system_solver", signature};

    if (matrix_dimension >= 2) {
      builder.single_input<float3>("b Vector");
    }
    if ((matrix_dimension == 1) || (matrix_dimension == 4)) {
      builder.single_input<float>("b Vector W");
    }
    if (matrix_dimension >= 2) {
      builder.single_input<float3>("Column 1");
    }
    if ((matrix_dimension == 1) || (matrix_dimension == 4)) {
      builder.single_input<float>("Column 1 W");
    }
    if (matrix_dimension >= 2) {
      builder.single_input<float3>("Column 2");
    }
    if (matrix_dimension == 4) {
      builder.single_input<float>("Column 2 W");
    }
    if (matrix_dimension >= 3) {
      builder.single_input<float3>("Column 3");
    }
    if (matrix_dimension == 4) {
      builder.single_input<float>("Column 3 W");
      builder.single_input<float3>("Column 4");
      builder.single_input<float>("Column 4 W");
    }

    if (matrix_dimension >= 2) {
      builder.single_output<float3>("Solution", mf::ParamFlag::SupportsUnusedOutput);
    }
    if ((matrix_dimension == 1) || (matrix_dimension == 4)) {
      builder.single_output<float>("Solution W", mf::ParamFlag::SupportsUnusedOutput);
    }

    return signature;
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    int param = 0;

    const VArray<float3> &unused_float3 = VArray<float3>{};
    const VArray<float> &unused_float = VArray<float>{};

    const VArray<float3> &b_vector = ELEM(matrix_dimension_, 2, 3, 4) ?
                                         params.readonly_single_input<float3>(param++,
                                                                              "b Vector") :
                                         unused_float3;
    const VArray<float> &b_vector_w = ELEM(matrix_dimension_, 1, 4) ?
                                          params.readonly_single_input<float>(param++,
                                                                              "b Vector W") :
                                          unused_float;
    const VArray<float3> &column_1 = ELEM(matrix_dimension_, 2, 3, 4) ?
                                         params.readonly_single_input<float3>(param++,
                                                                              "Column 1") :
                                         unused_float3;
    const VArray<float> &column_1_w = ELEM(matrix_dimension_, 1, 4) ?
                                          params.readonly_single_input<float>(param++,
                                                                              "Column 1 W") :
                                          unused_float;
    const VArray<float3> &column_2 = ELEM(matrix_dimension_, 2, 3, 4) ?
                                         params.readonly_single_input<float3>(param++,
                                                                              "Column 2") :
                                         unused_float3;
    const VArray<float> &column_2_w = ELEM(matrix_dimension_, 4) ?
                                          params.readonly_single_input<float>(param++,
                                                                              "Column 2 W") :
                                          unused_float;
    const VArray<float3> &column_3 = ELEM(matrix_dimension_, 3, 4) ?
                                         params.readonly_single_input<float3>(param++,
                                                                              "Column 3") :
                                         unused_float3;
    const VArray<float> &column_3_w = ELEM(matrix_dimension_, 4) ?
                                          params.readonly_single_input<float>(param++,
                                                                              "Column 3 W") :
                                          unused_float;
    const VArray<float3> &column_4 = ELEM(matrix_dimension_, 4) ?
                                         params.readonly_single_input<float3>(param++,
                                                                              "Column 4") :
                                         unused_float3;
    const VArray<float> &column_4_w = ELEM(matrix_dimension_, 4) ?
                                          params.readonly_single_input<float>(param++,
                                                                              "Column 4 W") :
                                          unused_float;

    MutableSpan<float3> r_solution;
    MutableSpan<float> r_solution_w;

    if (ELEM(matrix_dimension_, 2, 3, 4)) {
      r_solution = params.uninitialized_single_output_if_required<float3>(param++, "Solution");
    }
    if (ELEM(matrix_dimension_, 1, 4)) {
      r_solution_w = params.uninitialized_single_output_if_required<float>(param++, "Solution W");
    }

    const bool calc_solution = !r_solution.is_empty();
    const bool calc_solution_w = !r_solution_w.is_empty();

    mask.foreach_index([&](const int64_t i) {
      switch (matrix_dimension_) {
        case 1: {
          if (calc_solution_w) {
            if (column_1_w[i] != 0.0f) {
              r_solution_w[i] = b_vector_w[i] / column_1_w[i];
            }
            else {
              r_solution_w[i] = 0.0f;
            }
          }
          break;
        }
        case 2: {
          if (calc_solution) {
            float M_det = column_1[i].x * column_2[i].y - column_2[i].x * column_1[i].y;
            if (M_det != 0.0f) {
              float2 solution_xy = rc_linear_system_solve_non_singular_2x2(
                  float2(column_1[i].x, column_1[i].y),
                  float2(column_2[i].x, column_2[i].y),
                  float2(b_vector[i].x, b_vector[i].y),
                  M_det);
              r_solution[i] = float3(solution_xy.x, solution_xy.y, 0.0f);
            }
            else {
              r_solution[i] = float3(0.0f, 0.0f, 0.0f);
            }
          }
          break;
        }
        case 3: {
          if (calc_solution) {
            rcc_3x3 A_rcc = calculate_rcc_3x3(column_1[i], column_2[i], column_3[i]);
            if (A_rcc.M_det != 0.0f) {
              r_solution[i] = rc_linear_system_solve_non_singular_3x3(
                  column_1[i], column_2[i], column_3[i], b_vector[i], A_rcc);
            }
            else {
              r_solution[i] = float3(0.0f, 0.0f, 0.0f);
            }
          }
          break;
        }
        case 4: {
          if (calc_solution || calc_solution_w) {
            rcc_4x4 A_rcc = calculate_rcc_4x4(
                float4(column_1[i].x, column_1[i].y, column_1[i].z, column_1_w[i]),
                float4(column_2[i].x, column_2[i].y, column_2[i].z, column_2_w[i]),
                float4(column_3[i].x, column_3[i].y, column_3[i].z, column_3_w[i]),
                float4(column_4[i].x, column_4[i].y, column_4[i].z, column_4_w[i]));
            if (A_rcc.M_det != 0.0f) {
              float4 solution_xyzw = rc_linear_system_solve_non_singular_4x4(
                  float4(column_1[i].x, column_1[i].y, column_1[i].z, column_1_w[i]),
                  float4(column_2[i].x, column_2[i].y, column_2[i].z, column_2_w[i]),
                  float4(column_3[i].x, column_3[i].y, column_3[i].z, column_3_w[i]),
                  float4(column_4[i].x, column_4[i].y, column_4[i].z, column_4_w[i]),
                  float4(b_vector[i].x, b_vector[i].y, b_vector[i].z, b_vector_w[i]),
                  A_rcc);
              if (calc_solution) {
                r_solution[i] = float3(solution_xyzw.x, solution_xyzw.y, solution_xyzw.z);
              }
              if (calc_solution_w) {
                r_solution_w[i] = solution_xyzw.w;
              }
            }
            else {
              if (calc_solution) {
                r_solution[i] = float3(0.0f, 0.0f, 0.0f);
              }
              if (calc_solution_w) {
                r_solution_w[i] = 0.0f;
              }
            }
          }
          break;
        }
      }
    });
  }

  ExecutionHints get_execution_hints() const override
  {
    ExecutionHints hints;
    hints.allocates_array = false;
    hints.min_grain_size = 50;
    return hints;
  }
};

static void sh_node_linear_system_solver_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeLinearSystemSolver &storage = node_storage(builder.node());
  builder.construct_and_set_matching_fn<LinearSystemSolverFunction>(storage.matrix_dimension);
}

}  // namespace blender::nodes::node_shader_linear_system_solver_cc

void register_node_type_sh_linear_system_solver()
{
  namespace file_ns = blender::nodes::node_shader_linear_system_solver_cc;

  static blender::bke::bNodeType ntype;

  sh_fn_node_type_base(
      &ntype, SH_NODE_LINEAR_SYSTEM_SOLVER, "Linear System Solver", NODE_CLASS_TEXTURE);
  ntype.declare = file_ns::sh_node_linear_system_solver_declare;
  ntype.draw_buttons = file_ns::node_shader_buts_linear_system_solver;
  ntype.initfunc = file_ns::node_shader_init_linear_system_solver;
  blender::bke::node_type_storage(
      &ntype, "NodeLinearSystemSolver", node_free_standard_storage, node_copy_standard_storage);
  ntype.gpu_fn = file_ns::node_shader_gpu_linear_system_solver;
  ntype.updatefunc = file_ns::node_shader_update_linear_system_solver;
  ntype.build_multi_function = file_ns::sh_node_linear_system_solver_build_multi_function;

  blender::bke::node_register_type(&ntype);
}
