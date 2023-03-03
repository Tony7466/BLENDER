/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BLI_math_matrix.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_matrix_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix>(N_("Matrix"));
  b.add_input<decl::Vector>(N_("Vector"));
  b.add_output<decl::Vector>(N_("Vector"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "vector_mode", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_MATRIX_TRANSFORM_POINT;
}

static const mf::MultiFunction *get_multi_function(NodeMatrixTransformVectorMode vector_mode)
{
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();

  switch (vector_mode) {
    case NODE_MATRIX_TRANSFORM_POINT: {
      static auto fn = mf::build::SI2_SO<float4x4, float3, float3>(
          "transform_point",
          [](const float4x4 &m, const float3 &v) -> float3 { return math::transform_point(m, v); },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_TRANSFORM_DIRECTION: {
      static auto fn = mf::build::SI2_SO<float4x4, float3, float3>(
          "transform_direction",
          [](const float4x4 &m, const float3 &v) -> float3 { return math::transform_direction(m, v); },
          exec_preset);
      return &fn;
    }
    case NODE_MATRIX_TRANSFORM_NORMAL: {
      static auto fn = mf::build::SI2_SO<float4x4, float3, float3>(
          "transform_normal",
          [](const float4x4 &m, const float3 &v) -> float3 {
            const float3x3 transpose_inverse = math::transpose(
                math::invert(float3x3(m.view<3, 3>())));
            return math::transform_direction(transpose_inverse, v);
          },
          exec_preset);
      return &fn;
    }
  }

  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeMatrixTransformVectorMode vector_mode = (NodeMatrixTransformVectorMode)builder.node().custom1;
  const mf::MultiFunction *fn = get_multi_function(vector_mode);
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_matrix_transform_cc

void register_node_type_fn_matrix_transform(void)
{
  namespace file_ns = blender::nodes::node_fn_matrix_transform_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_MATRIX_TRANSFORM, "Matrix Transform", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
