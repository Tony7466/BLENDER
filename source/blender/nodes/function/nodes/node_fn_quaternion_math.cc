/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_listbase.h"
#include "BLI_math_quaternion.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "BLI_math_rotation.h"

#include "RNA_enum_types.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket_search_link.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_quaternion_math_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Rotation>("Rotation", "Rotation");
  b.add_input<decl::Rotation>("Rotation", "Rotation_001");

  b.add_output<decl::Rotation>("Rotation");
  b.add_output<decl::Float>("Value");
}

static void node_init(bNodeTree* /*ntree*/, bNode* node)
{
  node->custom1 = NODE_QUATERNION_MATH_MULTIPLY;
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", 0, "", ICON_NONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *socketInputRotationB = (bNodeSocket *)BLI_findlink(&node->inputs, 1);
  bNodeSocket* socketOutputRotation = (bNodeSocket*)BLI_findlink(&node->outputs, 0);
  bNodeSocket *socketOutputValue = (bNodeSocket *)BLI_findlink(&node->outputs, 1);

  bke::nodeSetSocketAvailability(ntree, socketInputRotationB, ELEM(node->custom1,
    NODE_QUATERNION_MATH_DOT_PRODUCT,
    NODE_QUATERNION_MATH_MULTIPLY,
    NODE_QUATERNION_MATH_ROTATION_DIFFERENCE
  ));
  bke::nodeSetSocketAvailability(ntree, socketOutputRotation, ELEM(node->custom1,
    NODE_QUATERNION_MATH_INVERT,
    NODE_QUATERNION_MATH_MULTIPLY,
    NODE_QUATERNION_MATH_NEGATE,
    NODE_QUATERNION_MATH_ROTATION_DIFFERENCE
  ));
  bke::nodeSetSocketAvailability(ntree, socketOutputValue, ELEM(node->custom1,
    NODE_QUATERNION_MATH_DOT_PRODUCT
  ));
}

static void node_label(const bNodeTree * /*tree*/,
                       const bNode *node,
                       char *label,
                       int label_maxncpy)
{
  const char *name;
  bool enum_label = RNA_enum_name(rna_enum_node_quaternion_math_items, node->custom1, &name);
  if (!enum_label) {
    name = "Unknown";
  }
  BLI_strncpy_utf8(label, IFACE_(name), label_maxncpy);
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (!params.node_tree().typeinfo->validate_link(static_cast<eNodeSocketDatatype>(params.other_socket().type), SOCK_ROTATION))
  {
    return;
  }

  for (const EnumPropertyItem *item = rna_enum_node_quaternion_math_items;item->identifier != nullptr; item++)
  {
    if (item->name != nullptr && item->identifier[0] != '\0') {
      NodeQuaternionMathOperation operation = static_cast<NodeQuaternionMathOperation>(item->value);
      params.add_item(IFACE_(item->name), [operation](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeQuaternionMath");
        node.custom1 = operation;
        params.update_and_connect_available_socket(node, "Rotation");
      });
    }
  }
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();
  static auto dot_product_fn = mf::build::SI2_SO<math::Quaternion, math::Quaternion, float>(
      "Dot Product", [](math::Quaternion a, math::Quaternion b) { return math::dot(a, b); }, exec_preset);
  static auto invert_fn = mf::build::SI1_SO<math::Quaternion, math::Quaternion>(
      "Invert", [](math::Quaternion a) { return math::invert_normalized(a); }, exec_preset);
  static auto multiply_fn = mf::build::SI2_SO<math::Quaternion, math::Quaternion, math::Quaternion>(
      "Multiply", [](math::Quaternion a, math::Quaternion b) { return a * b; }, exec_preset);
  static auto negate_fn = mf::build::SI1_SO<math::Quaternion, math::Quaternion>(
      "Negate", [](math::Quaternion a) { return -a; }, exec_preset);
  static auto rotation_difference_fn = mf::build::SI2_SO<math::Quaternion, math::Quaternion, math::Quaternion>(
      "Rotation Difference", [](math::Quaternion a, math::Quaternion b) {
      float q[4];
      rotation_between_quats_to_quat(q, static_cast<VecBase<float, 4>>(a), static_cast<VecBase<float, 4>>(b));
      return math::Quaternion(q);
    }, exec_preset);

  switch (bnode.custom1) {
    case NODE_QUATERNION_MATH_DOT_PRODUCT:
      return &dot_product_fn;
    case NODE_QUATERNION_MATH_INVERT:
      return &invert_fn;
    case NODE_QUATERNION_MATH_MULTIPLY:
      return &multiply_fn;
    case NODE_QUATERNION_MATH_NEGATE:
      return &negate_fn;
    case NODE_QUATERNION_MATH_ROTATION_DIFFERENCE:
      return &rotation_difference_fn;
  }

  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

} // namespace blender::nodes::node_fn_quaternion_math_cc

void register_node_type_fn_quaternion_math()
{
  namespace file_ns = blender::nodes::node_fn_quaternion_math_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_QUATERNION_MATH, "Quaternion Math", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.labelfunc = file_ns::node_label;
  ntype.updatefunc = file_ns::node_update;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  ntype.initfunc = file_ns::node_init;
  nodeRegisterType(&ntype);
}
