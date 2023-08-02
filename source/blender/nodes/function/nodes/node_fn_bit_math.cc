/* SPDX-FileCopyrightText: 2024 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <numeric>

#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "node_function_util.hh"

namespace blender::nodes::node_fn_bit_math_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Int>("Value");
  b.add_input<decl::Int>("Value", "Value_001");
  b.add_output<decl::Int>("Value");
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const bool one_input_ops = ELEM(node->custom1, NODE_BIT_MATH_NOT);

  bNodeSocket *sockA = static_cast<bNodeSocket *>(node->inputs.first);
  bNodeSocket *sockB = sockA->next;

  bke::node_set_socket_availability(ntree, sockB, !one_input_ops);
}

class SocketSearchOp {
 public:
  std::string socket_name;
  NodeBitMathOperation operation;
  void operator()(LinkSearchOpParams &params)
  {
    bNode &node = params.add_node("FunctionNodeBitMath");
    node.custom1 = NodeBitMathOperation(operation);
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (!params.node_tree().typeinfo->validate_link(eNodeSocketDatatype(params.other_socket().type),
                                                  SOCK_INT))
  {
    return;
  }

  const bool is_integer = params.other_socket().type == SOCK_INT;
  const int weight = is_integer ? 0 : -1;

  /* Add socket A operations. */
  for (const EnumPropertyItem *item = rna_enum_node_bit_math_items; item->identifier != nullptr;
       item++)
  {
    if (item->name != nullptr && item->identifier[0] != '\0') {
      params.add_item(
          IFACE_(item->name), SocketSearchOp{"Value", NodeBitMathOperation(item->value)}, weight);
    }
  }
}

static void node_label(const bNodeTree * /*ntree*/, const bNode *node, char *label, int maxlen)
{
  const char *name;
  bool enum_label = RNA_enum_name(rna_enum_node_bit_math_items, node->custom1, &name);
  if (!enum_label) {
    name = "Unknown";
  }
  BLI_strncpy(label, IFACE_(name), maxlen);
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  NodeBitMathOperation operation = NodeBitMathOperation(bnode.custom1);
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();
  static auto and_fn = mf::build::SI2_SO<int, int, int>(
      "And", [](int a, int b) { return a & b; }, exec_preset);
  static auto or_fn = mf::build::SI2_SO<int, int, int>(
      "Or", [](int a, int b) { return a | b; }, exec_preset);
  static auto xor_fn = mf::build::SI2_SO<int, int, int>(
      "Xor", [](int a, int b) { return a ^ b; }, exec_preset);
  static auto not_fn = mf::build::SI1_SO<int, int>(
      "Not", [](int a) { return ~a; }, exec_preset);
  static auto and_not_fn = mf::build::SI2_SO<int, int, int>(
      "And Not", [](int a, int b) { return a & ~b; }, exec_preset);
  static auto left_shift_fn = mf::build::SI2_SO<int, int, int>(
      "Left Shift", [](int a, int b) { return a << b; }, exec_preset);
  static auto right_shift_fn = mf::build::SI2_SO<int, int, int>(
      "Right Shift", [](int a, int b) { return a >> b; }, exec_preset);

  switch (operation) {
    case NODE_BIT_MATH_AND:
      return &and_fn;
    case NODE_BIT_MATH_OR:
      return &or_fn;
    case NODE_BIT_MATH_XOR:
      return &xor_fn;
    case NODE_BIT_MATH_NOT:
      return &not_fn;
    case NODE_BIT_MATH_AND_NOT:
      return &and_not_fn;
    case NODE_BIT_MATH_LEFT_SHIFT:
      return &left_shift_fn;
    case NODE_BIT_MATH_RIGHT_SHIFT:
      return &right_shift_fn;
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

static void node_rna(StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_node_enum(srna,
                           "operation",
                           "Operation",
                           "",
                           rna_enum_node_bit_math_items,
                           NOD_inline_enum_accessors(custom1),
                           NODE_BIT_MATH_AND);
  RNA_def_property_update_runtime(prop, rna_Node_socket_update);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_BIT_MATH, "Bit Math", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.labelfunc = node_label;
  ntype.updatefunc = node_update;
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;

  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_bit_math_cc
