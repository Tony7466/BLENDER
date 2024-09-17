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

namespace blender::nodes::node_fn_integer_math_cc {

NODE_STORAGE_FUNCS(NodeFunctionIntegerMath)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Int>("A");
  b.add_input<decl::Int>("B");
  b.add_input<decl::Int>("C");
  b.add_output<decl::Int>("Value");
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeFunctionIntegerMath &storage = node_storage(*node);

  const bool one_input_ops = ELEM(storage.operation,
                                  NODE_INTEGER_MATH_ABSOLUTE,
                                  NODE_INTEGER_MATH_SIGN,
                                  NODE_INTEGER_MATH_NOT,
                                  NODE_INTEGER_MATH_SQUARE,
                                  NODE_INTEGER_MATH_CUBE,
                                  NODE_INTEGER_MATH_NEGATE);
  const bool three_input_ops = ELEM(storage.operation,
                                    NODE_INTEGER_MATH_MULTIPLY_ADD,
                                    NODE_INTEGER_MATH_CLAMP,
                                    NODE_INTEGER_MATH_CLAMP_RANGE);

  bNodeSocket *sockA = (bNodeSocket *)BLI_findlink(&node->inputs, 0);

  bNodeSocket *sockB = (bNodeSocket *)BLI_findlink(&node->inputs, 1);
  bke::node_set_socket_availability(ntree, sockB, !one_input_ops);

  bNodeSocket *sockC = (bNodeSocket *)BLI_findlink(&node->inputs, 2);
  bke::node_set_socket_availability(ntree, sockC, three_input_ops);

  node_sock_label_clear(sockA);
  node_sock_label_clear(sockB);
  node_sock_label_clear(sockC);
  switch (storage.operation) {
    case NODE_INTEGER_MATH_CLAMP:
    case NODE_INTEGER_MATH_CLAMP_RANGE:
      node_sock_label(sockA, N_("Value"));
      node_sock_label(sockB, N_("Min"));
      node_sock_label(sockC, N_("Max"));
      break;
    case NODE_INTEGER_MATH_MULTIPLY_ADD:
      node_sock_label(sockA, N_("Value"));
      node_sock_label(sockB, N_("Multiplier"));
      node_sock_label(sockC, N_("Addend"));
      break;
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeFunctionIntegerMath *data = MEM_cnew<NodeFunctionIntegerMath>(__func__);
  data->operation = NODE_INTEGER_MATH_ADD;
  node->storage = data;
}

class SocketSearchOp {
 public:
  std::string socket_name;
  NodeIntegerMathOperation operation;
  void operator()(LinkSearchOpParams &params)
  {
    bNode &node = params.add_node("FunctionNodeIntegerMath");
    node_storage(node).operation = (NodeIntegerMathOperation)operation;
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  if (!params.node_tree().typeinfo->validate_link(
          static_cast<eNodeSocketDatatype>(params.other_socket().type), SOCK_INT))
  {
    return;
  }

  const int weight = ELEM(params.other_socket().type, SOCK_INT, SOCK_BOOLEAN) ? 0 : -1;

  /* Add socket A operations. */
  for (const EnumPropertyItem *item = rna_enum_node_integer_math_items;
       item->identifier != nullptr;
       item++)
  {
    if (item->name != nullptr && item->identifier[0] != '\0') {
      params.add_item(IFACE_(item->name),
                      SocketSearchOp{"Value", (NodeIntegerMathOperation)item->value},
                      weight);
    }
  }
}

static void node_label(const bNodeTree * /*ntree*/, const bNode *node, char *label, int maxlen)
{
  const NodeFunctionIntegerMath &storage = node_storage(*node);
  const char *name;
  bool enum_label = RNA_enum_name(rna_enum_node_integer_math_items, storage.operation, &name);
  if (!enum_label) {
    name = "Unknown";
  }
  BLI_strncpy(label, IFACE_(name), maxlen);
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const NodeFunctionIntegerMath &storage = node_storage(bnode);
  NodeIntegerMathOperation operation = (NodeIntegerMathOperation)storage.operation;
  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();
  static auto add_fn = mf::build::SI2_SO<int, int, int>(
      "Add", [](int a, int b) { return a + b; }, exec_preset);
  static auto sub_fn = mf::build::SI2_SO<int, int, int>(
      "Subtract", [](int a, int b) { return a - b; }, exec_preset);
  static auto multiply_fn = mf::build::SI2_SO<int, int, int>(
      "Multiply", [](int a, int b) { return a * b; }, exec_preset);
  static auto divide_fn = mf::build::SI2_SO<int, int, int>(
      "Divide", [](int a, int b) { return math::safe_divide(a, b); }, exec_preset);
  static auto divide_floor_fn = mf::build::SI2_SO<int, int, int>(
      "Divide Floor", [](int a, int b) { return b == 0 ? 0 : divide_floor_i(a, b); }, exec_preset);
  static auto divide_ceil_fn = mf::build::SI2_SO<int, int, int>(
      "Divide Ceil",
      [](int a, int b) { return b == 0 ? 0 :
                                a == 0 ? 0 :
                                         divide_floor_i(a + b - 1, b); },
      exec_preset);
  static auto divide_round_fn = mf::build::SI2_SO<int, int, int>(
      "Divide Round", [](int a, int b) { return b == 0 ? 0 : divide_round_i(a, b); }, exec_preset);
  static auto square_fn = mf::build::SI1_SO<int, int>(
      "Square", [](int a) { return a * a; }, exec_preset);
  static auto cube_fn = mf::build::SI1_SO<int, int>(
      "Cube", [](int a) { return a * a * a; }, exec_preset);
  static auto distance_fn = mf::build::SI2_SO<int, int, int>(
      "Distance", [](int a, int b) { return math::distance(a, b); }, exec_preset);
  static auto pow_fn = mf::build::SI2_SO<int, int, int>(
      "Power", [](int a, int b) { return math::pow(a, b); }, exec_preset);
  static auto madd_fn = mf::build::SI3_SO<int, int, int, int>(
      "Multiply Add", [](int a, int b, int c) { return a * b + c; }, exec_preset);
  static auto mod_fn = mf::build::SI2_SO<int, int, int>(
      "Modulo", [](int a, int b) { return b != 0 ? mod_i(a, b) : 0; }, exec_preset);
  static auto remainder_fn = mf::build::SI2_SO<int, int, int>(
      "Remainder", [](int a, int b) { return b != 0 ? a % b : 0; }, exec_preset);
  static auto abs_fn = mf::build::SI1_SO<int, int>(
      "Absolute", [](int a) { return math::abs(a); }, exec_preset);
  static auto sign_fn = mf::build::SI1_SO<int, int>(
      "Sign", [](int a) { return math::sign(a); }, exec_preset);
  static auto min_fn = mf::build::SI2_SO<int, int, int>(
      "Minimum", [](int a, int b) { return math::min(a, b); }, exec_preset);
  static auto max_fn = mf::build::SI2_SO<int, int, int>(
      "Maximum", [](int a, int b) { return math::max(a, b); }, exec_preset);
  static auto gcd_fn = mf::build::SI2_SO<int, int, int>(
      "GCD", [](int a, int b) { return std::gcd(a, b); }, exec_preset);
  static auto lcm_fn = mf::build::SI2_SO<int, int, int>(
      "LCM", [](int a, int b) { return std::lcm(a, b); }, exec_preset);
  static auto negate_fn = mf::build::SI1_SO<int, int>(
      "Negate", [](int a) { return -a; }, exec_preset);
  static auto clamp_fn = mf::build::SI3_SO<int, int, int, int>(
      "Clamp", [](int a, int b, int c) { return math::clamp(a, b, c); }, exec_preset);
  static auto clamp_range_fn = mf::build::SI3_SO<int, int, int, int>(
      "Clamp Range",
      [](int a, int b, int c) { return (b > c) ? math::clamp(a, c, b) : math::clamp(a, b, c); },
      exec_preset);
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
    case NODE_INTEGER_MATH_ADD:
      return &add_fn;
    case NODE_INTEGER_MATH_SUBTRACT:
      return &sub_fn;
    case NODE_INTEGER_MATH_MULTIPLY:
      return &multiply_fn;
    case NODE_INTEGER_MATH_DIVIDE:
      return &divide_fn;
    case NODE_INTEGER_MATH_DIVIDE_FLOOR:
      return &divide_floor_fn;
    case NODE_INTEGER_MATH_DIVIDE_CEIL:
      return &divide_ceil_fn;
    case NODE_INTEGER_MATH_DIVIDE_ROUND:
      return &divide_round_fn;
    case NODE_INTEGER_MATH_DISTANCE:
      return &distance_fn;
    case NODE_INTEGER_MATH_SQUARE:
      return &square_fn;
    case NODE_INTEGER_MATH_CUBE:
      return &cube_fn;
    case NODE_INTEGER_MATH_POWER:
      return &pow_fn;
    case NODE_INTEGER_MATH_MULTIPLY_ADD:
      return &madd_fn;
    case NODE_INTEGER_MATH_MODULO:
      return &mod_fn;
    case NODE_INTEGER_MATH_REMAINDER:
      return &remainder_fn;
    case NODE_INTEGER_MATH_ABSOLUTE:
      return &abs_fn;
    case NODE_INTEGER_MATH_SIGN:
      return &sign_fn;
    case NODE_INTEGER_MATH_MINIMUM:
      return &min_fn;
    case NODE_INTEGER_MATH_MAXIMUM:
      return &max_fn;
    case NODE_INTEGER_MATH_GCD:
      return &gcd_fn;
    case NODE_INTEGER_MATH_LCM:
      return &lcm_fn;
    case NODE_INTEGER_MATH_NEGATE:
      return &negate_fn;
    case NODE_INTEGER_MATH_CLAMP:
      return &clamp_fn;
    case NODE_INTEGER_MATH_CLAMP_RANGE:
      return &clamp_range_fn;
    case NODE_INTEGER_MATH_AND:
      return &and_fn;
    case NODE_INTEGER_MATH_OR:
      return &or_fn;
    case NODE_INTEGER_MATH_XOR:
      return &xor_fn;
    case NODE_INTEGER_MATH_NOT:
      return &not_fn;
    case NODE_INTEGER_MATH_AND_NOT:
      return &and_not_fn;
    case NODE_INTEGER_MATH_LEFT_SHIFT:
      return &left_shift_fn;
    case NODE_INTEGER_MATH_RIGHT_SHIFT:
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
  static const EnumPropertyItem mode_items[] = {
      RNA_ENUM_ITEM_HEADING(CTX_N_(BLT_I18NCONTEXT_ID_NODETREE, "Basic Operations"), nullptr),
      {NODE_INTEGER_MATH_ADD, "ADD", 0, "Add", "A + B"},
      {NODE_INTEGER_MATH_SUBTRACT, "SUBTRACT", 0, "Subtract", "A - B"},
      {NODE_INTEGER_MATH_MULTIPLY, "MULTIPLY", 0, "Multiply", "A * B"},
      {NODE_INTEGER_MATH_DIVIDE, "DIVIDE", 0, "Divide", "A / B"},
      {NODE_INTEGER_MATH_MULTIPLY_ADD, "MULTIPLY_ADD", 0, "Multiply Add", "A * B + C"},
      RNA_ENUM_ITEM_HEADING(CTX_N_(BLT_I18NCONTEXT_ID_NODETREE, "1-Ops"), nullptr),
      {NODE_INTEGER_MATH_ABSOLUTE, "ABSOLUTE", 0, "Absolute", "Non-negative value of A, abs(A)"},
      {NODE_INTEGER_MATH_CUBE, "CUBE", 0, "Cube", "A * A * A"},
      {NODE_INTEGER_MATH_NEGATE, "NEGATE", 0, "Negate", "-A"},
      {NODE_INTEGER_MATH_SIGN, "SIGN", 0, "Sign", "Return the sign of A, sign(A)"},
      {NODE_INTEGER_MATH_SQUARE, "SQUARE", 0, "Square", "A * A"},
      RNA_ENUM_ITEM_HEADING(CTX_N_(BLT_I18NCONTEXT_ID_NODETREE, "2-Ops"), nullptr),
      {NODE_INTEGER_MATH_DISTANCE,
       "DISTANCE",
       0,
       "Distance",
       "Distance between A and B, abs(A-B)"},
      {NODE_INTEGER_MATH_DIVIDE_CEIL,
       "DIVIDE_CEIL",
       0,
       "Divide Ceiling",
       "Divide and ceil result, the smallest integer greater than or equal A"},
      {NODE_INTEGER_MATH_DIVIDE_FLOOR,
       "DIVIDE_FLOOR",
       0,
       "Divide Floor",
       "Divide and floor result, the largest integer smaller than or equal A"},
      {NODE_INTEGER_MATH_DIVIDE_ROUND,
       "DIVIDE_ROUND",
       0,
       "Divide Round",
       "Divide and round result toward zero"},
      {NODE_INTEGER_MATH_MODULO, "MODULO", 0, "Modulo", "Remainder of A / B"},
      {NODE_INTEGER_MATH_POWER, "POWER", 0, "Power", "A power B, pow(A,B)"},
      {NODE_INTEGER_MATH_REMAINDER, "REMAINDER", 0, "Remainder", "Signed remainder of A / B"},
      {NODE_INTEGER_MATH_MINIMUM,
       "MINIMUM",
       0,
       "Minimum",
       "The minimum value from A and B, min(A,B)"},
      {NODE_INTEGER_MATH_MAXIMUM,
       "MAXIMUM",
       0,
       "Maximum",
       "The maximum value from A and B, max(A,B)"},
      RNA_ENUM_ITEM_SEPR,
      {NODE_INTEGER_MATH_GCD,
       "GCD",
       0,
       "Greatest Divisor",
       "The largest positive integer that divides into each of the values A and B, "
       "e.g. GCD(8,12) = 4"},
      {NODE_INTEGER_MATH_LCM,
       "LCM",
       0,
       "Lowest Multiple",
       "The smallest positive integer that is divisible by both A and B, e.g. LCM(6,10) = 30"},
      RNA_ENUM_ITEM_HEADING(CTX_N_(BLT_I18NCONTEXT_ID_NODETREE, "3-Ops"), nullptr),
      {NODE_INTEGER_MATH_CLAMP,
       "CLAMP",
       0,
       "Clamp",
       "Limit the minimum and maximum value of A between Min and Max, clamp(A,Min,Max)"},
      {NODE_INTEGER_MATH_CLAMP_RANGE,
       "CLAMP_RANGE",
       0,
       "Clamp Range",
       "Limit the value of A between Min and Max, clamp_range(A,Min,Max)"},
      RNA_ENUM_ITEM_HEADING(CTX_N_(BLT_I18NCONTEXT_ID_NODETREE, "Bitwise Ops"), nullptr),
      {NODE_INTEGER_MATH_AND,
       "AND",
       0,
       "And",
       "Compares bit values of A and B then returns a value where the bits are both set. "
       "e.g. 0110 (6) AND 1011 (11) = 0010 (2), A & B"},
      {NODE_INTEGER_MATH_OR,
       "OR",
       0,
       "Or",
       "Compares bit values of A and B then returns a value where either bit is set, e.g. "
       "0101 (5) OR 0011 (3) = 0111 (7), A | B"},
      {NODE_INTEGER_MATH_XOR,
       "XOR",
       0,
       "Exclusive Or",
       "Compares bit values of A and B then returns a value where only one bit from A or B "
       "is set, 0101 (5) XOR 0011 (3) = 0110 (6), A ^ B"},
      {NODE_INTEGER_MATH_NOT,
       "NOT",
       0,
       "Not",
       "Returns the opposite bit value of A, in decimal it is equivalent of A = -A − 1, ~ A"},
      {NODE_INTEGER_MATH_AND_NOT,
       "AND_NOT",
       0,
       "And Not",
       "Returns the cleared bit value of A by B, A &= ~B"},
      {NODE_INTEGER_MATH_LEFT_SHIFT,
       "LEFT_SHIFT",
       0,
       "Left Shift",
       "Shifts the bit values of A to the left by B, A << B"},
      {NODE_INTEGER_MATH_RIGHT_SHIFT,
       "RIGHT_SHIFT",
       0,
       "Right Shift",
       "Shifts the bit values of A to the right by B, A >> B"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  PropertyRNA *prop;

  prop = RNA_def_node_enum(srna,
                           "operation",
                           "Operation",
                           "",
                           mode_items,
                           NOD_storage_enum_accessors(operation),
                           NODE_INTEGER_MATH_ADD);
  RNA_def_property_update_runtime(prop, rna_Node_socket_update);
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_INTEGER_MATH, "Integer Math", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.labelfunc = node_label;
  ntype.updatefunc = node_update;
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.initfunc = node_init;
  node_type_storage(
      &ntype, "NodeFunctionIntegerMath", node_free_standard_storage, node_copy_standard_storage);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_integer_math_cc
