/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cmath>

#include "BLI_hash.h"
#include "BLI_math_quaternion.hh"
#include "BLI_noise.hh"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "node_function_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_fn_hash_value_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("Value");
  b.add_input<decl::Vector>("Vector");
  b.add_input<decl::String>("String");
  b.add_input<decl::Color>("Color");
  b.add_input<decl::Rotation>("Rotation");
  b.add_input<decl::Int>("Integer", "Integer");
  b.add_input<decl::Int>("Integer", "Integer_001");
  b.add_input<decl::Int>("Integer", "Integer_002");
  b.add_output<decl::Int>("Hash");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = CD_PROP_INT32;
}

static eCustomDataType hash_socket_to_type(const eNodeSocketDatatype socket_type)
{
  switch (socket_type) {
    case SOCK_FLOAT:
      return CD_PROP_FLOAT;
    case SOCK_INT:
      return CD_PROP_INT32;
    case SOCK_BOOLEAN:
      return CD_PROP_INT32;
    case SOCK_VECTOR:
      return CD_PROP_FLOAT3;
    case SOCK_RGBA:
      return CD_PROP_COLOR;
    case SOCK_ROTATION:
      return CD_PROP_QUATERNION;
    case SOCK_STRING:
      return CD_PROP_STRING;
    default:
      /* Fallback. */
      return CD_AUTO_FROM_NAME;
  }
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const eCustomDataType data_type = static_cast<eCustomDataType>(node->custom1);

  LISTBASE_FOREACH (bNodeSocket *, socket, &node->inputs) {
    bke::nodeSetSocketAvailability(
        ntree, socket, data_type == hash_socket_to_type(eNodeSocketDatatype(socket->type)));
  }
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const eCustomDataType data_type = static_cast<eCustomDataType>(bnode.custom1);

  static auto exec_preset = mf::build::exec_presets::AllSpanOrSingle();

  static auto fn_hash_float = mf::build::SI1_SO<float, int>(
      "Hash Float", [](float a) { return noise::hash_float(a); }, exec_preset);
  static auto fn_hash_vector = mf::build::SI1_SO<float3, int>(
      "Hash Vector", [](float3 a) { return noise::hash_float(a); }, exec_preset);
  static auto fn_hash_color = mf::build::SI1_SO<ColorGeometry4f, int>(
      "Hash Color", [](ColorGeometry4f a) { return noise::hash_float(float4(a)); }, exec_preset);
  static auto fn_hash_int_i3 = mf::build::SI3_SO<int, int, int, int>(
      "Hash Integer", [](int a, int b, int c) { return noise::hash(a, b, c); }, exec_preset);
  static auto fn_hash_string = mf::build::SI1_SO<std::string, int>(
      "Hash String", [](std::string a) { return BLI_hash_string(a.c_str()); }, exec_preset);
  static auto fn_hash_rotation = mf::build::SI1_SO<math::Quaternion, int>(
      "Hash Rotation",
      [](math::Quaternion a) { return noise::hash_float(float4(a)); },
      exec_preset);

  switch (data_type) {
    case CD_PROP_QUATERNION:
      return &fn_hash_rotation;
    case CD_PROP_STRING:
      return &fn_hash_string;
    case CD_PROP_FLOAT:
      return &fn_hash_float;
    case CD_PROP_FLOAT3:
      return &fn_hash_vector;
    case CD_PROP_COLOR:
      return &fn_hash_color;
    case CD_PROP_INT32:
      return &fn_hash_int_i3;
    default:
      BLI_assert_unreachable();
      return nullptr;
  }
}

class SocketSearchOp {
 public:
  const StringRef socket_name;
  eCustomDataType data_type;
  void operator()(LinkSearchOpParams &params)
  {
    bNode &node = params.add_node("FunctionNodeHashValue");
    node.custom1 = data_type;
    params.update_and_connect_available_socket(node, socket_name);
  }
};

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  eCustomDataType type = hash_socket_to_type(eNodeSocketDatatype(params.other_socket().type));
  if (type == CD_AUTO_FROM_NAME) {
    return;
  }
  if (params.in_out() == SOCK_IN) {
    if (type == CD_PROP_FLOAT) {
      params.add_item(IFACE_("Value"), SocketSearchOp{"Value", type});
    }
    else if (type == CD_PROP_FLOAT3) {
      params.add_item(IFACE_("Vector"), SocketSearchOp{"Vector", type});
    }
    else if (type == CD_PROP_COLOR) {
      params.add_item(IFACE_("Color"), SocketSearchOp{"Color", type});
    }
    else if (type == CD_PROP_INT32) {
      params.add_item(IFACE_("Integer"), SocketSearchOp{"Integer", type});
    }
    else if (type == CD_PROP_STRING) {
      params.add_item(IFACE_("String"), SocketSearchOp{"String", type});
    }
    else if (type == CD_PROP_QUATERNION) {
      params.add_item(IFACE_("Rotation"), SocketSearchOp{"Rotation", type});
    }
  }
  else {
    if (ELEM(type, CD_PROP_STRING, CD_PROP_QUATERNION)) {
      return;
    }
    params.add_item(IFACE_("Hash"), SocketSearchOp{"Hash", CD_PROP_INT32});
  }
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(
      srna,
      "data_type",
      "Data Type",
      "Type of data stored in attribute",
      rna_enum_attribute_type_items,
      NOD_inline_enum_accessors(custom1),
      CD_PROP_INT32,
      [](bContext * /*C*/, PointerRNA * /*ptr*/, PropertyRNA * /*prop*/, bool *r_free) {
        *r_free = true;
        return enum_items_filter(rna_enum_attribute_type_items, [](const EnumPropertyItem &item) {
          return ELEM(item.value,
                      CD_PROP_FLOAT,
                      CD_PROP_FLOAT3,
                      CD_PROP_STRING,
                      CD_PROP_COLOR,
                      CD_PROP_INT32,
                      CD_PROP_QUATERNION);
        });
      });
}

static void node_register()
{
  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_HASH_VALUE, "Hash Value", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.updatefunc = node_update;
  ntype.initfunc = node_init;
  ntype.build_multi_function = node_build_multi_function;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_fn_hash_value_cc
