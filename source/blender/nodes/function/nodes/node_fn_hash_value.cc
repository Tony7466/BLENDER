#include <cmath>

#include "BLI_hash.h"
#include "BLI_noise.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_function_util.hh"

#include "NOD_socket_search_link.hh"

namespace blender::nodes::node_fn_hash_value_cc {

NODE_STORAGE_FUNCS(NodeHashValue)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Float>("Value");
  b.add_input<decl::Vector>("Vector");
  b.add_input<decl::String>("String");
  b.add_input<decl::Color>("Color");
  b.add_input<decl::Int>("Integer", "Integer");
  b.add_input<decl::Int>("Integer", "Integer_001");
  b.add_input<decl::Int>("Integer", "Integer_002");
  b.add_input<decl::Int>("Hash");
  b.add_output<decl::Int>("Hash");
  b.add_output<decl::Float>("Value");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(const bNodeTree * /*tree*/, bNode *node)
{
  NodeHashValue *data = MEM_cnew<NodeHashValue>(__func__);
  data->mode = NODE_HASH_FLOAT;
  node->storage = data;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const NodeHashValue &storage = node_storage(*node);
  const NodeHashMode mode = static_cast<NodeHashMode>(storage.mode);

  const bool to_hash = mode != NODE_HASH_TO_FLOAT;
  const bool to_float = mode == NODE_HASH_TO_FLOAT;

  bNodeSocket *sock_value = (bNodeSocket *)node->inputs.first;
  bNodeSocket *sock_vector = sock_value->next;
  bNodeSocket *sock_string = sock_vector->next;
  bNodeSocket *sock_color = sock_string->next;
  bNodeSocket *sock_int_a = sock_color->next;
  bNodeSocket *sock_int_b = sock_int_a->next;
  bNodeSocket *sock_int_c = sock_int_b->next;
  bNodeSocket *sock_hash = sock_int_c->next;
  bke::nodeSetSocketAvailability(ntree, sock_value, mode == NODE_HASH_FLOAT && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_vector, mode == NODE_HASH_VECTOR && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_string, mode == NODE_HASH_STRING && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_color, mode == NODE_HASH_COLOR && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_int_a, mode == NODE_HASH_INTEGER && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_int_b, mode == NODE_HASH_INTEGER && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_int_c, mode == NODE_HASH_INTEGER && to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_hash, to_float);

  bNodeSocket *sock_out_hash = (bNodeSocket *)node->outputs.first;
  bNodeSocket *sock_out_value = sock_out_hash->next;
  bke::nodeSetSocketAvailability(ntree, sock_out_hash, to_hash);
  bke::nodeSetSocketAvailability(ntree, sock_out_value, to_float);
}

static const mf::MultiFunction *get_multi_function(const bNode &bnode)
{
  const NodeHashValue &storage = node_storage(bnode);
  const NodeHashMode mode = static_cast<NodeHashMode>(storage.mode);

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
  static auto fn_hash_to_float = mf::build::SI1_SO<int, float>(
      "Hash to Float", [](int a) { return noise::hash_to_float(a); }, exec_preset);

  switch (mode) {
    case NODE_HASH_STRING:
      return &fn_hash_string;
    case NODE_HASH_TO_FLOAT:
      return &fn_hash_to_float;
    case NODE_HASH_FLOAT:
      return &fn_hash_float;
    case NODE_HASH_VECTOR:
      return &fn_hash_vector;
    case NODE_HASH_COLOR:
      return &fn_hash_color;
    case NODE_HASH_INTEGER:
      return &fn_hash_int_i3;
    default:
      BLI_assert_unreachable();
      return nullptr;
  }
}

static std::optional<NodeHashMode> mode_from_socket_type(const bNodeSocket &socket)
{
  switch (socket.type) {
    case SOCK_FLOAT:
      return NODE_HASH_FLOAT;
    case SOCK_VECTOR:
      return NODE_HASH_VECTOR;
    case SOCK_RGBA:
      return NODE_HASH_COLOR;
    case SOCK_INT:
      return NODE_HASH_INTEGER;
    case SOCK_STRING:
      return NODE_HASH_STRING;
    default:
      return {};
  }
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
  const std::optional<NodeHashMode> mode = mode_from_socket_type(params.other_socket());
  if (!mode) {
    return;
  }
  if (params.in_out() == SOCK_IN) {
    if (*mode == NODE_HASH_FLOAT) {
      params.add_item(IFACE_("Value"), [mode](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = *mode;
        params.update_and_connect_available_socket(node, "Value");
      });
    }
    else if (*mode == NODE_HASH_VECTOR) {
      params.add_item(IFACE_("Value"), [mode](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = *mode;
        params.update_and_connect_available_socket(node, "Vector");
      });
    }
    else if (*mode == NODE_HASH_COLOR) {
      params.add_item(IFACE_("Value"), [mode](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = *mode;
        params.update_and_connect_available_socket(node, "Color");
      });
    }
    else if (*mode == NODE_HASH_INTEGER) {
      params.add_item(IFACE_("Value"), [mode](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = *mode;
        params.update_and_connect_available_socket(node, "Integer");
      });
      params.add_item(IFACE_("Hash"), [](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = NODE_HASH_TO_FLOAT;
        params.update_and_connect_available_socket(node, "Integer");
      });
    }
    else if (*mode == NODE_HASH_STRING) {
      params.add_item(IFACE_("Value"), [mode](LinkSearchOpParams &params) {
        bNode &node = params.add_node("FunctionNodeHashValue");
        node_storage(node).mode = *mode;
        params.update_and_connect_available_socket(node, "String");
      });
    }
  }
  else {
    params.add_item(IFACE_("Hash"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("FunctionNodeHashValue");
      node_storage(node).mode = NODE_HASH_FLOAT;
      params.update_and_connect_available_socket(node, "Hash");
    });
    params.add_item(IFACE_("Value"), [](LinkSearchOpParams &params) {
      bNode &node = params.add_node("FunctionNodeHashValue");
      node_storage(node).mode = NODE_HASH_TO_FLOAT;
      params.update_and_connect_available_socket(node, "Value");
    });
  }
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const mf::MultiFunction *fn = get_multi_function(builder.node());
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_hash_value_cc

void register_node_type_fn_hash_value()
{
  namespace file_ns = blender::nodes::node_fn_hash_value_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_HASH_VALUE, "Hash Value", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.updatefunc = file_ns::node_update;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.gather_link_search_ops = file_ns::node_gather_link_searches;
  node_type_storage(
      &ntype, "NodeHashValue", node_free_standard_storage, node_copy_standard_storage);
  nodeRegisterType(&ntype);
}
