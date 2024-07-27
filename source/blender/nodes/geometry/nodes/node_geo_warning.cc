/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_warning_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>("Show").default_value(false).hide_value();
  b.add_input<decl::String>("Message").hide_label();
}

class LazyFunctionForWarningNode : public LazyFunction {
  const bNode &node_;

 public:
  LazyFunctionForWarningNode(const bNode &node) : node_(node)
  {
    inputs_.append_as("Show", CPPType::get<SocketValueVariant>(), lf::ValueUsage::Used);
    inputs_.append_as("Message", CPPType::get<SocketValueVariant>());
  }

  void execute_impl(lf::Params &params, const lf::Context &context) const override
  {
    const bool show = params.get_input<SocketValueVariant>(0).get<bool>();
    if (!show) {
      return;
    }
    SocketValueVariant *message_variant =
        params.try_get_input_data_ptr_or_request<SocketValueVariant>(1);
    if (!message_variant) {
      return;
    }
    std::string message = message_variant->extract<std::string>();
    GeoNodesLFUserData &user_data = *static_cast<GeoNodesLFUserData *>(context.user_data);
    GeoNodesLFLocalUserData &local_user_data = *static_cast<GeoNodesLFLocalUserData *>(
        context.local_user_data);
    if (geo_eval_log::GeoTreeLogger *tree_logger = local_user_data.try_get_tree_logger(user_data))
    {
      tree_logger->node_warnings.append(
          *tree_logger->allocator,
          {node_.identifier, {NodeWarningType::Warning, std::move(message)}});
    }
  }
};

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_WARNING, "Warning", NODE_CLASS_OUTPUT);
  ntype.declare = node_declare;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_warning_cc

namespace blender::nodes {

std::unique_ptr<LazyFunction> get_warning_node_lazy_function(const bNode &node)
{
  using namespace node_geo_warning_cc;
  BLI_assert(node.type == GEO_NODE_WARNING);
  return std::make_unique<LazyFunctionForWarningNode>(node);
}

}  // namespace blender::nodes
