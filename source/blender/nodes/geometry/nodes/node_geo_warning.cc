/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_rna_define.hh"

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
          {node_.identifier, {this->get_warning_type(), std::move(message)}});
    }
  }

  NodeWarningType get_warning_type() const
  {
    switch (NodeGeometryWarningType(node_.custom1)) {
      case GEO_NODE_WARNING_TYPE_ERROR:
        return NodeWarningType::Error;
      case GEO_NODE_WARNING_TYPE_WARNING:
        return NodeWarningType::Warning;
      case GEO_NODE_WARNING_TYPE_INFO:
        return NodeWarningType::Info;
    }
    BLI_assert_unreachable();
    return NodeWarningType::Error;
  }
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "warning_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_rna(StructRNA *srna)
{
  static EnumPropertyItem warning_type_items[] = {
      {GEO_NODE_WARNING_TYPE_ERROR, "ERROR", 0, "Error", ""},
      {GEO_NODE_WARNING_TYPE_WARNING, "WARNING", 0, "Warning", ""},
      {GEO_NODE_WARNING_TYPE_INFO, "INFO", 0, "Info", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "warning_type",
                    "Warning Type",
                    "",
                    warning_type_items,
                    NOD_inline_enum_accessors(custom1));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_WARNING, "Warning", NODE_CLASS_OUTPUT);
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  blender::bke::nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
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
