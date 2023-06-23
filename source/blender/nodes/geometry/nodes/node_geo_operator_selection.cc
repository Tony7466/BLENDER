/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_fields.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_operator_selection_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>("Selection");
}

class AttributeExistsFieldInput final : public bke::GeometryFieldInput {
 public:
  AttributeExistsFieldInput() : bke::GeometryFieldInput(CPPType::get<bool>(), "Operator Selection")
  {
    category_ = Category::NamedAttribute;
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask & /*mask*/) const final
  {
    switch (context.type()) {
      case GeometryComponent::Type::Curve:
        return *context.attributes()->lookup_or_default<bool>(
            ".selection", context.domain(), true);
      default:
        return {};
    }
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  if (!params.user_data()->operator_data) {
    params.error_message_add(NodeWarningType::Error, "Node must be run as operator");
    params.set_default_remaining_outputs();
  }
  params.set_output("Selection", Field<bool>(std::make_shared<AttributeExistsFieldInput>()));
}

}  // namespace blender::nodes::node_geo_operator_selection_cc

void register_node_type_geo_operator_selection()
{
  namespace file_ns = blender::nodes::node_geo_operator_selection_cc;
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_OPERATOR_SELECTION, "Operator Selection", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
