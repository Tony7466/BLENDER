/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_socket_search_link.hh"

#include "RNA_enum_types.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_named_attribute_cc {

NODE_STORAGE_FUNCS(NodeGeometryInputNamedAttribute)

static void node_declare_dynamic(const bNodeTree & /*node_tree*/,
                                 const bNode &node,
                                 NodeDeclarationBuilder &b)
{
  const NodeGeometryInputNamedAttribute &storage = node_storage(node);
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  b.add_input<decl::String>("Name").is_attribute_name();

  b.add_output(data_type, "Attribute").field_source();
  b.add_output<decl::Bool>("Exists").field_source();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "data_type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryInputNamedAttribute *data = MEM_cnew<NodeGeometryInputNamedAttribute>(__func__);
  data->data_type = CD_PROP_FLOAT;
  node->storage = data;
}

static void node_gather_link_searches(GatherLinkSearchOpParams &params)
{
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const NodeGeometryInputNamedAttribute &storage = node_storage(params.node());
  const eCustomDataType data_type = eCustomDataType(storage.data_type);

  const std::string name = params.extract_input<std::string>("Name");

  if (name.empty()) {
    params.set_default_remaining_outputs();
    return;
  }
  if (!bke::allow_procedural_attribute_access(name)) {
    params.error_message_add(NodeWarningType::Info, TIP_(bke::no_procedural_access_message));
    params.set_default_remaining_outputs();
    return;
  }

  params.used_named_attribute(name, NamedAttributeUsage::Read);

  const CPPType &type = *bke::custom_data_type_to_cpp_type(data_type);

  params.set_output<GField>("Attribute", AttributeFieldInput::Create(name, type));
  params.set_output("Exists", bke::AttributeExistsFieldInput::Create(std::move(name)));
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "data_type",
                    "Data Type",
                    "The data type used to read the attribute values",
                    rna_enum_attribute_type_items,
                    NOD_storage_enum_accessors(data_type),
                    CD_PROP_FLOAT,
                    enums::attribute_type_type_with_socket_fn);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_NAMED_ATTRIBUTE, "Named Attribute", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = node_gather_link_searches;
  ntype.declare_dynamic = node_declare_dynamic;
  ntype.initfunc = node_init;
  node_type_storage(&ntype,
                    "NodeGeometryInputNamedAttribute",
                    node_free_standard_storage,
                    node_copy_standard_storage);
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_named_attribute_cc
