/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_material_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Material>("Material");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "material", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  Material *material = reinterpret_cast<Material *>(params.node().id);
  params.set_output("Material", material);
}

static void node_rna(StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "material", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Material");
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Material", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_MATERIAL, "Material", NODE_CLASS_INPUT);
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_material_cc
