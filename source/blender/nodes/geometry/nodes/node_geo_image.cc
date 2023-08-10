/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_image_types.h"

#include "node_geometry_util.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

namespace blender::nodes::node_geo_image_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Image>("Image");
}

static void node_layout(uiLayout *layout, bContext *C, PointerRNA *ptr)
{
  uiTemplateID(layout,
               C,
               ptr,
               "image",
               "IMAGE_OT_new",
               "IMAGE_OT_open",
               nullptr,
               UI_TEMPLATE_ID_FILTER_ALL,
               false,
               nullptr);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Image", reinterpret_cast<Image *>(params.node().id));
}

static void node_rna(StructRNA *srna)
{
  PropertyRNA *prop;

  prop = RNA_def_property(srna, "image", PROP_POINTER, PROP_NONE);
  RNA_def_property_pointer_sdna(prop, nullptr, "id");
  RNA_def_property_struct_type(prop, "Image");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_REFCOUNT);
  RNA_def_property_ui_text(prop, "Image", "");
  RNA_def_property_update(prop, NC_NODE | NA_EDITED, "rna_Node_update");
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMAGE, "Image", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  ntype.declare = node_declare;
  blender::bke::node_type_size_preset(&ntype, blender::bke::eNodeSizePreset::LARGE);
  nodeRegisterType(&ntype);
  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_image_cc
