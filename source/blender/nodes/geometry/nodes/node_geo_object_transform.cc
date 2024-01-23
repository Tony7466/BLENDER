/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_math_matrix.hh"

#include "DNA_object_types.h"

#include "NOD_rna_define.hh"
#include "NOD_socket_search_link.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_object_transform_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Object>("Object").hide_label();
  b.add_output<decl::Matrix>("Transform");
}

static void search_link_ops(GatherLinkSearchOpParams &params)
{
  if (U.experimental.use_new_matrix_socket) {
    nodes::search_link_ops_for_basic_node(params);
  }
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "transform_space", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const Object *object = params.get_input<Object *>("Object");
  if (!object) {
    params.set_default_remaining_outputs();
    return;
  }
  if (params.node().custom1 == GEO_NODE_TRANSFORM_SPACE_RELATIVE) {
    params.set_output("Transform",
                      float4x4(params.self_object()->world_to_object) *
                          float4x4(object->object_to_world));
  }
  else {
    params.set_output("Transform", float4x4(object->object_to_world));
  }
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem rna_node_geometry_object_info_transform_space_items[] = {
      {GEO_NODE_TRANSFORM_SPACE_ORIGINAL,
       "ORIGINAL",
       0,
       "Original",
       "Output the transform relative to the world origin"},
      {GEO_NODE_TRANSFORM_SPACE_RELATIVE,
       "RELATIVE",
       0,
       "Relative",
       "Bring the object's transform into the modified object's space, "
       "maintaining the relative position between the two objects"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "transform_space",
                    "Transform Space",
                    "",
                    rna_node_geometry_object_info_transform_space_items,
                    NOD_inline_enum_accessors(custom1),
                    GEO_NODE_TRANSFORM_SPACE_ORIGINAL);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_OBJECT_TRANSFORM, "Object Transform", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_object_transform_cc
