/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "RNA_enum_types.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "NOD_geometry.hh"
#include "NOD_socket.hh"
#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_for_each_group_input_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Int>("Group ID").hide_value().field_on_all();

  b.add_output<decl::Geometry>("Group Part").propagate_all();
  b.add_output<decl::Int>("Group Value");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = static_cast<bNode *>(ptr->data);
  printf(">>++ %d;\n", reinterpret_cast<const int &>(node->custom3));
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
  if (ELEM(eAttrDomain(node->custom2), ATTR_DOMAIN_POINT, ATTR_DOMAIN_EDGE, ATTR_DOMAIN_FACE)) {
    uiItemR(layout, ptr, "mode", UI_ITEM_NONE, "", ICON_NONE);
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_DELETE_GEOMETRY_MODE_ALL;
  node->custom2 = ATTR_DOMAIN_POINT;
  reinterpret_cast<int &>(node->custom3) = 0;
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_FOR_EACH_GROUP_INPUT, "For Each Group Input", NODE_CLASS_INTERFACE);
  ntype.initfunc = node_init;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.gather_link_search_ops = nullptr;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::

bool NOD_geometry_for_each_group_input_pair_with_output(const bNodeTree *node_tree,
                                                        bNode *feg_input_node,
                                                        const bNode *feg_output_node)
{
  namespace file_ns = blender::nodes::node_geo_for_each_group_input_cc;

  BLI_assert(feg_input_node->type == GEO_NODE_FOR_EACH_GROUP_INPUT);
  if (feg_output_node->type != GEO_NODE_FOR_EACH_GROUP_OUTPUT) {
    return false;
  }

  /* Allow only one input paired to an output. */
  for (const bNode *other_input_node : node_tree->nodes_by_type("GeometryNodeForEachGroupInput")) {
    if (other_input_node != feg_input_node) {
      if (reinterpret_cast<const int &>(other_input_node->custom3) == feg_output_node->identifier) {
        return false;
      }
    }
  }

  reinterpret_cast<int &>(feg_input_node->custom3) = feg_output_node->identifier;
  return true;
}