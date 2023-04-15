/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_geo_input_collection_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Collection>(N_("Collection"));
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "collection", 0, "", ICON_NONE);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  params.set_output("Collection", reinterpret_cast<Collection *>(params.node().id));
}

} // namespace blender::nodes::node_geo_input_collection_cc

void register_node_type_geo_input_collection()
{
  namespace file_ns = blender::nodes::node_geo_input_collection_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_INPUT_COLLECTION, "Collection", NODE_CLASS_INPUT);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}