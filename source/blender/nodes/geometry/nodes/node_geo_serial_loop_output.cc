/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_add_node_search.hh"
#include "NOD_geometry.h"
#include "NOD_socket.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_serial_loop_output_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Bool>(N_("Break")).hide_value();
  b.add_input<decl::Bool>(N_("Output Previous"));
  b.add_input<decl::Geometry>(N_("Geometry"));
  b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
}

static void search_node_add_ops(GatherAddNodeSearchParams &params)
{
  AddNodeItem item;
  item.ui_name = IFACE_("Serial Loop Zone");
  item.description = TIP_("Add a new serial loop input and output nodes to the node tree");
  item.add_fn = [](const bContext &C, bNodeTree &node_tree, float2 cursor) {
    bNode *input = nodeAddNode(&C, &node_tree, "GeometryNodeSerialLoopInput");
    bNode *output = nodeAddNode(&C, &node_tree, "GeometryNodeSerialLoopOutput");
    static_cast<NodeGeometrySerialLoopInput *>(input->storage)->output_node_id =
        output->identifier;

    input->locx = cursor.x / UI_SCALE_FAC - 150;
    input->locy = cursor.y / UI_SCALE_FAC + 20;
    output->locx = cursor.x / UI_SCALE_FAC + 150;
    output->locy = cursor.y / UI_SCALE_FAC + 20;

    return Vector<bNode *>({input, output});
  };
  params.add_item(std::move(item));
}

}  // namespace blender::nodes::node_geo_serial_loop_output_cc

void register_node_type_geo_serial_loop_output()
{
  namespace file_ns = blender::nodes::node_geo_serial_loop_output_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SERIAL_LOOP_OUTPUT, "Serial Loop Output", NODE_CLASS_INTERFACE);
  ntype.declare = file_ns::node_declare;
  ntype.gather_add_node_search_ops = file_ns::search_node_add_ops;
  nodeRegisterType(&ntype);
}
