/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_compute_contexts.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.h"

#include "UI_interface.h"
#include "UI_resources.h"

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

}  // namespace blender::nodes::node_geo_serial_loop_output_cc

void register_node_type_geo_serial_loop_output()
{
  namespace file_ns = blender::nodes::node_geo_serial_loop_output_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SERIAL_LOOP_OUTPUT, "Serial Loop Output", NODE_CLASS_INTERFACE);
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
