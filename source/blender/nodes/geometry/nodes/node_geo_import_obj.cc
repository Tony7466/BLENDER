/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_mesh.hh"

#include "BKE_report.hh"
#include "BLI_string.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_import_obj {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("Path")
      .default_value("")
      .subtype(PROP_FILEPATH)
      .hide_label()
      .description("Path to a STL file");

  b.add_output<decl::Geometry>("Mesh");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const std::string path = params.extract_input<std::string>("Path");

  if (path.empty()) {
    params.set_default_remaining_outputs();
    return;
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMPORT_OBJ, "Import OBJ", NODE_CLASS_INPUT);

  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops_for_import_node;

  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_import_obj
