/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_mesh.hh"

#include "BLI_string.h"

#include "IO_stl.hh"

namespace blender::nodes::node_geo_import_stl {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("Path").default_value("");

  b.add_output<decl::Geometry>("Mesh");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const std::string path = params.extract_input<std::string>("Path");

  if (path.empty()) {
    params.error_message_add(NodeWarningType::Error, TIP_("File path can't be empty"));
    params.set_default_remaining_outputs();
    return;
  }

  STLImportParams p;

  STRNCPY(p.filepath, path.c_str());

  p.forward_axis = IO_AXIS_NEGATIVE_Z;
  p.up_axis = IO_AXIS_Y;
  p.use_facet_normal = false;
  p.use_scene_unit = false;
  p.global_scale = 1.0f;
  p.use_mesh_validate = true;

  Mesh *mesh = STL_import_mesh(&p);

  if (mesh != nullptr) {
    params.set_output("Mesh", GeometrySet::from_mesh(mesh));
  }
  else {
    params.error_message_add(NodeWarningType::Error, TIP_("STL Import Failed"));
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMPORT_STL, "Import STL", NODE_CLASS_GEOMETRY);

  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;

  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_import_stl
