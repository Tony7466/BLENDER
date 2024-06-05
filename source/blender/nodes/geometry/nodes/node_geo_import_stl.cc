/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_mesh.hh"

#include "BKE_report.hh"
#include "BLI_string.h"

#include "IO_stl.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_import_stl {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("Path").default_value("").description("Path to a STL file");

  b.add_output<decl::Geometry>("Mesh");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const std::string path = params.extract_input<std::string>("Path");

  if (path.empty()) {
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

  ReportList reports;

  BKE_reports_init(&reports, RPT_STORE);

  p.reports = &reports;

  Mesh *mesh = STL_import_mesh(&p);

  LISTBASE_FOREACH (Report *, report, &(p.reports)->list) {
    NodeWarningType type;

    switch (report->type) {
      case RPT_ERROR:
        type = NodeWarningType::Error;
        break;
      default:
        type = NodeWarningType::Info;
        break;
    }

    params.error_message_add(type, TIP_(report->message));
  }

  BKE_reports_free(&reports);

  if (mesh != nullptr) {
    params.set_output("Mesh", GeometrySet::from_mesh(mesh));
  }
  else {
    params.set_default_remaining_outputs();
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMPORT_STL, "Import STL", NODE_CLASS_INPUT);

  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops_for_import_node;

  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_import_stl
