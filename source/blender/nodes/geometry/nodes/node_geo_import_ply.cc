/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BLI_string.h"

#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_report.hh"

#include "IO_ply.hh"

#include "node_geometry_cache.hh"

namespace blender::nodes::nodes_geo_import_ply {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::String>("Path")
      .subtype(PROP_FILEPATH)
      .hide_label()
      .description("Path to a PLY file");

  b.add_output<decl::Geometry>("Mesh");
}

static void node_geo_exec(GeoNodeExecParams params)
{
#ifdef WITH_IO_PLY
  const std::string path = params.extract_input<std::string>("Path");
  if (path.empty()) {
    params.set_default_remaining_outputs();
    return;
  }

  GeometrySet output;

  if (geo_cache_contains(path)) {
    output = geo_cache_get(path);
  }
  else {
    PLYImportParams import_params{};
    STRNCPY(import_params.filepath, path.c_str());
    import_params.import_attributes = true;

    ReportList reports;
    BKE_reports_init(&reports, RPT_STORE);
    BLI_SCOPED_DEFER([&]() { BKE_reports_free(&reports); })
    import_params.reports = &reports;

    Mesh *mesh = PLY_import_mesh(&import_params);

    LISTBASE_FOREACH (Report *, report, &(import_params.reports)->list) {
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

    output = GeometrySet::from_mesh(mesh);

    geo_cache_set(path, output);
  }

  params.set_output("Mesh", output);

#else
  params.error_message_add(NodeWarningType::Error,
                           TIP_("Disabled, Blender was compiled without PLY I/O"));
  params.set_default_remaining_outputs();
#endif
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_IMPORT_PLY, "Import PLY", NODE_CLASS_INPUT);

  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.gather_link_search_ops = search_link_ops_for_import_node;

  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::nodes_geo_import_ply
