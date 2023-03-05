/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_lib_id.h"
#include "BKE_material.h"

namespace blender::nodes::node_geo_copy_materials_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry")).supported_type(GEO_COMPONENT_TYPE_MESH);
  b.add_input<decl::Geometry>(N_("Source")).supported_type(GEO_COMPONENT_TYPE_MESH).multi_input();
  b.add_output<decl::Geometry>(N_("Geometry"));
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set_edit = params.extract_input<GeometrySet>("Geometry");
  Vector<GeometrySet> geometry_set_sources = params.extract_input<Vector<GeometrySet>>("Source");
  geometry_set_edit.modify_geometry_sets([&](GeometrySet &geometry_set_edit) {
    if (Mesh *mesh_edit = geometry_set_edit.get_mesh_for_write()) {
      ID *id = &mesh_edit->id;
      int offset = (mesh_edit->totcol);
      for (const int index : geometry_set_sources.index_range()) {
        if (const Mesh *mesh_source = geometry_set_sources[index].get_mesh_for_read()) {
          for (const int i : IndexRange(mesh_source->totcol)) {
            offset++;
            BKE_id_material_eval_assign(id, offset, mesh_source->mat[i]);
          }
        }
      }
    }
  });
  params.set_output("Geometry", std::move(geometry_set_edit));
}

}  // namespace blender::nodes::node_geo_copy_materials_cc

void register_node_type_geo_copy_materials()
{
  namespace file_ns = blender::nodes::node_geo_copy_materials_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_COPY_MATERIALS, "Copy Materials", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
