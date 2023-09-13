/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_delaunay_2d.h"
#include "BLI_math_vector_types.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.hh"

#include "BLI_task.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_fill_cc {

NODE_STORAGE_FUNCS(NodeGeometryCurveFill)

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(GeometryComponent::Type::Curve);
  b.add_output<decl::Geometry>("Mesh");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  NodeGeometryCurveFill *data = MEM_cnew<NodeGeometryCurveFill>(__func__);

  data->mode = GEO_NODE_CURVE_FILL_MODE_TRIANGULATED;
  node->storage = data;
}

static meshintersect::CDT_result<double> do_cdt(const bke::CurvesGeometry &curves,
                                                const CDT_output_type output_type)
{
  const Span<float3> positions = curves.evaluated_positions();

  Array<double2> verts(positions.size());
  threading::parallel_for(positions.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      verts[i] = double2(positions[i].x, positions[i].y);
    }
  });
  Array<int> face_vert_indices(positions.size());
  std::iota(face_vert_indices.begin(), face_vert_indices.end(), 0);

  meshintersect::CDT_input<double> input;
  input.vert = verts;
  input.faces = curves.evaluated_points_by_curve();
  input.face_vert_indices = face_vert_indices;
  input.need_ids = false;
  return delaunay_2d_calc(input, output_type);
}

/* Converts the CDT result into a Mesh. */
static Mesh *cdt_to_mesh(const meshintersect::CDT_result<double> &result)
{
  Mesh *mesh = BKE_mesh_new_nomain(result.vert.size(),
                                   result.edge.size(),
                                   result.faces().size(),
                                   result.face_vert_indices.size());

  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  mesh->edges_for_write().copy_from(result.edge.as_span().cast<int2>());
  mesh->face_offsets_for_write().copy_from(result.face_offsets);
  mesh->corner_verts_for_write().copy_from(result.face_vert_indices);
  for (const int i : IndexRange(result.vert.size())) {
    positions[i] = float3(float(result.vert[i].x), float(result.vert[i].y), 0.0f);
  }

  /* The delaunay triangulation doesn't seem to return all of the necessary edges, even in
   * triangulation mode. */
  BKE_mesh_calc_edges(mesh, true, false);
  BKE_mesh_smooth_flag_set(mesh, false);
  return mesh;
}

static void curve_fill_calculate(GeometrySet &geometry_set, const GeometryNodeCurveFillMode mode)
{
  if (!geometry_set.has_curves()) {
    return;
  }

  const Curves &curves_id = *geometry_set.get_curves();
  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  if (curves.curves_num() == 0) {
    geometry_set.replace_curves(nullptr);
    return;
  }

  const CDT_output_type output_type = (mode == GEO_NODE_CURVE_FILL_MODE_NGONS) ?
                                          CDT_CONSTRAINTS_VALID_BMESH_WITH_HOLES :
                                          CDT_INSIDE_WITH_HOLES;

  const meshintersect::CDT_result<double> results = do_cdt(curves, output_type);
  Mesh *mesh = cdt_to_mesh(results);

  geometry_set.replace_mesh(mesh);
  geometry_set.remove<CurveComponent>();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");

  const NodeGeometryCurveFill &storage = node_storage(params.node());
  const GeometryNodeCurveFillMode mode = (GeometryNodeCurveFillMode)storage.mode;

  geometry_set.modify_geometry_sets(
      [&](GeometrySet &geometry_set) { curve_fill_calculate(geometry_set, mode); });

  params.set_output("Mesh", std::move(geometry_set));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem mode_items[] = {
      {GEO_NODE_CURVE_FILL_MODE_TRIANGULATED, "TRIANGLES", 0, "Triangles", ""},
      {GEO_NODE_CURVE_FILL_MODE_NGONS, "NGONS", 0, "N-gons", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "mode",
                    "Mode",
                    "",
                    mode_items,
                    NOD_storage_enum_accessors(mode),
                    GEO_NODE_CURVE_FILL_MODE_TRIANGULATED);
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_FILL_CURVE, "Fill Curve", NODE_CLASS_GEOMETRY);

  ntype.initfunc = node_init;
  node_type_storage(
      &ntype, "NodeGeometryCurveFill", node_free_standard_storage, node_copy_standard_storage);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_fill_cc
