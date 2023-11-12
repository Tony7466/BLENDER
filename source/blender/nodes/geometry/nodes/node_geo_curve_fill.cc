/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_delaunay_2d.h"
#include "BLI_math_vector_types.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
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
  b.add_input<decl::Geometry>("Curve").supported_type(
      {GeometryComponent::Type::Curve, GeometryComponent::Type::GreasePencil});
  b.add_input<decl::Int>("Group ID")
      .supports_field()
      .hide_value()
      .description(
          "An index used to group curves together; filling is done separately for each group");
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
  const OffsetIndices points_by_curve = curves.evaluated_points_by_curve();
  const Span<float3> positions = curves.evaluated_positions();

  meshintersect::CDT_input<double> input;
  input.need_ids = false;
  input.vert.reinitialize(points_by_curve.total_size());
  input.face.reinitialize(curves.curves_num());

  for (const int i_curve : curves.curves_range()) {
    const IndexRange points = points_by_curve[i_curve];

    for (const int i : points) {
      input.vert[i] = double2(positions[i].x, positions[i].y);
    }

    input.face[i_curve].resize(points.size());
    MutableSpan<int> face_verts = input.face[i_curve];
    for (const int i : face_verts.index_range()) {
      face_verts[i] = points[i];
    }
  }
  meshintersect::CDT_result<double> result = delaunay_2d_calc(input, output_type);
  return result;
}

static meshintersect::CDT_result<double> do_cdt_with_mask(const bke::CurvesGeometry &curves,
                                                          const CDT_output_type output_type,
                                                          const IndexMask &mask)
{
  const OffsetIndices points_by_curve = curves.evaluated_points_by_curve();
  const Span<float3> positions = curves.evaluated_positions();

  int vert_len = 0;
  mask.foreach_index([&](const int i) { vert_len += points_by_curve[i].size(); });

  meshintersect::CDT_input<double> input;
  input.need_ids = false;
  input.vert.reinitialize(vert_len);
  input.face.reinitialize(mask.size());

  Array<int> offsets_data(mask.size() + 1);
  const OffsetIndices points_by_curve_masked = offset_indices::gather_selected_offsets(
      points_by_curve, mask, offsets_data);

  mask.foreach_index(GrainSize(1024), [&](const int src_curve, const int dst_curve) {
    const IndexRange src_points = points_by_curve[src_curve];
    const IndexRange dst_points = points_by_curve_masked[dst_curve];

    for (const int i : src_points.index_range()) {
      const int src = src_points[i];
      const int dst = dst_points[i];
      input.vert[dst] = double2(positions[src].x, positions[src].y);
    }

    input.face[dst_curve].resize(src_points.size());
    array_utils::fill_index_range<int>(input.face[dst_curve], dst_points.start());
  });

  meshintersect::CDT_result<double> result = delaunay_2d_calc(input, output_type);
  return result;
}

static Vector<meshintersect::CDT_result<double>> do_group_aware_cdt(
    const bke::CurvesGeometry &curves,
    const CDT_output_type output_type,
    const Field<int> &group_index_field)
{
  const bke::GeometryFieldContext field_context{curves, ATTR_DOMAIN_CURVE};
  fn::FieldEvaluator data_evaluator{field_context, curves.curves_num()};
  data_evaluator.add(group_index_field);
  data_evaluator.evaluate();
  const VArray<int> curve_group_ids = data_evaluator.get_evaluated<int>(0);

  Vector<meshintersect::CDT_result<double>> cdt_results;

  if (curve_group_ids.is_single()) {
    cdt_results.append(do_cdt(curves, output_type));
    return cdt_results;
  }

  const VArraySpan<int> group_ids_span(curve_group_ids);
  const int domain_size = group_ids_span.size();

  VectorSet<int> group_indexing;
  for (const int index : IndexRange(domain_size)) {
    const int group_id = group_ids_span[index];
    group_indexing.add(group_id);
  }
  const int groups_num = group_indexing.size();

  IndexMaskMemory mask_memory;
  Array<IndexMask> group_masks(groups_num);

  const auto get_group_index = [&](const int i) {
    const int group_id = group_ids_span[i];
    return group_indexing.index_of(group_id);
  };

  IndexMask::from_groups<int>(IndexMask(domain_size), mask_memory, get_group_index, group_masks);

  /* The grain size should be larger as each group gets smaller. */
  const int avg_group_size = domain_size / groups_num;
  const int grain_size = std::max(8192 / avg_group_size, 1);
  threading::parallel_for(IndexRange(groups_num), grain_size, [&](const IndexRange range) {
    for (const int group_index : range) {
      const IndexMask &mask = group_masks[group_index];
      cdt_results.append(do_cdt_with_mask(curves, output_type, mask));
    }
  });

  return cdt_results;
}

/* Converts multiple CDT results into a single Mesh. */
static Mesh *cdts_to_mesh(const Span<meshintersect::CDT_result<double>> results)
{
  int vert_len = 0;
  int edge_len = 0;
  int face_len = 0;
  int loop_len = 0;
  for (const meshintersect::CDT_result<double> &result : results) {
    vert_len += result.vert.size();
    edge_len += result.edge.size();
    face_len += result.face.size();
    for (const Vector<int> &face : result.face) {
      loop_len += face.size();
    }
  }

  Mesh *mesh = BKE_mesh_new_nomain(vert_len, edge_len, face_len, loop_len);
  MutableSpan<float3> positions = mesh->vert_positions_for_write();
  MutableSpan<int2> edges = mesh->edges_for_write();
  MutableSpan<int> face_offsets = mesh->face_offsets_for_write();
  MutableSpan<int> corner_verts = mesh->corner_verts_for_write();

  int base_vert_index = 0;
  int base_edge_index = 0;
  int base_face_index = 0;
  int i_face_corner = 0;
  for (const meshintersect::CDT_result<double> &result : results) {
    for (const int i : IndexRange(result.vert.size())) {
      positions[i + base_vert_index] = float3(
          float(result.vert[i].x), float(result.vert[i].y), 0.0f);
    }
    for (const int i : IndexRange(result.edge.size())) {
      edges[i + base_edge_index] = int2(result.edge[i].first + base_vert_index,
                                        result.edge[i].second + base_vert_index);
    }
    for (const int i : IndexRange(result.face.size())) {
      face_offsets[i + base_face_index] = i_face_corner;
      for (const int j : result.face[i].index_range()) {
        corner_verts[i_face_corner] = result.face[i][j] + base_vert_index;
        i_face_corner++;
      }
    }

    base_vert_index += result.vert.size();
    base_edge_index += result.edge.size();
    base_face_index += result.face.size();
  }

  /* The delaunay triangulation doesn't seem to return all of the necessary edges, even in
   * triangulation mode. */
  BKE_mesh_calc_edges(mesh, true, false);
  BKE_mesh_smooth_flag_set(mesh, false);
  return mesh;
}

static void curve_fill_calculate(GeometrySet &geometry_set,
                                 const GeometryNodeCurveFillMode mode,
                                 const ValueOrField<int> &group_index_value_or_field)
{
  const CDT_output_type output_type = (mode == GEO_NODE_CURVE_FILL_MODE_NGONS) ?
                                          CDT_CONSTRAINTS_VALID_BMESH_WITH_HOLES :
                                          CDT_INSIDE_WITH_HOLES;
  if (geometry_set.has_curves()) {
    const Curves &curves_id = *geometry_set.get_curves();
    const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
    if (curves.curves_num() > 0) {
      if (group_index_value_or_field.is_field()) {
        const Vector<meshintersect::CDT_result<double>> results = do_group_aware_cdt(
            curves, output_type, group_index_value_or_field.as_field());
        Mesh *mesh = cdts_to_mesh(results);
        geometry_set.replace_mesh(mesh);
      }
      else {
        const meshintersect::CDT_result<double> result = do_cdt(curves, output_type);
        Mesh *mesh = cdts_to_mesh(Span(&result, 1));
        geometry_set.replace_mesh(mesh);
      }
    }
    geometry_set.replace_curves(nullptr);
  }

  if (geometry_set.has_grease_pencil()) {
    using namespace blender::bke::greasepencil;
    const GreasePencil &grease_pencil = *geometry_set.get_grease_pencil();
    Vector<Mesh *> mesh_by_layer(grease_pencil.layers().size(), nullptr);
    for (const int layer_index : grease_pencil.layers().index_range()) {
      const Drawing *drawing = get_eval_grease_pencil_layer_drawing(grease_pencil, layer_index);
      if (drawing == nullptr) {
        continue;
      }
      const bke::CurvesGeometry &src_curves = drawing->strokes();
      if (src_curves.curves_num() == 0) {
        continue;
      }
      if (group_index_value_or_field.is_field()) {
        const Vector<meshintersect::CDT_result<double>> results = do_group_aware_cdt(
            src_curves, output_type, group_index_value_or_field.as_field());
        mesh_by_layer[layer_index] = cdts_to_mesh(results);
      }
      else {
        const meshintersect::CDT_result<double> result = do_cdt(src_curves, output_type);
        mesh_by_layer[layer_index] = cdts_to_mesh(Span(&result, 1));
      }
    }
    if (!mesh_by_layer.is_empty()) {
      InstancesComponent &instances_component =
          geometry_set.get_component_for_write<InstancesComponent>();
      bke::Instances *instances = instances_component.get_for_write();
      if (instances == nullptr) {
        instances = new bke::Instances();
        instances_component.replace(instances);
      }
      for (Mesh *mesh : mesh_by_layer) {
        if (!mesh) {
          /* Add an empty reference so the number of layers and instances match.
           * This makes it easy to reconstruct the layers afterwards and keep their attributes.
           * Although in this particular case we don't propagate the attributes. */
          const int handle = instances->add_reference(bke::InstanceReference());
          instances->add_instance(handle, float4x4::identity());
          continue;
        }
        GeometrySet temp_set = GeometrySet::from_mesh(mesh);
        const int handle = instances->add_reference(bke::InstanceReference{temp_set});
        instances->add_instance(handle, float4x4::identity());
      }
    }
    geometry_set.replace_grease_pencil(nullptr);
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curve");
  ValueOrField<int> group_index_value_or_field = params.extract_input<ValueOrField<int>>(
      "Group ID");

  const NodeGeometryCurveFill &storage = node_storage(params.node());
  const GeometryNodeCurveFillMode mode = (GeometryNodeCurveFillMode)storage.mode;

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    curve_fill_calculate(geometry_set, mode, group_index_value_or_field);
  });

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
