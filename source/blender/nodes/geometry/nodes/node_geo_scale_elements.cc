/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "atomic_ops.h"

#include "BLI_array.hh"
#include "BLI_atomic_disjoint_set.hh"
#include "BLI_disjoint_set.hh"
#include "BLI_math_matrix.hh"
#include "BLI_task.hh"
#include "BLI_vector.hh"
#include "BLI_vector_set.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BKE_mesh.hh"

#include "NOD_rna_define.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_scale_elements_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_input<decl::Float>("Scale", "Scale").default_value(1.0f).min(0.0f).field_on_all();
  b.add_input<decl::Vector>("Center")
      .subtype(PROP_TRANSLATION)
      .implicit_field_on_all(implicit_field_inputs::position)
      .description(
          "Origin of the scaling for each element. If multiple elements are connected, their "
          "center is averaged");
  b.add_input<decl::Vector>("Axis")
      .default_value({1.0f, 0.0f, 0.0f})
      .field_on_all()
      .description("Direction in which to scale the element")
      .make_available([](bNode &node) { node.custom2 = GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS; });
  b.add_output<decl::Geometry>("Geometry").propagate_all();
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
  uiItemR(layout, ptr, "scale_mode", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = ATTR_DOMAIN_FACE;
  node->custom2 = GEO_NODE_SCALE_ELEMENTS_UNIFORM;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  bNodeSocket *geometry_socket = static_cast<bNodeSocket *>(node->inputs.first);
  bNodeSocket *selection_socket = geometry_socket->next;
  bNodeSocket *scale_float_socket = selection_socket->next;
  bNodeSocket *center_socket = scale_float_socket->next;
  bNodeSocket *axis_socket = center_socket->next;

  const GeometryNodeScaleElementsMode mode = GeometryNodeScaleElementsMode(node->custom2);
  const bool use_single_axis = mode == GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS;

  bke::nodeSetSocketAvailability(ntree, axis_socket, use_single_axis);
}

static Array<int> create_reverse_offsets(const Span<int> indices, const int items_num)
{
  Array<int> offsets(items_num + 1, 0);
  offset_indices::build_reverse_offsets(indices, offsets);
  return offsets;
}

static void sort_small_groups(const OffsetIndices<int> groups,
                              const int grain_size,
                              MutableSpan<int> indices)
{
  threading::parallel_for(groups.index_range(), grain_size, [&](const IndexRange range) {
    for (const int64_t index : range) {
      MutableSpan<int> group = indices.slice(groups[index]);
      std::sort(group.begin(), group.end());
    }
  });
}

static Array<int> reverse_indices_in_groups(const Span<int> group_indices,
                                            const OffsetIndices<int> offsets)
{
  if (group_indices.is_empty()) {
    return {};
  }
  BLI_assert(*std::max_element(group_indices.begin(), group_indices.end()) < offsets.size());
  BLI_assert(*std::min_element(group_indices.begin(), group_indices.end()) >= 0);

  /* `counts` keeps track of how many elements have been added to each group, and is incremented
   * atomically by many threads in parallel. `calloc` can be measurably faster than a parallel fill
   * of zero. Alternatively the offsets could be copied and incremented directly, but the cost of
   * the copy is slightly higher than the cost of `calloc`. */
  int *counts = MEM_cnew_array<int>(size_t(offsets.size()), __func__);
  BLI_SCOPED_DEFER([&]() { MEM_freeN(counts); })
  Array<int> results(group_indices.size());
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const int group_index = group_indices[i];
      const int index_in_group = atomic_fetch_and_add_int32(&counts[group_index], 1);
      results[offsets[group_index][index_in_group]] = int(i);
    }
  });
  sort_small_groups(offsets, 1024, results);
  return results;
}

static GroupedSpan<int> gather_groups(const Span<int> group_indices,
                                      const int groups_num,
                                      Array<int> &r_offsets,
                                      Array<int> &r_indices)
{
  r_offsets = create_reverse_offsets(group_indices, groups_num);
  r_indices = reverse_indices_in_groups(group_indices, r_offsets.as_span());
  return {OffsetIndices<int>(r_offsets), r_indices};
}

struct UniformScaleFields {
  Field<bool> selection;
  Field<float> scale;
  Field<float3> center;
};

struct UniformScaleParams {
  IndexMask selection;
  VArray<float> scales;
  VArray<float3> centers;
};

struct AxisScaleFields {
  Field<bool> selection;
  Field<float> scale;
  Field<float3> center;
  Field<float3> axis;
};

struct AxisScaleParams {
  IndexMask selection;
  VArray<float> scales;
  VArray<float3> centers;
  VArray<float3> axis_vectors;
};

/**
 * When multiple elements share the same vertices, they are scaled together.
 */
struct ElementIsland {
  /* Either face or edge indices. */
  Vector<int> element_indices;
};

static float3 transform_with_uniform_scale(const float3 &position,
                                           const float3 &center,
                                           const float scale)
{
  const float3 diff = position - center;
  const float3 scaled_diff = scale * diff;
  const float3 new_position = center + scaled_diff;
  return new_position;
}

static float4x4 create_single_axis_transform(const float3 &center,
                                             const float3 &axis,
                                             const float scale)
{
  /* Scale along x axis. The other axis need to be orthogonal, but their specific value does not
   * matter. */
  const float3 x_axis = math::normalize(axis);
  float3 y_axis = math::cross(x_axis, float3(0.0f, 0.0f, 1.0f));
  if (math::is_zero(y_axis)) {
    y_axis = math::cross(x_axis, float3(0.0f, 1.0f, 0.0f));
  }
  y_axis = math::normalize(y_axis);
  const float3 z_axis = math::cross(x_axis, y_axis);

  float4x4 transform = float4x4::identity();

  /* Move scaling center to the origin. */
  transform.location() -= center;

  /* `base_change` and `base_change_inv` are used to rotate space so that scaling along the
   * provided axis is the same as scaling along the x axis. */
  float4x4 base_change = float4x4::identity();
  base_change.x_axis() = x_axis;
  base_change.y_axis() = y_axis;
  base_change.z_axis() = z_axis;

  /* Can invert by transposing, because the matrix is orthonormal. */
  float4x4 base_change_inv = math::transpose(base_change);

  float4x4 scale_transform = float4x4::identity();
  scale_transform[0][0] = scale;

  transform = base_change * scale_transform * base_change_inv * transform;

  /* Move scaling center back to where it was. */
  transform.location() += center;

  return transform;
}

using GetVertexIndicesFn = FunctionRef<void(Span<int2> edges,
                                            OffsetIndices<int> faces,
                                            Span<int> corner_verts,
                                            int element_index,
                                            VectorSet<int> &r_vertex_indices)>;

static int face_island_indices(const Mesh &mesh,
                               const IndexMask &face_selection,
                               MutableSpan<int> r_vert_island_indices,
                               MutableSpan<int> r_face_island_indices)
{
  AtomicDisjointSet disjoint_set(mesh.totvert);
  const GroupedSpan<int> face_verts(mesh.face_offsets(), mesh.corner_verts());
  face_selection.foreach_index(GrainSize(2048), [&](const int face_index) {
    const Span<int> verts = face_verts[face_index];
    for (const int loop_index : verts.index_range().drop_back(1)) {
      const int v1 = verts[loop_index];
      const int v2 = verts[loop_index + 1];
      disjoint_set.join(v1, v2);
    }
    disjoint_set.join(verts.first(), verts.last());
  });

  disjoint_set.calc_reduced_ids(r_vert_island_indices);

  face_selection.foreach_index(GrainSize(4096), [&](const int face_index, const int face_pos) {
    const int face_vertex_i = face_verts[face_index].first();
    const int face_island = r_vert_island_indices[face_vertex_i];
    r_face_island_indices[face_pos] = face_island;
  });

  return disjoint_set.count_sets();
}

static int edge_island_indices(const Mesh &mesh,
                               const IndexMask &edge_selection,
                               MutableSpan<int> r_vert_island_indices,
                               MutableSpan<int> r_edge_island_indices)
{
  AtomicDisjointSet disjoint_set(mesh.totvert);
  const Span<int2> edges = mesh.edges();
  edge_selection.foreach_index(GrainSize(4096), [&](const int edge_index) {
    const int2 &edge = edges[edge_index];
    disjoint_set.join(edge[0], edge[1]);
  });

  disjoint_set.calc_reduced_ids(r_vert_island_indices);

  edge_selection.foreach_index(GrainSize(4096), [&](const int edge_index, const int edge_pos) {
    const int edge_vertex_i = edges[edge_index][0];
    const int edge_island = r_vert_island_indices[edge_vertex_i];
    r_edge_island_indices[edge_pos] = edge_island;
  });

  return disjoint_set.count_sets();
}

static void scale_vertex_islands_uniformly(Mesh &mesh,
                                           const GroupedSpan<int> vert_islands,
                                           const GroupedSpan<int> elem_islands,
                                           const UniformScaleParams &params)
{
  MutableSpan<float3> positions = mesh.vert_positions_for_write();
  threading::parallel_for(elem_islands.index_range(), 256, [&](const IndexRange range) {
    for (const int island_index : range) {
      const Span<int> vert_island = vert_islands[island_index];
      const Span<int> elem_island = elem_islands[island_index];

      float scale = 0.0f;
      float3 center = {0.0f, 0.0f, 0.0f};
      for (const int index : elem_island) {
        center += params.centers[index];
        scale += params.scales[index];
      }

      scale /= elem_island.size();
      center /= elem_island.size();

      for (const int vert_index : vert_island) {
        positions[vert_index] = transform_with_uniform_scale(positions[vert_index], center, scale);
      }
    }
  });

  BKE_mesh_tag_positions_changed(&mesh);
}

static void scale_vertex_islands_on_axis(Mesh &mesh,
                                         const GroupedSpan<int> vert_islands,
                                         const GroupedSpan<int> elem_islands,
                                         const AxisScaleParams &params)
{
  MutableSpan<float3> positions = mesh.vert_positions_for_write();
  threading::parallel_for(elem_islands.index_range(), 256, [&](const IndexRange range) {
    for (const int island_index : range) {
      const Span<int> vert_island = vert_islands[island_index];
      const Span<int> elem_island = elem_islands[island_index];

      float scale = 0.0f;
      float3 center(0.0f);
      float3 axis(0.0f);
      for (const int index : elem_island) {
        center += params.centers[index];
        scale += params.scales[index];
        axis += params.axis_vectors[index];
      }

      scale /= elem_island.size();
      center /= elem_island.size();
      axis /= elem_island.size();

      if (math::is_zero(axis)) {
        axis = float3(1.0f, 0.0f, 0.0f);
      }

      const float4x4 transform = create_single_axis_transform(center, axis, scale);
      for (const int vert_index : vert_island) {
        positions[vert_index] = math::transform_point(transform, positions[vert_index]);
      }
    }
  });

  BKE_mesh_tag_positions_changed(&mesh);
}

static AxisScaleParams evaluate_axis_scale_fields(FieldEvaluator &evaluator,
                                                  const AxisScaleFields &fields)
{
  AxisScaleParams out;
  evaluator.set_selection(fields.selection);
  evaluator.add(fields.scale, &out.scales);
  evaluator.add(fields.center, &out.centers);
  evaluator.add(fields.axis, &out.axis_vectors);
  evaluator.evaluate();
  out.selection = evaluator.get_evaluated_selection_as_mask();
  return out;
}

template<typename T, typename Function>
inline void parallel_transform(MutableSpan<T> data,
                               const int64_t grain_size,
                               const Function &function)
{
  threading::parallel_for(data.index_range(), grain_size, [&](const IndexRange range) {
    MutableSpan<T> data_range = data.slice(range);
    std::transform(data_range.begin(), data_range.end(), data_range.begin(), function);
  });
}

static void scale_faces_on_axis(Mesh &mesh, const AxisScaleFields &fields)
{
  const bke::MeshFieldContext field_context{mesh, ATTR_DOMAIN_FACE};
  FieldEvaluator evaluator{field_context, mesh.faces_num};
  AxisScaleParams params = evaluate_axis_scale_fields(evaluator, fields);

  Array<int> vert_offsets;
  Array<int> vert_indices;

  Array<int> face_offsets;
  Array<int> face_indices;
  {
    Array<int> vert_island_indices(mesh.totvert);
    Array<int> face_island_indices_data(params.selection.size());
    const int total_islands = face_island_indices(
        mesh, params.selection, vert_island_indices, face_island_indices_data);
    gather_groups(vert_island_indices, total_islands, vert_offsets, vert_indices);
    gather_groups(face_island_indices_data, total_islands, face_offsets, face_indices);
    parallel_transform<int>(
        face_indices, 2048, [&](const int pos) { return params.selection[pos]; });
  }
  const GroupedSpan<int> vert_islands(vert_offsets.as_span(), vert_indices);
  const GroupedSpan<int> face_islands(face_offsets.as_span(), face_indices);
  scale_vertex_islands_on_axis(mesh, vert_islands, face_islands, params);
}

static UniformScaleParams evaluate_uniform_scale_fields(FieldEvaluator &evaluator,
                                                        const UniformScaleFields &fields)
{
  UniformScaleParams out;
  evaluator.set_selection(fields.selection);
  evaluator.add(fields.scale, &out.scales);
  evaluator.add(fields.center, &out.centers);
  evaluator.evaluate();
  out.selection = evaluator.get_evaluated_selection_as_mask();
  return out;
}

static void scale_faces_uniformly(Mesh &mesh, const UniformScaleFields &fields)
{
  const bke::MeshFieldContext field_context{mesh, ATTR_DOMAIN_FACE};
  FieldEvaluator evaluator{field_context, mesh.faces_num};
  UniformScaleParams params = evaluate_uniform_scale_fields(evaluator, fields);

  Array<int> vert_offsets;
  Array<int> vert_indices;

  Array<int> face_offsets;
  Array<int> face_indices;
  {
    Array<int> vert_island_indices(mesh.totvert);
    Array<int> face_island_indices_data(params.selection.size());
    const int total_islands = face_island_indices(
        mesh, params.selection, vert_island_indices, face_island_indices_data);
    gather_groups(vert_island_indices, total_islands, vert_offsets, vert_indices);
    gather_groups(face_island_indices_data, total_islands, face_offsets, face_indices);
    parallel_transform<int>(
        face_indices, 2048, [&](const int pos) { return params.selection[pos]; });
  }
  const GroupedSpan<int> vert_islands(vert_offsets.as_span(), vert_indices);
  const GroupedSpan<int> face_islands(face_offsets.as_span(), face_indices);
  scale_vertex_islands_uniformly(mesh, vert_islands, face_islands, params);
}

static void scale_edges_uniformly(Mesh &mesh, const UniformScaleFields &fields)
{
  const bke::MeshFieldContext field_context{mesh, ATTR_DOMAIN_EDGE};
  FieldEvaluator evaluator{field_context, mesh.totedge};
  UniformScaleParams params = evaluate_uniform_scale_fields(evaluator, fields);

  Array<int> vert_offsets;
  Array<int> vert_indices;

  Array<int> edge_offsets;
  Array<int> edge_indices;
  {
    Array<int> vert_island_indices(mesh.totvert);
    Array<int> edge_island_indices_data(params.selection.size());
    const int total_islands = edge_island_indices(
        mesh, params.selection, vert_island_indices, edge_island_indices_data);
    gather_groups(vert_island_indices, total_islands, vert_offsets, vert_indices);
    gather_groups(edge_island_indices_data, total_islands, edge_offsets, edge_indices);
    parallel_transform<int>(
        edge_indices, 2048, [&](const int pos) { return params.selection[pos]; });
  }
  const GroupedSpan<int> vert_islands(vert_offsets.as_span(), vert_indices);
  const GroupedSpan<int> edge_islands(edge_offsets.as_span(), edge_indices);
  scale_vertex_islands_uniformly(mesh, vert_islands, edge_islands, params);
}

static void scale_edges_on_axis(Mesh &mesh, const AxisScaleFields &fields)
{
  const bke::MeshFieldContext field_context{mesh, ATTR_DOMAIN_EDGE};
  FieldEvaluator evaluator{field_context, mesh.totedge};
  AxisScaleParams params = evaluate_axis_scale_fields(evaluator, fields);

  Array<int> vert_offsets;
  Array<int> vert_indices;

  Array<int> edge_offsets;
  Array<int> edge_indices;
  {
    Array<int> vert_island_indices(mesh.totvert);
    Array<int> edge_island_indices_data(params.selection.size());
    const int total_islands = edge_island_indices(
        mesh, params.selection, vert_island_indices, edge_island_indices_data);
    gather_groups(vert_island_indices, total_islands, vert_offsets, vert_indices);
    gather_groups(edge_island_indices_data, total_islands, edge_offsets, edge_indices);
    parallel_transform<int>(
        edge_indices, 2048, [&](const int pos) { return params.selection[pos]; });
  }
  const GroupedSpan<int> vert_islands(vert_offsets.as_span(), vert_indices);
  const GroupedSpan<int> edge_islands(edge_offsets.as_span(), edge_indices);
  scale_vertex_islands_on_axis(mesh, vert_islands, edge_islands, params);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const bNode &node = params.node();
  const eAttrDomain domain = eAttrDomain(node.custom1);
  const GeometryNodeScaleElementsMode scale_mode = GeometryNodeScaleElementsMode(node.custom2);

  GeometrySet geometry = params.extract_input<GeometrySet>("Geometry");

  Field<bool> selection_field = params.get_input<Field<bool>>("Selection");
  Field<float> scale_field = params.get_input<Field<float>>("Scale");
  Field<float3> center_field = params.get_input<Field<float3>>("Center");
  Field<float3> axis_field;
  if (scale_mode == GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS) {
    axis_field = params.get_input<Field<float3>>("Axis");
  }

  geometry.modify_geometry_sets([&](GeometrySet &geometry) {
    if (Mesh *mesh = geometry.get_mesh_for_write()) {
      switch (domain) {
        case ATTR_DOMAIN_FACE: {
          switch (scale_mode) {
            case GEO_NODE_SCALE_ELEMENTS_UNIFORM: {
              scale_faces_uniformly(*mesh, {selection_field, scale_field, center_field});
              break;
            }
            case GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS: {
              scale_faces_on_axis(*mesh, {selection_field, scale_field, center_field, axis_field});
              break;
            }
          }
          break;
        }
        case ATTR_DOMAIN_EDGE: {
          switch (scale_mode) {
            case GEO_NODE_SCALE_ELEMENTS_UNIFORM: {
              scale_edges_uniformly(*mesh, {selection_field, scale_field, center_field});
              break;
            }
            case GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS: {
              scale_edges_on_axis(*mesh, {selection_field, scale_field, center_field, axis_field});
              break;
            }
          }
          break;
        }
        default:
          BLI_assert_unreachable();
          break;
      }
    }
  });

  params.set_output("Geometry", std::move(geometry));
}

static void node_rna(StructRNA *srna)
{
  static const EnumPropertyItem domain_items[] = {
      {ATTR_DOMAIN_FACE,
       "FACE",
       ICON_NONE,
       "Face",
       "Scale individual faces or neighboring face islands"},
      {ATTR_DOMAIN_EDGE,
       "EDGE",
       ICON_NONE,
       "Edge",
       "Scale individual edges or neighboring edge islands"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static const EnumPropertyItem scale_mode_items[] = {
      {GEO_NODE_SCALE_ELEMENTS_UNIFORM,
       "UNIFORM",
       ICON_NONE,
       "Uniform",
       "Scale elements by the same factor in every direction"},
      {GEO_NODE_SCALE_ELEMENTS_SINGLE_AXIS,
       "SINGLE_AXIS",
       ICON_NONE,
       "Single Axis",
       "Scale elements in a single direction"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "Element type to transform",
                    domain_items,
                    NOD_inline_enum_accessors(custom1),
                    ATTR_DOMAIN_FACE);

  RNA_def_node_enum(
      srna, "scale_mode", "Scale Mode", "", scale_mode_items, NOD_inline_enum_accessors(custom2));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SCALE_ELEMENTS, "Scale Elements", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_scale_elements_cc
