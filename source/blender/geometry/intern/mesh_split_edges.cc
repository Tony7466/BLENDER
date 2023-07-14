/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <variant>

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.h"

#include "GEO_mesh_split_edges.hh"

namespace blender::geometry {

static void add_new_vertices(Mesh &mesh, const Span<int> new_to_old_verts_map)
{
  /* These types aren't supported for interpolation below. */
  CustomData_free_layers(&mesh.vdata, CD_SHAPEKEY, mesh.totvert);
  CustomData_free_layers(&mesh.vdata, CD_CLOTH_ORCO, mesh.totvert);
  CustomData_free_layers(&mesh.vdata, CD_MVERT_SKIN, mesh.totvert);
  CustomData_realloc(&mesh.vdata, mesh.totvert, mesh.totvert + new_to_old_verts_map.size());
  mesh.totvert += new_to_old_verts_map.size();

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  for (const bke::AttributeIDRef &id : attributes.all_ids()) {
    if (attributes.lookup_meta_data(id)->domain != ATTR_DOMAIN_POINT) {
      continue;
    }
    bke::GSpanAttributeWriter attribute = attributes.lookup_for_write_span(id);
    if (!attribute) {
      continue;
    }

    bke::attribute_math::gather(attribute.span,
                                new_to_old_verts_map,
                                attribute.span.take_back(new_to_old_verts_map.size()));

    attribute.finish();
  }
  if (float3 *orco = static_cast<float3 *>(
          CustomData_get_layer_for_write(&mesh.vdata, CD_ORCO, mesh.totvert)))
  {
    array_utils::gather(Span(orco, mesh.totvert),
                        new_to_old_verts_map,
                        MutableSpan(orco, mesh.totvert).take_back(new_to_old_verts_map.size()));
  }
  if (int *orig_indices = static_cast<int *>(
          CustomData_get_layer_for_write(&mesh.vdata, CD_ORIGINDEX, mesh.totvert)))
  {
    array_utils::gather(
        Span(orig_indices, mesh.totvert),
        new_to_old_verts_map,
        MutableSpan(orig_indices, mesh.totvert).take_back(new_to_old_verts_map.size()));
  }
}

static void propagate_edge_attributes(
    Mesh &mesh,
    const int new_edges_num,
    const IndexMask &unselected_edges,
    const Span<int> old_edge_indices,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();

  /* Store a copy of the IDs locally since we will remove the existing attributes which
   * can also free the names, since the API does not provide pointer stability. */
  Vector<std::string> named_ids;
  Vector<bke::AnonymousAttributeIDPtr> anonymous_ids;
  for (const bke::AttributeIDRef &id : attributes.all_ids()) {
    if (attributes.lookup_meta_data(id)->domain != ATTR_DOMAIN_EDGE) {
      continue;
    }
    if (id.is_anonymous() && !propagation_info.propagate(id.anonymous_id())) {
      continue;
    }
    if (!id.is_anonymous()) {
      if (id.name() != ".edge_verts") {
        named_ids.append(id.name());
      }
    }
    else {
      anonymous_ids.append(&id.anonymous_id());
      id.anonymous_id().add_user();
    }
  }
  Vector<bke::AttributeIDRef> local_edge_ids;
  for (const StringRef name : named_ids) {
    local_edge_ids.append(name);
  }
  for (const bke::AnonymousAttributeIDPtr &id : anonymous_ids) {
    local_edge_ids.append(*id);
  }

  /* Build new arrays for the copied edge attributes. Unlike vertices, new edges aren't all at the
   * end of the array, so just copying to the new edges would overwrite old values when they were
   * still needed. */
  struct NewAttributeData {
    const bke::AttributeIDRef &local_id;
    const CPPType &type;
    void *array;
  };
  Vector<NewAttributeData> dst_attributes;
  for (const bke::AttributeIDRef &local_id : local_edge_ids) {
    const GVArray src = *attributes.lookup(local_id);
    if (!src) {
      continue;
    }

    const CPPType &type = src.type();
    void *new_data = MEM_malloc_arrayN(new_edges_num, type.size(), __func__);
    GMutableSpan dst(type, new_data, new_edges_num);

    array_utils::gather(src, unselected_edges, dst.take_front(unselected_edges.size()));
    bke::attribute_math::gather(src, old_edge_indices, dst.take_back(old_edge_indices.size()));

    /* Free the original attribute as soon as possible to lower peak memory usage. */
    attributes.remove(local_id);
    dst_attributes.append({local_id, type, new_data});
  }

  int *new_orig_indices = nullptr;
  if (const int *orig_indices = static_cast<const int *>(
          CustomData_get_layer(&mesh.edata, CD_ORIGINDEX)))
  {
    const Span src(orig_indices, mesh.totedge);
    new_orig_indices = static_cast<int *>(MEM_malloc_arrayN(new_edges_num, sizeof(int), __func__));
    MutableSpan dst(new_orig_indices, new_edges_num);
    array_utils::gather(src, unselected_edges, dst.take_front(unselected_edges.size()));
    array_utils::gather(src, old_edge_indices, dst.take_back(old_edge_indices.size()));
  }

  CustomData_free(&mesh.edata, mesh.totedge);
  mesh.totedge = new_edges_num;

  if (new_orig_indices != nullptr) {
    CustomData_add_layer_with_data(
        &mesh.edata, CD_ORIGINDEX, new_orig_indices, mesh.totedge, nullptr);
  }

  for (NewAttributeData &new_data : dst_attributes) {
    attributes.add(new_data.local_id,
                   ATTR_DOMAIN_EDGE,
                   bke::cpp_type_to_custom_data_type(new_data.type),
                   bke::AttributeInitMoveArray(new_data.array));
  }
}

/** A vertex is selected if it's used by a selected edge. */
static IndexMask vert_selection_from_edge(const Span<int2> edges,
                                          const IndexMask &selected_edges,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  Array<bool> array(verts_num, false);
  selected_edges.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
    array[edges[i][0]] = true;
    array[edges[i][1]] = true;
  });
  return IndexMask::from_bools(array, memory);
}

static BitVector<> selection_to_bit_vector(const IndexMask &selection, const int universe_size)
{
  BitVector<> bits(universe_size);
  selection.to_bits(bits);
  return bits;
}

static bool array_contains_non_zero(const Span<int> span)
{
  return std::all_of(span.begin(), span.end(), [](const int a) { return a == 0; });
}

static int first_index_of_non_zero(const Span<int> span)
{
  for (const int i : span.index_range()) {
    if (span[i] != 0) {
      return i;
    }
  }
  BLI_assert_unreachable();
  return -1;
}

static int adjacent_corner(const Span<int> corner_verts,
                           const int corner,
                           const IndexRange poly,
                           const int vert)
{
  return corner_verts[corner] == vert ? bke::mesh::poly_corner_prev(poly, corner) :
                                        bke::mesh::poly_corner_next(poly, corner);
}

using CornerFan = Vector<int, 2>;
using SplitLooseEdgeFan = int;
using LooseEdgeGroupfan = Vector<int, 2>;

using Fan = std::variant<CornerFan, SplitLooseEdgeFan, LooseEdgeGroupfan>;

//     struct Vector<Fan> {
//   Vector<CornerFan> corner_fans;
//   /** Each split loose edge is its own "fan" made up of just that edge. */
//   Vector<int> split_loose_edges;
//   /** All non split loose edges can be attached to the same vertex in the final mesh. */
//   Vector<int> non_split_loose_edges;
// };

static CornerFan gather_corner_fan(const OffsetIndices<int> polys,
                                   const Span<int> corner_verts,
                                   const Span<int> corner_edges,
                                   const GroupedSpan<int> edge_to_corner_map,
                                   const Span<int> corner_to_poly_map,
                                   const Span<int> connected_edges,
                                   const BitSpan split_edges,
                                   const int vert,
                                   const int start_corner,
                                   MutableSpan<int> users_left)
{
  CornerFan fan;

  Vector<int> corners_to_follow({start_corner});

  auto gather_corners_connected_to_edge = [&](const int edge) {  // TODO: Use disjoint set?
    BLI_assert(edge_to_corner_map[edge].size() >= 1);
    for (const int corner : edge_to_corner_map[edge]) {
      const int poly = corner_to_poly_map[corner];
      const int next_corner = adjacent_corner(corner_verts, corner, polys[poly], vert);
      const int next_edge = corner_edges[next_corner];
      const int next = connected_edges.first_index(next_edge);
      if (users_left[next] == 0) {
        continue;
      }
      corners_to_follow.append(next_corner);
    }
  };

  gather_corners_connected_to_edge(corner_edges[start_corner]);

  while (!corners_to_follow.is_empty()) {
    const int corner = corners_to_follow.pop_last();
    const int edge = corner_edges[corner];
    const int current = connected_edges.first_index(edge);

    if (corner_verts[corner] == vert) {
      fan.append(corner);
    }
    users_left[current]--;

    if (!split_edges[edge]) {
      gather_corners_connected_to_edge(edge);
    }
  }
  return fan;
}

static Vector<Fan> calc_fans_for_vert(const OffsetIndices<int> polys,
                                      const Span<int> corner_verts,
                                      const Span<int> corner_edges,
                                      const BitSpan loose_edges,
                                      const GroupedSpan<int> edge_to_corner_map,
                                      const Span<int> corner_to_poly_map,
                                      const BitSpan split_edges,
                                      const Span<int> connected_edges,
                                      const int vert)
{
  BLI_assert(!connected_edges.is_empty());

  Array<int, 8> users_left(connected_edges.size());
  for (const int i : users_left.index_range()) {
    const int edge = connected_edges[i];
    users_left[i] = edge_to_corner_map[edge].size();
  }

  Vector<Fan> fans;

  if (!loose_edges.is_empty()) {
    Vector<int> non_split_loose_edges;
    for (const int i : connected_edges.index_range()) {
      const int edge = connected_edges[i];
      if (loose_edges[edge]) {
        users_left[i]--;
        if (split_edges[edge]) {
          fans.append_as(SplitLooseEdgeFan(edge));
        }
        else {
          non_split_loose_edges.append(edge);
        }
      }
    }
    if (!non_split_loose_edges.is_empty()) {
      fans.append_as(LooseEdgeGroupfan(std::move(non_split_loose_edges)));
    }
  }

  while (!array_contains_non_zero(users_left)) {
    const int start = first_index_of_non_zero(users_left);
    const int start_edge = connected_edges[start];
    const int start_corner = edge_to_corner_map[start_edge].first();
    CornerFan fan = gather_corner_fan(polys,
                                      corner_verts,
                                      corner_edges,
                                      edge_to_corner_map,
                                      corner_to_poly_map,
                                      connected_edges,
                                      split_edges,
                                      vert,
                                      start_corner,
                                      users_left);
    fans.append_as(std::move(fan));
  }

  return fans;
}

/* Calculate groups of edges that are contiguously connected to each input vertex. */
static Array<Vector<Fan>> calc_all_vert_fans(const OffsetIndices<int> polys,
                                             const Span<int> corner_verts,
                                             const Span<int> corner_edges,
                                             const BitSpan loose_edges,
                                             const GroupedSpan<int> vert_to_edge_map,
                                             const GroupedSpan<int> edge_to_corner_map,
                                             const Span<int> corner_to_poly_map,
                                             const BitSpan split_edges,
                                             const IndexMask &affected_verts)
{
  Array<Vector<Fan>> vert_fans(affected_verts.size()); /* TODO: Use NoInitialization() */
  affected_verts.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
    vert_fans[mask] = calc_fans_for_vert(polys,
                                         corner_verts,
                                         corner_edges,
                                         loose_edges,
                                         edge_to_corner_map,
                                         corner_to_poly_map,
                                         split_edges,
                                         vert_to_edge_map[vert],
                                         vert);
  });
  return vert_fans;
}

static OffsetIndices<int> calc_vert_ranges_per_old_vert(const IndexMask &affected_verts,
                                                        const Span<Vector<Fan>> &vert_fans,
                                                        Array<int> &offset_data)
{
  offset_data.reinitialize(affected_verts.size() + 1);
  threading::parallel_for(affected_verts.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      const Vector<Fan> &fans = vert_fans[i];
      /* Reuse the original vertex for the last fan. */
      offset_data[i] = fans.size() - 1;
    }
  });
  return offset_indices::accumulate_counts_to_offsets(offset_data);
}

static Array<int> calc_updated_corner_verts(const int orig_verts_num,
                                            const Span<int> orig_corner_verts,
                                            const Span<Vector<Fan>> &vert_fans,
                                            const OffsetIndices<int> new_verts_by_affected_vert)
{
  Array<int> new_corner_verts(orig_corner_verts.size());
  new_corner_verts.as_mutable_span().copy_from(orig_corner_verts);

  /* Update corner verts so that each fan of edges gets its own vertex. For the last "new vertex"
   * we can reuse the original vertex, which would otherwise become unused by any faces. */
  threading::parallel_for(vert_fans.index_range(), 512, [&](const IndexRange range) {
    for (const int i : range) {
      const Vector<Fan> &fans = vert_fans[i];
      const IndexRange new_verts = new_verts_by_affected_vert[i];
      for (const int i : fans.index_range().drop_back(1)) {
        if (std::holds_alternative<CornerFan>(fans[i])) {
          const Span<int> corners = std::get<CornerFan>(fans[i]);
          const int new_vert = orig_verts_num + new_verts[i];
          new_corner_verts.as_mutable_span().fill_indices(corners, new_vert);
        }
      }
    }
  });

  return new_corner_verts;
}

static VectorSet<OrderedEdge> calc_duplicate_edges(const OffsetIndices<int> polys,
                                                   const Span<int> orig_corner_edges,
                                                   const BitSpan selection,
                                                   const Span<int> new_corner_verts,
                                                   MutableSpan<int> new_corner_edges)
{
  VectorSet<OrderedEdge> duplicate_edges;
  duplicate_edges.reserve(selection.size() * 2);
  for (const int i : polys.index_range()) {
    const IndexRange poly = polys[i];
    for (const int corner : poly) {
      const int orig_edge = orig_corner_edges[corner];
      if (selection[orig_edge]) {
        const int vert_1 = new_corner_verts[corner];
        const int vert_2 = new_corner_verts[bke::mesh::poly_corner_next(poly, corner)];
        new_corner_edges[corner] = duplicate_edges.index_of_or_add_as(OrderedEdge(vert_1, vert_2));
      }
    }
  }
  return duplicate_edges;
}

/**
 * Different types of edges are placed in different chunks of the final edge output array.
 *  1. The are all of the unselected edges that aren't affected by the operation.
 *  2. Next are the (potentially duplicated) edges that come from the selected split edges.
 *  3. The final is the selected loose edges. This is a separate chunk because the duplicate edges
 *     are stored in a relatively expensive #VectorSet, and adding loose edges to that storage is
 *     pointless because they are always unique after the splitting anyway.
 */
static Array<int2> combine_all_final_edges(const Span<int2> orig_edges,
                                           const IndexMask &unselected_edges,
                                           const Span<OrderedEdge> duplicate_edges,
                                           const IndexMask &selected_loose_edges)
{
  Array<int2> final_edges(orig_edges.size() + duplicate_edges.size() +
                          selected_loose_edges.size());
  array_utils::gather(orig_edges,
                      unselected_edges,
                      final_edges.as_mutable_span().take_front(unselected_edges.size()));
  final_edges.as_mutable_span()
      .drop_front(unselected_edges.size())
      .take_front(duplicate_edges.size())
      .copy_from(duplicate_edges.cast<int2>());
  array_utils::gather(orig_edges,
                      selected_loose_edges,
                      final_edges.as_mutable_span().take_back(selected_loose_edges.size()));
  return final_edges;
}

static void swap_edge_vert(int2 &edge, const int old_vert, const int new_vert)
{
  if (edge[0] == old_vert) {
    edge[0] = new_vert;
  }
  else if (edge[1] == old_vert) {
    edge[1] = new_vert;
  }
}

/** Assign the newly created vertex duplicates to the loose edges around this vertex. */
static void reassign_loose_edge_verts(const IndexMask &affected_verts,
                                      const Span<Vector<Fan>> &vert_fans,
                                      const OffsetIndices<int> new_verts_by_affected_vert,
                                      const int selected_loose_edge_start,
                                      MutableSpan<int2> edges)
{
  affected_verts.foreach_index(GrainSize(1024), [&](const int vert, const int mask) {
    const Vector<Fan> &fans = vert_fans[mask];
    const IndexRange new_verts = new_verts_by_affected_vert[mask];
    for (const int i : fans.index_range().drop_back(1)) {
      if (std::holds_alternative<SplitLooseEdgeFan>(fans[i])) {
        const int orig_edge = std::get<SplitLooseEdgeFan>(fans[i]);
        const int new_edge = selected_loose_edge_start + 0;  // TODO!!!!
        swap_edge_vert(edges[new_edge], vert, new_verts[i]);
      }
      else if (std::holds_alternative<LooseEdgeGroupfan>(fans[i])) {
        for (const int orig_edge : std::get<LooseEdgeGroupfan>(fans[i])) {
          const int new_edge = selected_loose_edge_start + 0;  // TODO!!!!
          swap_edge_vert(edges[new_edge], vert, new_verts[i]);
        }
      }
    }
  });
}

static Array<int> calc_new_to_old_edge_map(const OffsetIndices<int> polys,
                                           const Span<int> orig_corner_edges,
                                           const Span<int> new_corner_edges,
                                           const int new_edges_num)
{
  Array<int> new_to_old_edge_map(new_edges_num);
  for (const int i : polys.index_range()) {
    const IndexRange poly = polys[i];
    for (const int corner : poly) {
      const int new_edge_i = new_corner_edges[corner];
      const int old_edge_i = orig_corner_edges[corner];
      new_to_old_edge_map[new_edge_i] = old_edge_i;
    }
  }
  return new_to_old_edge_map;
}

static Array<int> calc_new_to_old_vert_map(const IndexMask &affected_verts,
                                           const OffsetIndices<int> new_verts_by_affected_vert)
{
  Array<int> map(new_verts_by_affected_vert.total_size());
  affected_verts.foreach_index(GrainSize(1024), [&](const int vert, const int mask) {
    map.as_mutable_span().slice(new_verts_by_affected_vert[mask]).fill(vert);
  });
  return map;
}

void split_edges(Mesh &mesh,
                 const IndexMask &selected_edges,
                 const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const int orig_verts_num = mesh.totvert;
  const Span<int2> orig_edges = mesh.edges();
  const OffsetIndices polys = mesh.polys();
  const Span<int> orig_corner_verts = mesh.corner_verts();
  const Span<int> orig_corner_edges = mesh.corner_edges();

  Array<int> vert_to_edge_offsets;
  Array<int> vert_to_edge_indices;
  const GroupedSpan<int> vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
      orig_edges, orig_verts_num, vert_to_edge_offsets, vert_to_edge_indices);

  Array<int> edge_to_corner_offsets;
  Array<int> edge_to_corner_indices;
  const GroupedSpan<int> edge_to_corner_map = bke::mesh::build_edge_to_loop_map(
      orig_corner_edges, orig_edges.size(), edge_to_corner_offsets, edge_to_corner_indices);

  const Array<int> corner_to_poly_map = bke::mesh::build_loop_to_poly_map(mesh.polys());

  IndexMaskMemory memory;
  const IndexMask unselected_edges = selected_edges.complement(orig_edges.index_range(), memory);
  const IndexMask affected_verts = vert_selection_from_edge(
      orig_edges, selected_edges, orig_verts_num, memory);
  const BitVector<> selection_bits = selection_to_bit_vector(selected_edges, orig_edges.size());
  const bke::LooseEdgeCache &loose_edge_cache = mesh.loose_edges();
  IndexMask selected_loose_edges;
  if (loose_edge_cache.count > 0) {
    selected_loose_edges = IndexMask::from_bits(
        selected_edges, loose_edge_cache.is_loose_bits, memory);
  }

  const Array<Vector<Fan>> vert_fans = calc_all_vert_fans(polys,
                                                          orig_corner_verts,
                                                          orig_corner_edges,
                                                          loose_edge_cache.is_loose_bits,
                                                          vert_to_edge_map,
                                                          edge_to_corner_map,
                                                          corner_to_poly_map,
                                                          selection_bits,
                                                          affected_verts);

  Array<int> vert_new_vert_offset_data;
  const OffsetIndices new_verts_by_affected_vert = calc_vert_ranges_per_old_vert(
      affected_verts, vert_fans, vert_new_vert_offset_data);

  Array<int> new_corner_verts = calc_updated_corner_verts(
      orig_verts_num, orig_corner_verts, vert_fans, new_verts_by_affected_vert);

  /* Create new edges. */
  Array<int> new_corner_edges(orig_corner_edges.size());
  new_corner_edges.as_mutable_span().copy_from(orig_corner_edges);
  VectorSet<OrderedEdge> duplicate_edges = calc_duplicate_edges(
      polys, orig_corner_edges, selection_bits, new_corner_verts, new_corner_edges);

  Array<int2> result_edges = combine_all_final_edges(
      orig_edges, unselected_edges, duplicate_edges, selected_loose_edges);
  if (loose_edge_cache.count > 0) {
    reassign_loose_edge_verts(affected_verts,
                              vert_fans,
                              new_verts_by_affected_vert,
                              0,  // TODO
                              result_edges);
  }

  const Array<int> new_to_old_edge_map = calc_new_to_old_edge_map(
      polys, orig_corner_edges, new_corner_edges, result_edges.size());

  const Array<int> new_to_old_vert_map = calc_new_to_old_vert_map(affected_verts,
                                                                  new_verts_by_affected_vert);

  /* Resize the mesh to add the new vertices and rebuild the edges. */
  add_new_vertices(mesh, new_to_old_vert_map);
  propagate_edge_attributes(
      mesh, result_edges.size(), unselected_edges, new_to_old_edge_map, propagation_info);
  mesh.attributes_for_write().add<int2>(
      ".edge_verts",
      ATTR_DOMAIN_EDGE,
      bke::AttributeInitVArray(VArray<int2>::ForSpan(result_edges)));

  BKE_mesh_tag_edges_split(&mesh);
}

}  // namespace blender::geometry
