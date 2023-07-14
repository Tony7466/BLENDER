/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

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

static void add_new_edges(Mesh &mesh,
                          const Span<int2> new_edges,
                          const Span<int> new_to_old_edges_map,
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
    bke::GAttributeReader attribute = attributes.lookup(local_id);
    if (!attribute) {
      continue;
    }

    const CPPType &type = attribute.varray.type();
    void *new_data = MEM_malloc_arrayN(new_edges.size(), type.size(), __func__);

    bke::attribute_math::gather(
        attribute.varray, new_to_old_edges_map, GMutableSpan(type, new_data, new_edges.size()));

    /* Free the original attribute as soon as possible to lower peak memory usage. */
    attributes.remove(local_id);
    dst_attributes.append({local_id, type, new_data});
  }

  int *new_orig_indices = nullptr;
  if (const int *orig_indices = static_cast<const int *>(
          CustomData_get_layer(&mesh.edata, CD_ORIGINDEX)))
  {
    new_orig_indices = static_cast<int *>(
        MEM_malloc_arrayN(new_edges.size(), sizeof(int), __func__));
    array_utils::gather(Span(orig_indices, mesh.totedge),
                        new_to_old_edges_map,
                        {new_orig_indices, new_edges.size()});
  }

  CustomData_free(&mesh.edata, mesh.totedge);
  mesh.totedge = new_edges.size();
  CustomData_add_layer_named(
      &mesh.edata, CD_PROP_INT32_2D, CD_CONSTRUCT, mesh.totedge, ".edge_verts");
  mesh.edges_for_write().copy_from(new_edges);

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

/** Assign the newly created vertex duplicates to the loose edges around this vertex. */
static void reassign_loose_edge_verts(const int vertex,
                                      const int start_offset,
                                      const Span<int> fans,
                                      const Span<int> fan_sizes,
                                      const BoundedBitSpan loose_edges,
                                      MutableSpan<int2> edges)
{
  int fan_start = 0;
  /* We don't need to create a new vertex for the last fan. That fan can just be connected to the
   * original vertex. */
  for (const int i : fan_sizes.index_range().drop_back(1)) {
    const int new_vert = start_offset + i;
    for (const int edge_i : fans.slice(fan_start, fan_sizes[i])) {
      if (loose_edges[edge_i]) {
        if (edges[edge_i][0] == vertex) {
          edges[edge_i][0] = new_vert;
        }
        else if (edges[edge_i][1] == vertex) {
          edges[edge_i][1] = new_vert;
        }
      }
    }
    fan_start += fan_sizes[i];
  }
}

/** A vertex is selected if it's used by a selected edge. */
static IndexMask vert_selection_from_edge(const Span<int2> edges,
                                          const IndexMask &edge_mask,
                                          const int verts_num,
                                          IndexMaskMemory &memory)
{
  Array<bool> array(verts_num, false);
  edge_mask.foreach_index_optimized<int>(GrainSize(4096), [&](const int i) {
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

/**
 * Get the index of the adjacent edge to a loop connected to a vertex. In other words, for the
 * given polygon return the unique edge connected to the given vertex and not on the given loop.
 */
static int adjacent_edge(const Span<int> corner_verts,
                         const Span<int> corner_edges,
                         const int corner,
                         const IndexRange poly,
                         const int vert)
{
  const int adjacent_loop_i = (corner_verts[corner] == vert) ?
                                  bke::mesh::poly_corner_prev(poly, corner) :
                                  bke::mesh::poly_corner_next(poly, corner);
  return corner_edges[adjacent_loop_i];
}

using VertEdgeFans = Vector<Vector<int>>;

static Vector<int> gather_edges_in_fan(const OffsetIndices<int> polys,
                                       const Span<int> corner_verts,
                                       const Span<int> corner_edges,
                                       const GroupedSpan<int> edge_to_corner_map,
                                       const Span<int> corner_to_poly_map,
                                       const Span<int> connected_edges,
                                       const BitSpan split_edges,
                                       const int vert,
                                       const int start,
                                       MutableSpan<int> users_left)
{
  Vector<int> fan;
  Vector<int> edge_stack({connected_edges[start]});
  while (!edge_stack.is_empty()) {
    const int edge = edge_stack.pop_last();
    const int current = connected_edges.first_index(edge);

    fan.append(edge);
    users_left[current]--;

    if (split_edges[edge] && current != start) {
      continue;
    }

    for (const int corner : edge_to_corner_map[edge]) {
      const int poly = corner_to_poly_map[corner];
      const int next_edge = adjacent_edge(corner_verts, corner_edges, corner, polys[poly], vert);
      const int next = connected_edges.first_index(next_edge);
      if (users_left[next] == 0) {
        continue;
      }
      edge_stack.append(next_edge);
    }
  }
  return fan;
}

/* TODO: Use thread local allocators for #VertEdgeFans memory. */
static VertEdgeFans calc_fans_for_vert(const OffsetIndices<int> polys,
                                       const Span<int> corner_verts,
                                       const Span<int> corner_edges,
                                       const GroupedSpan<int> vert_to_edge_map,
                                       const GroupedSpan<int> edge_to_corner_map,
                                       const Span<int> corner_to_poly_map,
                                       const BitSpan split_edges,
                                       const int vert)
{
  const Span<int> connected_edges = vert_to_edge_map[vert];
  if (connected_edges.size() <= 1) {
    return Vector<Vector<int>>({Vector<int>(connected_edges)});
  }

  Array<int, 8> users_left(connected_edges.size());
  for (const int i : users_left.index_range()) {
    const int edge = connected_edges[i];
    users_left[i] = edge_to_corner_map[edge].size();
  }

  Vector<Vector<int>> edge_fans;

  while (!array_contains_non_zero(users_left)) {
    const int start = first_index_of_non_zero(users_left);
    edge_fans.append(gather_edges_in_fan(polys,
                                         corner_verts,
                                         corner_edges,
                                         edge_to_corner_map,
                                         corner_to_poly_map,
                                         connected_edges,
                                         split_edges,
                                         vert,
                                         start,
                                         users_left));
  }

  return edge_fans;
}

/* Calculate groups of edges that are contiguously connected to each input vertex. */
static Array<VertEdgeFans> calc_all_vert_fans(const OffsetIndices<int> polys,
                                              const Span<int> corner_verts,
                                              const Span<int> corner_edges,
                                              const GroupedSpan<int> vert_to_edge_map,
                                              const GroupedSpan<int> edge_to_corner_map,
                                              const Span<int> corner_to_poly_map,
                                              const BitSpan split_edges,
                                              const IndexMask &vert_mask)
{
  Array<VertEdgeFans> vert_edge_fans(vert_mask.size()); /* TODO: Use NoInitialization() */
  vert_mask.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
    vert_edge_fans[mask] = calc_fans_for_vert(polys,
                                              corner_verts,
                                              corner_edges,
                                              vert_to_edge_map,
                                              edge_to_corner_map,
                                              corner_to_poly_map,
                                              split_edges,
                                              vert);
  });
  return vert_edge_fans;
}

static OffsetIndices<int> calc_vert_ranges_per_old_vert(const IndexMask &vert_mask,
                                                        const Span<VertEdgeFans> &vert_edge_fans,
                                                        Array<int> &offset_data)
{
  offset_data.reinitialize(vert_mask.size() + 1);
  threading::parallel_for(vert_mask.index_range(), 2048, [&](const IndexRange range) {
    for (const int i : range) {
      /* Reuse the original vertex for the last fan. */
      offset_data[i] = vert_edge_fans[i].size() - 1;
    }
  });
  return offset_indices::accumulate_counts_to_offsets(offset_data);
}

static Array<int> calc_updated_corner_verts(const int orig_verts_num,
                                            const Span<int> orig_corner_verts,
                                            const GroupedSpan<int> edge_to_corner_map,
                                            const IndexMask &vert_mask,
                                            const Span<VertEdgeFans> &vert_edge_fans,
                                            const OffsetIndices<int> new_verts_by_masked_vert)
{
  Array<int> new_corner_verts(orig_corner_verts.size());
  new_corner_verts.as_mutable_span().copy_from(orig_corner_verts);

  /* Update corner verts so that each fan of edges gets its own vertex. The last vertex can be
   * reused. */
  vert_mask.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
    const VertEdgeFans &vert_fans = vert_edge_fans[mask];

    int new_vert = new_verts_by_masked_vert[mask].start() + orig_verts_num;
    for (const int fan : vert_fans.index_range().drop_back(1)) {
      for (const int edge : vert_fans[fan]) {
        /* Could potentially use a vert to corner map. */
        for (const int corner : edge_to_corner_map[edge]) {
          if (new_corner_verts[corner] == vert) {
            new_corner_verts[corner] = new_vert;
            new_vert++;
          }
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
  duplicate_edges.reserve(selection.size());
  /* Could potentially use a compacted poly selection. */
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

static Array<int2> combine_all_final_edges(const Span<int2> orig_edges,
                                           const IndexMask &edge_mask_inverse,
                                           const Span<OrderedEdge> duplicate_edges)
{
  Array<int2> final_edges(orig_edges.size() + duplicate_edges.size());
  array_utils::gather(orig_edges,
                      edge_mask_inverse,
                      final_edges.as_mutable_span().take_front(edge_mask_inverse.size()));
  final_edges.as_mutable_span()
      .take_back(duplicate_edges.size())
      .copy_from(duplicate_edges.cast<int2>());
  return final_edges;
}

static Array<int> calc_new_to_old_vert_map(const IndexMask &vert_mask,
                                           const OffsetIndices<int> new_verts_by_masked_vert)
{
  Array<int> map(new_verts_by_masked_vert.total_size());
  vert_mask.foreach_index(GrainSize(1024), [&](const int vert, const int mask) {
    map.as_mutable_span().slice(new_verts_by_masked_vert[mask]).fill(vert);
  });
  return map;
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

void split_edges(Mesh &mesh,
                 const IndexMask &edge_mask,
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

  // const bke::LooseEdgeCache &loose_edges_cache = mesh.loose_edges();
  // const IndexMask loose_edges = IndexMask::from_bits(loose_edges_cache.is_loose_bits, memory);

  const IndexMask edge_mask_inverse = edge_mask.complement(orig_edges.index_range(), memory);
  const IndexMask vert_mask = vert_selection_from_edge(
      orig_edges, edge_mask, orig_verts_num, memory);
  const BitVector<> selection_bits = selection_to_bit_vector(edge_mask, orig_edges.size());

  const Array<VertEdgeFans> vert_edge_fans = calc_all_vert_fans(polys,
                                                                orig_corner_verts,
                                                                orig_corner_edges,
                                                                vert_to_edge_map,
                                                                edge_to_corner_map,
                                                                corner_to_poly_map,
                                                                selection_bits,
                                                                vert_mask);

  Array<int> vert_new_vert_offset_data;
  const OffsetIndices new_verts_by_masked_vert = calc_vert_ranges_per_old_vert(
      vert_mask, vert_edge_fans, vert_new_vert_offset_data);

  Array<int> new_corner_verts = calc_updated_corner_verts(orig_verts_num,
                                                          orig_corner_verts,
                                                          edge_to_corner_map,
                                                          vert_mask,
                                                          vert_edge_fans,
                                                          new_verts_by_masked_vert);

  /* Create new edges. */
  Array<int> new_corner_edges(orig_corner_edges.size());
  new_corner_edges.as_mutable_span().copy_from(orig_corner_edges);
  VectorSet<OrderedEdge> duplicate_edges = calc_duplicate_edges(
      polys, orig_corner_edges, selection_bits, new_corner_verts, new_corner_edges);

  Array<int2> result_edges = combine_all_final_edges(
      orig_edges, edge_mask_inverse, duplicate_edges);

  const Array<int> new_to_old_edge_map = calc_new_to_old_edge_map(
      polys, orig_corner_edges, new_corner_edges, result_edges.size());

  const Array<int> new_to_old_vert_map = calc_new_to_old_vert_map(vert_mask,
                                                                  new_verts_by_masked_vert);

  /* Resize the mesh to add the new vertices and rebuild the edges. */
  add_new_vertices(mesh, new_to_old_vert_map);
  add_new_edges(mesh, result_edges, new_to_old_edge_map, propagation_info);

  BKE_mesh_tag_edges_split(&mesh);
}

}  // namespace blender::geometry
