/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <variant>

#include "BLI_array_utils.hh"
#include "BLI_disjoint_set.hh"
#include "BLI_index_mask.hh"
#include "BLI_ordered_edge.hh"
#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.h"

#include "GEO_mesh_split_edges.hh"

namespace blender::geometry {

/** Stores where in the output edge domain each "type" of edge should be stored. */
struct ResultEdgeRanges {
  int total_size;
  /** All of the unselected edges that aren't affected by the operation. */
  IndexRange unselected;
  /** The (potentially duplicated) edges that come from the selected split edges. */
  IndexRange duplicate;
  /**
   * Selected loose edges, separate from the "duplicate" edges because loose edges will always
   * be unique in the result anyway, and aren't the mapping isn't tracked with the corner edges
   * arrays.
   */
  IndexRange loose;
};

struct ResultEdgeMaps {
  /** Original indices of unaffected edges. */
  const IndexMask &unselected;
  /** Map from new duplicate edge indices (within their range) to original edges. */
  Span<int> duplicate;
  /* Original indices of selected loose edges. */
  const IndexMask &loose;
};

static void propagate_vert_attributes(Mesh &mesh, const Span<int> new_to_old_verts_map)
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
    const ResultEdgeRanges &ranges,
    const ResultEdgeMaps &maps,
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

  const int new_edges_num = ranges.total_size;

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

    array_utils::gather(src, maps.unselected, dst.slice(ranges.unselected));
    bke::attribute_math::gather(src, maps.duplicate, dst.slice(ranges.duplicate));
    array_utils::gather(src, maps.loose, dst.slice(ranges.loose));

    /* Free the original attribute as soon as possible to lower peak memory usage. */
    attributes.remove(local_id);
    dst_attributes.append({local_id, type, new_data});
  }

  int *new_orig_indices = nullptr;
  if (const int *orig_indices = static_cast<const int *>(
          CustomData_get_layer(&mesh.edata, CD_ORIGINDEX)))
  {
    new_orig_indices = static_cast<int *>(MEM_malloc_arrayN(new_edges_num, sizeof(int), __func__));
    const Span src(orig_indices, mesh.totedge);
    MutableSpan dst(new_orig_indices, new_edges_num);
    array_utils::gather(src, maps.unselected, dst.slice(ranges.unselected));
    bke::attribute_math::gather(src, maps.duplicate, dst.slice(ranges.duplicate));
    array_utils::gather(src, maps.loose, dst.slice(ranges.loose));
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

static int corner_on_edge_connected_to_vert(const Span<int> corner_verts,
                                            const int corner,
                                            const IndexRange face,
                                            const int vert)
{
  if (corner_verts[corner] == vert) {
    return corner;
  }
  const int other = bke::mesh::face_corner_prev(face, corner);
  BLI_assert(corner_verts[other] == vert);
  return other;
}

using CornerFan = Vector<int>;

/**
 * A corner fan is a group of corners bordered by boundary edges or split vertices. We store corner
 * indices instead of edge indices because later on in the algorithm we only relink the
 * `corner_vert` array to each fan's new vertices. The edges are built in a separate step.
 *
 * Find all groups of corners using this vertex that are reachable by crossing over connected faces
 * at non-split edges.
 */
static Vector<CornerFan> calc_corner_fans_for_vertex(const OffsetIndices<int> faces,
                                                     const Span<int> corner_verts,
                                                     const Span<int> corner_edges,
                                                     const GroupedSpan<int> edge_to_corner_map,
                                                     const Span<int> corner_to_face_map,
                                                     const BitSpan split_edges,
                                                     const Span<int> connected_corners,
                                                     const int vert)
{
  Vector<CornerFan> fans;
  BitVector<> used_corners(connected_corners.size());

  for (const int corner : connected_corners) {
    const int i = connected_corners.first_index(corner);
    if (used_corners[i]) {
      continue;
    }
    CornerFan fan;
    used_corners[i].set();
    fan.append(corner);

    const IndexRange face = faces[corner_to_face_map[corner]];
    const int prev_corner = bke::mesh::face_corner_prev(face, corner);
    for (const int edge : {corner_edges[corner], corner_edges[prev_corner]}) {
      if (split_edges[edge]) {
        continue;
      }
      for (const int other_corner : edge_to_corner_map[edge]) {
        if (other_corner == corner) {
          continue;
        }
        const IndexRange other_face = faces[corner_to_face_map[other_corner]];
        const int neighbor_corner = corner_on_edge_connected_to_vert(
            corner_verts, other_corner, other_face, vert);

        fan.append(neighbor_corner);
        used_corners[connected_corners.first_index(neighbor_corner)].set();
      }
    }
    fans.append(std::move(fan));
  }

  return fans;
}

/* Calculate groups of edges that are contiguously connected to each input vertex. */
static Array<Vector<CornerFan>> calc_all_corner_fans(const OffsetIndices<int> faces,
                                                     const Span<int> corner_verts,
                                                     const Span<int> corner_edges,
                                                     const GroupedSpan<int> vert_to_corner_map,
                                                     const GroupedSpan<int> edge_to_corner_map,
                                                     const Span<int> corner_to_face_map,
                                                     const BitSpan split_edges,
                                                     const IndexMask &affected_verts)
{
  Array<Vector<Vector<int>>> corner_fans(affected_verts.size()); /* TODO: Use NoInitialization() */
  affected_verts.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
    corner_fans[mask] = calc_corner_fans_for_vertex(faces,
                                                    corner_verts,
                                                    corner_edges,
                                                    edge_to_corner_map,
                                                    corner_to_face_map,
                                                    split_edges,
                                                    vert_to_corner_map[vert],
                                                    vert);
  });
  return corner_fans;
}

struct LooseEdgeInfo {
  GroupedSpan<int> vert_to_edge_map;
  BitSpan loose_edges;
};

struct VertLooseEdges {
  Vector<int> split;
  Vector<int> unselected;
};

static VertLooseEdges calc_vert_loose_edges(const LooseEdgeInfo &loose_edge_info,
                                            const BitSpan split_edges,
                                            const int vert)
{
  VertLooseEdges info;
  for (const int edge : loose_edge_info.vert_to_edge_map[vert]) {
    if (loose_edge_info.loose_edges[edge]) {
      if (split_edges[edge]) {
        info.split.append(edge);
      }
      else {
        info.unselected.append(edge);
      }
    }
  }
  return info;
}

/**
 * Every affected vertex maps to potentially multiple output vertices. Create a mapping from
 * affected vertex index to the of output vertex (indices are to those sets, not indices in arrays
 * of _all_ vertices). For every original vertex, reuse the original vertex for the first of:
 *  1. The last face corner fan
 *  2. The last split loose edge
 *  3. The group of non-selected loose edges
 * Using this order prioritizes the simplicity of the no-loose-edge case, which we assume is
 * more common.
 */
static OffsetIndices<int> calc_vert_ranges_per_old_vert(const IndexMask &affected_verts,
                                                        const Span<Vector<CornerFan>> corner_fans,
                                                        const LooseEdgeInfo *loose_edge_info,
                                                        const BitSpan split_edges,
                                                        Array<int> &offset_data)
{
  offset_data.reinitialize(affected_verts.size() + 1);
  threading::parallel_for(affected_verts.index_range(), 2048, [&](const IndexRange range) {
    offset_data.as_mutable_span().slice(range).fill(-1);
    for (const int i : range) {
      offset_data[i] += corner_fans[i].size();
    }
  });
  if (loose_edge_info) {
    affected_verts.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
      const VertLooseEdges info = calc_vert_loose_edges(*loose_edge_info, split_edges, vert);
      offset_data[mask] += info.split.size();
      offset_data[mask] += info.unselected.size() > 0;
    });
  }
  return offset_indices::accumulate_counts_to_offsets(offset_data);
}

/**
 * Update corner verts so that each fan of edges gets its own vertex. For the last "new vertex" we
 * can reuse the original vertex, which would otherwise become unused by any faces. The loose edge
 * case will have to deal with this later.
 */
static void calc_updated_corner_verts(const int orig_verts_num,
                                      const Span<Vector<CornerFan>> corner_fans,
                                      const OffsetIndices<int> new_verts_by_affected_vert,
                                      MutableSpan<int> new_corner_verts)
{
  threading::parallel_for(corner_fans.index_range(), 512, [&](const IndexRange range) {
    for (const int new_vert : range) {
      const Span<CornerFan> fans = corner_fans[new_vert];
      const IndexRange new_verts = new_verts_by_affected_vert[new_vert];
      for (const int fan : fans.index_range().drop_back(1)) {
        const int new_vert = orig_verts_num + new_verts[fan];
        new_corner_verts.fill_indices(fans[fan].as_span(), new_vert);
      }
    }
  });
}

/**
 * When each of an edge's vertices only have a single grouped edge fan, the vertices will not be
 * split, meaning all output edges will reuse the same vertices. Because edges must be unique, they
 * must be deduplicated. Though smarter heuristics may be possible given the available information,
 * using a #VectorSet is a foolproof way to do this deduplication.
 */
static VectorSet<OrderedEdge> calc_duplicate_edges(const OffsetIndices<int> faces,
                                                   const Span<int> orig_corner_edges,
                                                   const BitSpan selection,
                                                   const Span<int> new_corner_verts,
                                                   const int duplicate_edge_start,
                                                   MutableSpan<int> new_corner_edges)
{
  VectorSet<OrderedEdge> duplicate_edges;
  duplicate_edges.reserve(selection.size());
  for (const int i : faces.index_range()) {
    const IndexRange face = faces[i];
    for (const int corner : face) {
      const int orig_edge = orig_corner_edges[corner];
      if (selection[orig_edge]) {
        const int vert_1 = new_corner_verts[corner];
        const int vert_2 = new_corner_verts[bke::mesh::face_corner_next(face, corner)];
        const int duplicate_i = duplicate_edges.index_of_or_add_as(OrderedEdge(vert_1, vert_2));
        new_corner_edges[corner] = duplicate_edge_start + duplicate_i;
      }
    }
  }
  return duplicate_edges;
}

static ResultEdgeRanges get_result_edge_ranges(const int unselected_num,
                                               const int duplicate_num,
                                               const int loose_num)
{
  const IndexRange all(unselected_num + duplicate_num + loose_num);
  ResultEdgeRanges ranges;
  ranges.total_size = all.size();
  ranges.unselected = all.take_front(unselected_num);
  ranges.duplicate = all.drop_front(unselected_num).take_front(duplicate_num);
  ranges.loose = all.take_back(loose_num);
  return ranges;
}

static void combine_result_edges(const Span<int2> orig_edges,
                                 const ResultEdgeRanges &ranges,
                                 const ResultEdgeMaps &maps,
                                 const Span<OrderedEdge> duplicate_edges,
                                 MutableSpan<int2> result_edges)
{
  array_utils::gather(orig_edges, maps.unselected, result_edges.slice(ranges.unselected));
  result_edges.slice(ranges.duplicate).copy_from(duplicate_edges.cast<int2>());
  array_utils::gather(orig_edges, maps.loose, result_edges.slice(ranges.loose));
}

static void create_reverse_map(const IndexMask &mask, MutableSpan<int> r_map)
{
#ifdef DEBUG
  r_map.fill(-1);
#endif
  mask.foreach_index_optimized<int>(
      GrainSize(4096), [&](const int src_i, const int dst_i) { r_map[src_i] = dst_i; });
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

/**
 * Assign the newly created vertex duplicates to the loose edges around this vertex. Every split
 * loose edge is reattached to a newly created vertex. If there are non-split loose edges attached
 * to the vertex, they all reuse the original vertex.
 */
static void reassign_loose_edge_verts(const int orig_verts_num,
                                      const int orig_edges_num,
                                      const IndexMask &affected_verts,
                                      const LooseEdgeInfo &loose_edge_info,
                                      const BitSpan split_edges,
                                      const Span<Vector<CornerFan>> corner_fans,
                                      const OffsetIndices<int> new_verts_by_affected_vert,
                                      const ResultEdgeRanges &ranges,
                                      const IndexMask &result_loose_map,
                                      MutableSpan<int2> edges)
{
  /* This map is only useful because loose edges are not duplicated. Non-loose edges can
   * potentially be duplicated into multiple final edges. In the end this is equivalent
   * to using O(logn) lookup of the index in the index mask, but should be faster. */
  Array<int> old_to_new_edge_map(orig_edges_num);
  create_reverse_map(result_loose_map, old_to_new_edge_map);

  affected_verts.foreach_index(GrainSize(1024), [&](const int vert, const int mask) {
    /* Account for the reuse of the original vertex by non-loose edge fans. */
    int new_vert_i = std::max<int>(corner_fans[mask].size() - 1, 0);
    const IndexRange new_verts = new_verts_by_affected_vert[mask];
    const VertLooseEdges vert_info = calc_vert_loose_edges(loose_edge_info, split_edges, vert);

    for (const int orig_edge : vert_info.split) {
      const int new_vert = orig_verts_num + new_verts[new_vert_i];
      const int new_edge = ranges.loose[old_to_new_edge_map[orig_edge]];
      swap_edge_vert(edges[new_edge], vert, new_vert);
      new_vert_i++;
      if (new_vert_i == new_verts.size()) {
        break;
      }
    }
    if (new_vert_i == new_verts.size() - 1) {
      const int new_vert = orig_verts_num + new_verts[new_vert_i];
      for (const int orig_edge : vert_info.unselected) {
        swap_edge_vert(edges[orig_edge], vert, new_vert);
      }
    }
  });
}

/**
 * Using the original corner edge array and the updated one, calculate the mapping from duplicate
 * edges to original edges.
 */
static Array<int> calc_duplicate_to_old_edge_map(const OffsetIndices<int> faces,
                                                 const Span<int> orig_corner_edges,
                                                 const BitSpan selection,
                                                 const Span<int> new_corner_edges,
                                                 const IndexRange duplicates)
{
  Array<int> duplicate_to_old_edge_map(duplicates.size());
  for (const int i : faces.index_range()) {
    const IndexRange face = faces[i];
    for (const int corner : face) {
      const int old_edge = orig_corner_edges[corner];
      if (selection[old_edge]) {
        const int new_edge = new_corner_edges[corner];
        const int duplicate = new_edge - duplicates.start();
        duplicate_to_old_edge_map[duplicate] = old_edge;
      }
    }
  }
  return duplicate_to_old_edge_map;
}

/**
 * Transform the #OffsetIndices storage of new vertices per source vertex into a more
 * standard index map which can be used with existing utilities to copy vertex attributes.
 */
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
  const OffsetIndices faces = mesh.faces();
  const Array<int> orig_corner_edges = mesh.corner_edges();

  IndexMaskMemory memory;
  const IndexMask unselected_edges = selected_edges.complement(orig_edges.index_range(), memory);
  const IndexMask affected_verts = vert_selection_from_edge(
      orig_edges, selected_edges, orig_verts_num, memory);
  const BitVector<> selection_bits = selection_to_bit_vector(selected_edges, orig_edges.size());
  const bke::LooseEdgeCache &loose_edges = mesh.loose_edges();
  const IndexMask selected_loose_edges = loose_edges.count > 0 ?
                                             IndexMask::from_bits(selected_edges,
                                                                  loose_edges.is_loose_bits,
                                                                  memory) :
                                             IndexMask();

  Array<int> vert_to_corner_offsets;
  Array<int> vert_to_corner_indices;
  const GroupedSpan<int> vert_to_corner_map = bke::mesh::build_vert_to_loop_map(
      mesh.corner_verts(), orig_verts_num, vert_to_corner_offsets, vert_to_corner_indices);

  Array<int> edge_to_corner_offsets;
  Array<int> edge_to_corner_indices;
  const GroupedSpan<int> edge_to_corner_map = bke::mesh::build_edge_to_loop_map(
      orig_corner_edges, orig_edges.size(), edge_to_corner_offsets, edge_to_corner_indices);

  Array<int> vert_to_edge_offsets;
  Array<int> vert_to_edge_indices;
  GroupedSpan<int> vert_to_edge_map;
  if (mesh.loose_edges().count > 0) {
    vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        orig_edges, orig_verts_num, vert_to_edge_offsets, vert_to_edge_indices);
  }
  const LooseEdgeInfo loose_edge_info{vert_to_edge_map, loose_edges.is_loose_bits};

  const Array<int> corner_to_face_map = bke::mesh::build_loop_to_face_map(mesh.faces());

  const Array<Vector<CornerFan>> corner_fans = calc_all_corner_fans(faces,
                                                                    mesh.corner_verts(),
                                                                    orig_corner_edges,
                                                                    vert_to_corner_map,
                                                                    edge_to_corner_map,
                                                                    corner_to_face_map,
                                                                    selection_bits,
                                                                    affected_verts);

  Array<int> vert_new_vert_offset_data;
  const OffsetIndices new_verts_by_affected_vert = calc_vert_ranges_per_old_vert(
      affected_verts,
      corner_fans,
      mesh.loose_edges().count == 0 ? nullptr : &loose_edge_info,
      selection_bits,
      vert_new_vert_offset_data);

  MutableSpan<int> new_corner_verts = mesh.corner_verts_for_write();
  calc_updated_corner_verts(
      orig_verts_num, corner_fans, new_verts_by_affected_vert, new_corner_verts);

  MutableSpan<int> new_corner_edges = mesh.corner_edges_for_write();
  VectorSet<OrderedEdge> duplicate_edges = calc_duplicate_edges(faces,
                                                                orig_corner_edges,
                                                                selection_bits,
                                                                new_corner_verts,
                                                                unselected_edges.size(),
                                                                new_corner_edges);

  const ResultEdgeRanges edge_ranges = get_result_edge_ranges(
      unselected_edges.size(), duplicate_edges.size(), selected_loose_edges.size());

  const Array<int> duplicate_edge_map = calc_duplicate_to_old_edge_map(
      faces, orig_corner_edges, selection_bits, new_corner_edges, edge_ranges.duplicate);

  const ResultEdgeMaps edge_maps{unselected_edges, duplicate_edge_map, selected_loose_edges};

  Array<int2> result_edges(edge_ranges.total_size);
  combine_result_edges(orig_edges, edge_ranges, edge_maps, duplicate_edges, result_edges);
  if (!edge_ranges.loose.is_empty()) {
    reassign_loose_edge_verts(orig_verts_num,
                              orig_edges.size(),
                              affected_verts,
                              loose_edge_info,
                              selection_bits,
                              corner_fans,
                              new_verts_by_affected_vert,
                              edge_ranges,
                              edge_maps.loose,
                              result_edges);
  }

  propagate_edge_attributes(mesh, edge_ranges, edge_maps, propagation_info);
  mesh.attributes_for_write().add<int2>(
      ".edge_verts",
      ATTR_DOMAIN_EDGE,
      bke::AttributeInitVArray(VArray<int2>::ForSpan(result_edges)));

  const Array<int> vert_map = calc_new_to_old_vert_map(affected_verts, new_verts_by_affected_vert);
  propagate_vert_attributes(mesh, vert_map);

  BKE_mesh_tag_edges_split(&mesh);

  BLI_assert(BKE_mesh_is_valid(&mesh));
}

}  // namespace blender::geometry
