/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_vector_set.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_mesh.hh"
#include "BKE_mesh_mapping.h"

#include "GEO_mesh_split_edges.hh"

namespace blender::geometry {

static void propagate_vert_attributes(Mesh &mesh, const Span<int> new_to_old_verts_map)
{
  /* These types aren't supported for interpolation below. */
  CustomData_free_layers(&mesh.vert_data, CD_SHAPEKEY, mesh.totvert);
  CustomData_free_layers(&mesh.vert_data, CD_CLOTH_ORCO, mesh.totvert);
  CustomData_free_layers(&mesh.vert_data, CD_MVERT_SKIN, mesh.totvert);
  CustomData_realloc(&mesh.vert_data, mesh.totvert, mesh.totvert + new_to_old_verts_map.size());
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
          CustomData_get_layer_for_write(&mesh.vert_data, CD_ORCO, mesh.totvert)))
  {
    array_utils::gather(Span(orco, mesh.totvert),
                        new_to_old_verts_map,
                        MutableSpan(orco, mesh.totvert).take_back(new_to_old_verts_map.size()));
  }
  if (int *orig_indices = static_cast<int *>(
          CustomData_get_layer_for_write(&mesh.vert_data, CD_ORIGINDEX, mesh.totvert)))
  {
    array_utils::gather(
        Span(orig_indices, mesh.totvert),
        new_to_old_verts_map,
        MutableSpan(orig_indices, mesh.totvert).take_back(new_to_old_verts_map.size()));
  }
}

static void propagate_edge_attributes(Mesh &mesh, const Span<int> new_to_old_edge_map)
{
  CustomData_free_layers(&mesh.edge_data, CD_FREESTYLE_EDGE, mesh.totedge);
  CustomData_realloc(&mesh.edge_data, mesh.totedge, mesh.totedge + new_to_old_edge_map.size());
  mesh.totedge += new_to_old_edge_map.size();

  bke::MutableAttributeAccessor attributes = mesh.attributes_for_write();
  for (const bke::AttributeIDRef &id : attributes.all_ids()) {
    if (attributes.lookup_meta_data(id)->domain != ATTR_DOMAIN_EDGE) {
      continue;
    }
    if (id.name() == ".edge_verts") {
      continue;
    }
    bke::GSpanAttributeWriter attribute = attributes.lookup_for_write_span(id);
    if (!attribute) {
      continue;
    }
    bke::attribute_math::gather(
        attribute.span, new_to_old_edge_map, attribute.span.take_back(new_to_old_edge_map.size()));
    attribute.finish();
  }

  if (int *orig_indices = static_cast<int *>(
          CustomData_get_layer_for_write(&mesh.edge_data, CD_ORIGINDEX, mesh.totedge)))
  {
    array_utils::gather(
        Span(orig_indices, mesh.totedge),
        new_to_old_edge_map,
        MutableSpan(orig_indices, mesh.totedge).take_back(new_to_old_edge_map.size()));
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

static BitVector<> selection_to_bit_vector(const IndexMask &selection, const int total_size)
{
  BitVector<> bits(total_size);
  selection.to_bits(bits);
  return bits;
}

/**
 * Used for fanning around the corners connected to a vertex.
 *
 * Depending on the winding direction of neighboring faces, travelling from a corner across an edge
 * to a different face can give a corner that uses a different vertex than the original. To find
 * the face's corner that uses the original vertex, we may have to use the next corner instead.
 */
static int corner_on_edge_connected_to_vert(const Span<int> corner_verts,
                                            const int corner,
                                            const IndexRange face,
                                            const int vert)
{
  if (corner_verts[corner] == vert) {
    return corner;
  }
  const int other = bke::mesh::face_corner_next(face, corner);
  BLI_assert(corner_verts[other] == vert);
  return other;
}

using CornerFan = Vector<int>;

/**
 * A corner fan is a group of corners bordered by boundary edges or split vertices. We store corner
 * indices instead of edge indices because later on in the algorithm we only relink the
 * `corner_vert` array to each fan's new vertices.
 *
 * Find all groups of corners using this vertex that are reachable by crossing over connected faces
 * at non-split edges. The corners are not ordered in winding order, since we only need to group
 * connected faces into each fan.
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
  /* Each corner should only be added to a single fan. */
  BitVector<> used_corners(connected_corners.size());
  for (const int start_corner : connected_corners) {
    CornerFan fan;
    Vector<int> corner_stack({start_corner});
    while (!corner_stack.is_empty()) {
      const int corner = corner_stack.pop_last();
      const int i = connected_corners.first_index(corner);
      if (used_corners[i]) {
        continue;
      }
      used_corners[i].set();
      fan.append(corner);
      const int face = corner_to_face_map[corner];
      const int prev_corner = bke::mesh::face_corner_prev(faces[face], corner);
      /* Travel across the two edges neighboring this vertex, if they aren't split. */
      for (const int edge : {corner_edges[corner], corner_edges[prev_corner]}) {
        if (split_edges[edge]) {
          continue;
        }
        for (const int other_corner : edge_to_corner_map[edge]) {
          const int other_face = corner_to_face_map[other_corner];
          if (other_face == face) {
            /* Avoid continuing back to the same face. */
            continue;
          }
          const int neighbor_corner = corner_on_edge_connected_to_vert(
              corner_verts, other_corner, faces[other_face], vert);
          corner_stack.append(neighbor_corner);
        }
      }
    }
    if (!fan.is_empty()) {
      fans.append(std::move(fan));
    }
  }

  return fans;
}

/* Calculate groups of corners that are contiguously connected to each input vertex. */
static Array<Vector<CornerFan>> calc_all_corner_fans(const OffsetIndices<int> faces,
                                                     const Span<int> corner_verts,
                                                     const Span<int> corner_edges,
                                                     const GroupedSpan<int> vert_to_corner_map,
                                                     const GroupedSpan<int> edge_to_corner_map,
                                                     const Span<int> corner_to_face_map,
                                                     const BitSpan split_edges,
                                                     const IndexMask &affected_verts)
{
  Array<Vector<CornerFan>> corner_fans(affected_verts.size(), NoInitialization());
  affected_verts.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
    new (&corner_fans[mask])
        Vector<CornerFan>(calc_corner_fans_for_vertex(faces,
                                                      corner_verts,
                                                      corner_edges,
                                                      edge_to_corner_map,
                                                      corner_to_face_map,
                                                      split_edges,
                                                      vert_to_corner_map[vert],
                                                      vert));
  });
  return corner_fans;
}

/** Selected and unselected loose edges attached to a vertex. */
struct VertLooseEdges {
  Vector<int> selected;
  Vector<int> unselected;
};

/** Find selected and non-selected loose edges connected to a vertex. */
static VertLooseEdges calc_vert_loose_edges(const GroupedSpan<int> vert_to_edge_map,
                                            const BitSpan loose_edges,
                                            const BitSpan split_edges,
                                            const int vert)
{
  VertLooseEdges info;
  for (const int edge : vert_to_edge_map[vert]) {
    if (loose_edges[edge]) {
      if (split_edges[edge]) {
        info.selected.append(edge);
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
 * affected vertex index to the group of output vertex indices (indices are within those groups,
 * not indices in arrays of _all_ vertices). For every original vertex, reuse the original vertex
 * for the first of:
 *  1. The last face corner fan
 *  2. The last selected loose edge
 *  3. The group of non-selected loose edges
 * Using this order prioritizes the simplicity of the no-loose-edge case, which we assume is
 * more common.
 */
static OffsetIndices<int> calc_vert_ranges_per_old_vert(const IndexMask &affected_verts,
                                                        const Span<Vector<CornerFan>> corner_fans,
                                                        const GroupedSpan<int> vert_to_edge_map,
                                                        const BitSpan loose_edges,
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
  if (!loose_edges.is_empty()) {
    affected_verts.foreach_index(GrainSize(512), [&](const int vert, const int mask) {
      const VertLooseEdges info = calc_vert_loose_edges(
          vert_to_edge_map, loose_edges, split_edges, vert);
      offset_data[mask] += info.selected.size();
      if (corner_fans[mask].is_empty()) {
        /* Loose edges share their vertex with a corner fan if possible. */
        offset_data[mask] += info.unselected.size() > 0;
      }
    });
  }
  return offset_indices::accumulate_counts_to_offsets(offset_data);
}

/**
 * Update corner verts so that each fan of corners gets its own vertex. For the last "new vertex"
 * we can reuse the original vertex, which would otherwise become unused by any faces. The loose
 * edge case will have to deal with this later.
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
 * When each of an edge's vertices only have a single grouped corner fan, the vertices will not be
 * split, meaning all output edges will reuse the same vertices. Even if the edge was used by many
 * faces originally, because new vertices are not created, every edge would have the same vertex
 * indices. Using this fact we can avoid the need to use a more expensive data structure to
 * deduplicate the result edges.
 */
static BitVector<> calc_merged_new_edges(const int orig_verts_num,
                                         const Span<int2> orig_edges,
                                         const IndexMask &affected_verts,
                                         const Span<Vector<CornerFan>> corner_fans)
{
  Array<int> fan_counts(orig_verts_num, 0);
  affected_verts.foreach_index(GrainSize(4096), [&](const int vert, const int mask) {
    fan_counts[vert] = corner_fans[mask].size();
  });
  BitVector<> output_single_edge(orig_edges.size());
  threading::parallel_for_aligned(
      orig_edges.index_range(), 4096, bits::BitsPerInt, [&](const IndexRange range) {
        for (const int edge : range) {
          output_single_edge[edge].set(fan_counts[orig_edges[edge][0]] == 1 &&
                                       fan_counts[orig_edges[edge][1]] == 1);
        }
      });
  return output_single_edge;
}

/**
 * Based on updated corner vertex indices, update the edges in each face. This includes updating
 * corner edge indices, adding new edges, and reusing original edges for the first "split" edge.
 *
 * \param output_single_edge: As described in #calc_merged_new_edges, some edges will only
 * result in a single output edge, regardless of how many faces used the edge originally. Those
 * edges are marked by this bit map.
 */
static Vector<int2> add_new_edges(const OffsetIndices<int> faces,
                                  const Span<int> orig_corner_edges,
                                  const BitSpan selection,
                                  const Span<int> new_corner_verts,
                                  const BitSpan output_single_edge,
                                  MutableSpan<int2> edges,
                                  MutableSpan<int> corner_edges,
                                  Vector<int2> &new_edges,
                                  Vector<int> &new_edge_orig_indices)
{
  BitVector<> edge_used(edges.size());
  for (const int i : faces.index_range()) {
    const IndexRange face = faces[i];
    for (const int corner : face) {
      const int corner_next = bke::mesh::face_corner_next(face, corner);
      const int edge = orig_corner_edges[corner];
      const int2 new_edge(new_corner_verts[corner], new_corner_verts[corner_next]);
      if (!selection[edge] || output_single_edge[edge]) {
        /* Even if an edge wasn't selected, its vertex indices can be updated when neighboring
         * edges are selected and new vertices are created for the corner fans of its vertices. */
        edges[edge] = new_edge;
        continue;
      }
      if (edge_used[edge]) {
        corner_edges[corner] = edges.size() + new_edges.size();
        new_edges.append(new_edge);
        new_edge_orig_indices.append(edge);
      }
      else {
        edges[edge] = new_edge;
        edge_used[edge].set();
      }
    }
  }
  return new_edges;
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
                                      const IndexMask &affected_verts,
                                      const GroupedSpan<int> vert_to_edge_map,
                                      const BitSpan loose_edges,
                                      const BitSpan split_edges,
                                      const Span<Vector<CornerFan>> corner_fans,
                                      const OffsetIndices<int> new_verts_by_affected_vert,
                                      MutableSpan<int2> edges)
{
  affected_verts.foreach_index(GrainSize(1024), [&](const int vert, const int mask) {
    const IndexRange new_verts = new_verts_by_affected_vert[mask];
    /* Account for the reuse of the original vertex by non-loose corner fans. In practice this
     * means using the new vertices for each split loose edge until we run out of new vertices. We
     * then expect the count to match up with the number of new vertices reserved by
     * #calc_vert_ranges_per_old_vert. */
    int new_vert_i = std::max<int>(corner_fans[mask].size() - 1, 0);
    if (new_vert_i == new_verts.size()) {
      return;
    }
    const VertLooseEdges vert_info = calc_vert_loose_edges(
        vert_to_edge_map, loose_edges, split_edges, vert);
    for (const int edge : vert_info.selected) {
      const int new_vert = orig_verts_num + new_verts[new_vert_i];
      swap_edge_vert(edges[edge], vert, new_vert);
      new_vert_i++;
      if (new_vert_i == new_verts.size()) {
        return;
      }
    }
    const int new_vert = orig_verts_num + new_verts[new_vert_i];
    for (const int orig_edge : vert_info.unselected) {
      swap_edge_vert(edges[orig_edge], vert, new_vert);
    }
  });
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
                 const bke::AnonymousAttributePropagationInfo & /*propagation_info*/)
{
  const int orig_verts_num = mesh.totvert;
  const Span<int2> orig_edges = mesh.edges();
  const OffsetIndices faces = mesh.faces();
  const Array<int> orig_corner_edges = mesh.corner_edges();

  IndexMaskMemory memory;
  const IndexMask affected_verts = vert_selection_from_edge(
      orig_edges, selected_edges, orig_verts_num, memory);
  const BitVector<> selection_bits = selection_to_bit_vector(selected_edges, orig_edges.size());
  const bke::LooseEdgeCache &loose_edges = mesh.loose_edges();

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
  if (loose_edges.count > 0) {
    vert_to_edge_map = bke::mesh::build_vert_to_edge_map(
        orig_edges, orig_verts_num, vert_to_edge_offsets, vert_to_edge_indices);
  }

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
      vert_to_edge_map,
      loose_edges.is_loose_bits,
      selection_bits,
      vert_new_vert_offset_data);

  MutableSpan<int> new_corner_verts = mesh.corner_verts_for_write();
  calc_updated_corner_verts(
      orig_verts_num, corner_fans, new_verts_by_affected_vert, new_corner_verts);

  const BitVector<> single_edges = calc_merged_new_edges(
      orig_verts_num, orig_edges, affected_verts, corner_fans);

  Vector<int2> new_edges;
  Vector<int> new_edge_orig_indices;
  new_edges.reserve(selected_edges.size());
  new_edges.reserve(new_edge_orig_indices.size());
  add_new_edges(faces,
                orig_corner_edges,
                selection_bits,
                new_corner_verts,
                single_edges,
                mesh.edges_for_write(),
                mesh.corner_edges_for_write(),
                new_edges,
                new_edge_orig_indices);

  if (loose_edges.count > 0) {
    reassign_loose_edge_verts(orig_verts_num,
                              affected_verts,
                              vert_to_edge_map,
                              loose_edges.is_loose_bits,
                              selection_bits,
                              corner_fans,
                              new_verts_by_affected_vert,
                              mesh.edges_for_write());
  }

  propagate_edge_attributes(mesh, new_edge_orig_indices);
  mesh.edges_for_write().take_back(new_edges.size()).copy_from(new_edges);

  const Array<int> vert_map = calc_new_to_old_vert_map(affected_verts, new_verts_by_affected_vert);
  propagate_vert_attributes(mesh, vert_map);

  BKE_mesh_tag_edges_split(&mesh);
}

}  // namespace blender::geometry
