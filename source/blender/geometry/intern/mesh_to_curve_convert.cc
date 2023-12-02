/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "atomic_ops.h"

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_atomic_disjoint_set.hh"
#include "BLI_index_mask.hh"
#include "BLI_set.hh"
#include "BLI_sort.hh"
#include "BLI_task.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_mesh.hh"

#include "GEO_mesh_to_curve.hh"
#include "GEO_randomize.hh"

namespace blender::geometry {

BLI_NOINLINE bke::CurvesGeometry create_curve_from_vert_indices(
    const bke::AttributeAccessor &mesh_attributes,
    const Span<int> vert_indices,
    const Span<int> curve_offsets,
    const IndexRange cyclic_curves,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  bke::CurvesGeometry curves(vert_indices.size(), curve_offsets.size());
  curves.offsets_for_write().drop_back(1).copy_from(curve_offsets);
  curves.offsets_for_write().last() = vert_indices.size();
  curves.fill_curve_types(CURVE_TYPE_POLY);

  bke::MutableAttributeAccessor curves_attributes = curves.attributes_for_write();

  if (!cyclic_curves.is_empty()) {
    curves.cyclic_for_write().slice(cyclic_curves).fill(true);
  }

  /* Don't copy attributes that are built-in on meshes but not on curves. */
  Set<std::string> skip;
  for (const bke::AttributeIDRef &id : mesh_attributes.all_ids()) {
    if (mesh_attributes.is_builtin(id) && !curves_attributes.is_builtin(id)) {
      skip.add(id.name());
    }
  }

  bke::gather_attributes(
      mesh_attributes, ATTR_DOMAIN_POINT, propagation_info, skip, vert_indices, curves_attributes);

  mesh_attributes.for_all(
      [&](const bke::AttributeIDRef &id, const bke::AttributeMetaData meta_data) {
        if (meta_data.domain == ATTR_DOMAIN_POINT) {
          return true;
        }
        if (skip.contains(id.name())) {
          return true;
        }
        if (id.is_anonymous() && !propagation_info.propagate(id.anonymous_id())) {
          return true;
        }

        const bke::GAttributeReader src = mesh_attributes.lookup(id, ATTR_DOMAIN_POINT);
        /* Some attributes might not exist if they were builtin on domains that don't have
         * any elements, i.e. a face attribute on the output of the line primitive node. */
        if (!src) {
          return true;
        }
        bke::GSpanAttributeWriter dst = curves_attributes.lookup_or_add_for_write_only_span(
            id, ATTR_DOMAIN_POINT, meta_data.data_type);
        bke::attribute_math::gather(*src, vert_indices, dst.span);
        dst.finish();
        return true;
      });

  debug_randomize_curve_order(&curves);

  return curves;
}

struct CurveFromEdgesOutput {
  /** The indices in the mesh for each control point of each result curves. */
  Vector<int> vert_indices;
  /** The first index of each curve in the result. */
  Vector<int> curve_offsets;
  /** A subset of curves that should be set cyclic. */
  IndexRange cyclic_curves;
};

static Span<int> front_indices_to_same_value(const Span<int> indices, const Span<int> values)
{
  const int value = values[indices.first()];
  const int &first_other = *std::find_if(
      indices.begin(), indices.end(), [&](const int index) { return values[index] != value; });
  return indices.take_front(&first_other - indices.begin());
}

static void from_indices_large_groups(const Span<int> group_indices,
                                      MutableSpan<int> r_counts_to_offset,
                                      MutableSpan<int> r_indices)
{
  constexpr const int segment_size = 1024;
  constexpr const IndexRange segment(segment_size);
  const bool last_small_segmet = bool(group_indices.size() % segment_size);
  const int total_segments = group_indices.size() / segment_size + int(last_small_segmet);

  Array<int> src_indices(group_indices.size());
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    const IndexRange range = segment.shift(segment_size * segment_index);
    MutableSpan<int> segment_indices = src_indices.as_mutable_span().slice_safe(range);
    std::iota(segment_indices.begin(), segment_indices.end(), segment_size * segment_index);
    parallel_sort(segment_indices.begin(), segment_indices.end(), [&](const int a, const int b) {
      return group_indices[a] < group_indices[b];
    });

    for (Span<int> indices = segment_indices; !indices.is_empty();) {
      const int group = group_indices[indices.first()];
      const int step_size = front_indices_to_same_value(indices, group_indices).size();
      atomic_add_and_fetch_int32(&r_counts_to_offset[group], step_size);
      indices = indices.drop_front(step_size);
    }
  });

  const OffsetIndices<int> offset = offset_indices::accumulate_counts_to_offsets(
      r_counts_to_offset);
  Array<int> counts(offset.size(), 0);
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    const IndexRange range = segment.shift(segment_size * segment_index);
    const Span<int> segment_indices = src_indices.as_span().slice_safe(range);
    for (Span<int> indices = segment_indices; !indices.is_empty();) {
      const Span<int> indices_of_current_group = front_indices_to_same_value(indices,
                                                                             group_indices);
      const int step_size = indices_of_current_group.size();
      const int group = group_indices[indices.first()];
      const int start = atomic_add_and_fetch_int32(&counts[group], step_size) - step_size;
      const IndexRange dst_range = offset[group].slice(start, step_size);
      array_utils::copy(indices_of_current_group, r_indices.slice(dst_range));
      indices = indices.drop_front(step_size);
    }
  });
}

static GroupedSpan<int> gather_groups(const Span<int> group_indices,
                                      const int groups_num,
                                      Array<int> &r_offsets,
                                      Array<int> &r_indices)
{
  r_offsets.reinitialize(groups_num + 1);
  r_offsets.as_mutable_span().fill(0);
  r_indices.reinitialize(group_indices.size());
  from_indices_large_groups(group_indices, r_offsets, r_indices);
  return {OffsetIndices<int>(r_offsets), r_indices};
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

static void sort_pairs(const OffsetIndices<int> offset, MutableSpan<int2> r_pairs)
{
  static const auto ensure_order = [](const int2 current, int2 &next) {
    if (ELEM(next[1], current[0], current[1])) {
      std::swap(next[0], next[1]);
    }
    BLI_assert(current[1] == next[0]);
  };

  threading::parallel_for(offset.index_range(), 1024, [&](const IndexRange range) {
    for (const int64_t group_i : range) {
      MutableSpan<int2> next_segment = r_pairs.slice(offset[group_i]);
      for (const int index : offset[group_i].index_range().drop_back(1)) {
        int2 &current = next_segment.first();
        next_segment = next_segment.drop_front(1);
        int2 *next = std::find_if(next_segment.begin(), next_segment.end(), [&](const int2 next) {
          return ELEM(current[1], next[0], next[1]);
        });
        if (next != next_segment.end()) {
          ensure_order(current, *next);
          std::swap(next_segment.first(), *next);
          continue;
        }
        /* This is the first element a non-cyclic star and looking for unconnected edge was
         * failure. Try again in other direction. */
        int2 *other_next = std::find_if(
            next_segment.begin(), next_segment.end(), [&](const int2 next) {
              return ELEM(current[0], next[0], next[1]);
            });
        if (other_next != next_segment.end()) {
          std::swap(current[0], current[1]);
          ensure_order(current, *other_next);
          std::swap(next_segment.first(), *other_next);
          continue;
        }
        BLI_assert_unreachable();
      }
    }
  });
}

static int edges_to_curve_point_indices(const int verts_num,
                                        const Span<int2> edges,
                                        Array<int> &r_curve_offsets,
                                        Array<int> &r_curve_indices)
{
  Array<int> vert_total_edges(verts_num, 0);
  array_utils::count_indices(edges.cast<int>(), vert_total_edges);

  const auto vert_is_end = [&](const int vert_i) -> bool { return vert_total_edges[vert_i] != 2; };

  IndexMaskMemory memory;

  /* All edges that will be converted in individual curves. */
  const IndexMask short_edges_mask = IndexMask::from_predicate(
      edges.index_range(), GrainSize(4098), memory, [&](const int edge_i) {
        const int2 edge = edges[edge_i];
        return vert_is_end(edge[0]) && vert_is_end(edge[1]);
      });

  /* All edges that will be in curve with at least 2 segment. */
  const IndexMask long_edges_mask = short_edges_mask.complement(edges.index_range(), memory);

  /* Edges that is the ends on curves. Curve might be formed by just to the end edges. */
  const IndexMask end_edges_mask = IndexMask::from_predicate(
      long_edges_mask, GrainSize(4098), memory, [&](const int edge_i) {
        const int2 edge = edges[edge_i];
        return vert_is_end(edge[0]) != vert_is_end(edge[1]);
      });

  /* Edges that might form cyclic curve or be part of non cyclic curve that will be capped by the
   * end edges. */
  const IndexMask body_edges_mask = IndexMask::from_predicate(
      long_edges_mask, GrainSize(4098), memory, [&](const int edge_i) {
        const int2 edge = edges[edge_i];
        return !vert_is_end(edge[0]) && !vert_is_end(edge[1]);
      });

  /* Vertices between just two edges. This vertices will be only in single curve. */
  const IndexMask verts_mask = IndexMask::from_predicate(
      IndexMask(verts_num), GrainSize(4098), memory, [&](const int vert_i) {
        return !vert_is_end(vert_i);
      });

  Array<int> reverse_vert_pos(verts_mask.min_array_size());
  index_mask::build_reverse_map<int>(verts_mask, reverse_vert_pos);

  AtomicDisjointSet vert_curves(verts_mask.size());
  body_edges_mask.foreach_index(GrainSize(2048), [&](const int edge_i) {
    const int2 edge = edges[edge_i];
    vert_curves.join(reverse_vert_pos[edge[0]], reverse_vert_pos[edge[1]]);
  });

  const int long_curves_total = vert_curves.count_sets();

  Array<int> vert_curve_index(verts_mask.size());
  vert_curves.calc_reduced_ids(vert_curve_index);

  Array<bool> cyclic(long_curves_total, true);
  end_edges_mask.foreach_index_optimized<int>(GrainSize(4098), [&](const int edge_i) {
    /* Curve is not cyclic if contains at least one end edge. One vert of the end edge is vert of
     * non-cyclic curve body. */
    const int2 edge = edges[edge_i];
    const int vert_i = vert_is_end(edge[0]) ? edge[1] : edge[0];
    const int vert_pos = reverse_vert_pos[vert_i];
    const int curve_i = vert_curve_index[vert_pos];
    cyclic[curve_i] = false;
  });

  const IndexMask cyclic_curves_mask = IndexMask::from_bools(cyclic, memory);
  const IndexMask non_cyclic_curves_mask = cyclic_curves_mask.complement(cyclic.index_range(),
                                                                         memory);

  const int curves_total = long_curves_total + short_edges_mask.size();
  r_curve_offsets.reinitialize(curves_total + 1);
  MutableSpan<int> curve_offsets = r_curve_offsets.as_mutable_span().drop_back(1);

  const IndexRange cyclic_curves_range(cyclic_curves_mask.size());
  const IndexRange non_cyclic_curves_range(cyclic_curves_mask.size(),
                                           non_cyclic_curves_mask.size());
  const IndexRange short_curves_range =
      IndexRange(curves_total).take_back(short_edges_mask.size());

  /* Curves from just one edge. */
  MutableSpan<int> cyclic_curves = curve_offsets.slice(cyclic_curves_range);
  MutableSpan<int> non_cyclic_curves = curve_offsets.slice(non_cyclic_curves_range);
  curve_offsets.slice(short_curves_range).fill(2);

  Array<int> body_edges_curve_index(body_edges_mask.size());
  body_edges_mask.foreach_index_optimized<int>(
      GrainSize(2098), [&](const int edge_i, const int edge_pos) {
        const int2 edge = edges[edge_i];
        const int vert_pos = reverse_vert_pos[edge[0]];
        body_edges_curve_index[edge_pos] = vert_curve_index[vert_pos];
      });

  Array<int> body_edges_curves_offsets;
  Array<int> body_edges_curves_indices;
  gather_groups(body_edges_curve_index,
                long_curves_total,
                body_edges_curves_offsets,
                body_edges_curves_indices);
  OffsetIndices<int> long_curves(body_edges_curves_offsets);

  cyclic_curves_mask.foreach_index_optimized<int>(
      GrainSize(4098), [&](const int curve_i, const int curve_pos) {
        cyclic_curves[curve_pos] = long_curves[curve_i].size();
      });
  non_cyclic_curves_mask.foreach_index_optimized<int>(
      GrainSize(4098), [&](const int curve_i, const int curve_pos) {
        non_cyclic_curves[curve_pos] = long_curves[curve_i].size() + 2;
      });

  OffsetIndices<int> curves = offset_indices::accumulate_counts_to_offsets(r_curve_offsets);
  r_curve_indices.reinitialize(curves.total_size());

  const OffsetIndices<int> short_curves = curves.slice(short_curves_range);
  short_edges_mask.foreach_index_optimized<int>(
      GrainSize(4098), [&](const int edge_i, const int edge_pos) {
        const int2 &edge = edges[edge_i];
        r_curve_indices[short_curves[edge_pos][0]] = edge[0];
        r_curve_indices[short_curves[edge_pos][1]] = edge[1];
      });

  const OffsetIndices<int> cyclic_curves_offsets = curves.slice(cyclic_curves_range);

  Array<int2> curve_body_edges(body_edges_curves_indices.size());
  parallel_transform(body_edges_curves_indices, 4098, [&](const int edge_pos) {
    return body_edges_mask[edge_pos];
  });
  array_utils::gather(
      edges, body_edges_curves_indices.as_span(), curve_body_edges.as_mutable_span());
  sort_pairs(cyclic_curves_offsets, curve_body_edges);

  cyclic_curves_mask.foreach_index_optimized<int>(
      GrainSize(4098), [&](const int curve_i, const int curve_pos) {
        MutableSpan<int> curve_vert_indices = r_curve_indices.as_mutable_span().slice(
            cyclic_curves_offsets[curve_pos]);
        MutableSpan<int2> curve_edge_indices = curve_body_edges.as_mutable_span().slice(
            long_curves[curve_i]);
      });

  /*
    Array<int> curves_offsets(cyclic_curves_offsets.size());
    array_utils::copy(cyclic_curves_offsets.as_span(), curves_offsets.as_mutable_span());
    threading::parallel_for(curves_offsets.index_range(), 4098, )

        threading::parallel_for(long_curves.index_range(), 1024, [&](const IndexRange range) {
          for (const int64_t i : range) {
            MutableSpan<int> curve_verts = curves_indices.as_mutable_span().slice(
                long_curves[i]);
          }
        });
  */

  return cyclic_curves_range.size();
}

BLI_NOINLINE static bke::CurvesGeometry edges_to_curves_convert(
    const Mesh &mesh,
    const Span<int2> edges,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  Array<int> curve_offsets;
  Array<int> curve_indices;
  const int total_cyclic = edges_to_curve_point_indices(
      mesh.totvert, edges, curve_offsets, curve_indices);
  return create_curve_from_vert_indices(mesh.attributes(),
                                        curve_indices,
                                        curve_offsets.as_span().drop_back(1),
                                        IndexRange(total_cyclic),
                                        propagation_info);
}

bke::CurvesGeometry mesh_to_curve_convert(
    const Mesh &mesh,
    const IndexMask &selection,
    const bke::AnonymousAttributePropagationInfo &propagation_info)
{
  const Span<int2> edges = mesh.edges();
  if (selection.size() == edges.size()) {
    return edges_to_curves_convert(mesh, edges, propagation_info);
  }
  Array<int2> selected_edges(selection.size());
  array_utils::gather(edges, selection, selected_edges.as_mutable_span());
  return edges_to_curves_convert(mesh, selected_edges, propagation_info);
}

}  // namespace blender::geometry
