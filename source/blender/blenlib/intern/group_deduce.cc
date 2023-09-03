/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_offset_indices.hh"
#include "BLI_sort.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector_set.hh"

#include "atomic_ops.h"

namespace blender::grouped_indices {

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
  constexpr int segment_size = 1024;
  constexpr IndexRange segment(segment_size);
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

static void reverse_copy(const OffsetIndices<int> offset,
                         const Span<int> src_indices,
                         MutableSpan<int> dst_indices)
{
  Array<int> revers_indices(src_indices.size());
  Array<int> counts(offset.size(), 0);
  threading::parallel_for(src_indices.index_range(), 1024, [&](const IndexRange range) {
    const Span<int> segment_src_indices = src_indices.slice(range);
    MutableSpan<int> segment_dst_indices = revers_indices.as_mutable_span().slice(range);
    array_utils::copy(segment_src_indices, segment_dst_indices);
    for (int &group_to_index : segment_dst_indices) {
      const int index_in_group = atomic_add_and_fetch_int32(&counts[group_to_index], 1) - 1;
      group_to_index = offset[group_to_index][index_in_group];
    }
  });

  threading::parallel_for(dst_indices.index_range(), 2048, [&](const IndexRange range) {
    for (const int index : range) {
      dst_indices[revers_indices[index]] = index;
    }
  });
}

static void from_indices_many_groups(const Span<int> group_indices,
                                     MutableSpan<int> r_counts_to_offset,
                                     MutableSpan<int> r_indices)
{
  offset_indices::build_reverse_offsets(group_indices, r_counts_to_offset);
  reverse_copy(OffsetIndices<int>(r_counts_to_offset), group_indices, r_indices);
}

bool is_fragmented(const Span<int> group_indices, const int total_groups)
{
  /* The magic number of group sizes is derived in a practical way.
   * To get around the problems of accuracy and error in this kind of
   * group size eval, there is a epsilon 100 in down direction. */
  return group_indices.size() / total_groups <= 900;
}

int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices, const bool stable)
{
  const VectorSet<int> deduplicated_groups(r_identifiers_to_indices);
  threading::parallel_for(
      r_identifiers_to_indices.index_range(), 2048, [&](const IndexRange range) {
        for (int &value : r_identifiers_to_indices.slice(range)) {
          value = deduplicated_groups.index_of(value);
        }
      });

  if (stable) {
    Array<int> indices(deduplicated_groups.size());
    std::iota(indices.begin(), indices.end(), 0);
    parallel_sort(indices.begin(), indices.end(), [&](const int index_a, const int index_b) {
      return deduplicated_groups[index_a] < deduplicated_groups[index_b];
    });

    threading::parallel_for(
        r_identifiers_to_indices.index_range(), 2048, [&](const IndexRange range) {
          for (int &value : r_identifiers_to_indices.slice(range)) {
            value = indices[value];
          }
        });
  }

  return deduplicated_groups.size();
}

GroupedSpan<int> from_indices(const Span<int> group_indices,
                              MutableSpan<int> r_counts_to_offset,
                              MutableSpan<int> r_indices,
                              const bool fragmented,
                              const bool stable)
{
  if (group_indices.is_empty()) {
    return {OffsetIndices<int>(r_counts_to_offset), r_indices};
  }
  BLI_assert(group_indices.size() == r_indices.size());
  BLI_assert(std::all_of(
      r_counts_to_offset.begin(), r_counts_to_offset.end(), [](int value) { return value == 0; }));
  BLI_assert(*std::min_element(group_indices.begin(), group_indices.end()) >= 0);
  BLI_assert(*std::max_element(group_indices.begin(), group_indices.end()) <
             r_counts_to_offset.size() - 1);

  if (fragmented) {
    from_indices_many_groups(group_indices, r_counts_to_offset, r_indices);
  }
  else {
    from_indices_large_groups(group_indices, r_counts_to_offset, r_indices);
  }

  const OffsetIndices<int> offset(r_counts_to_offset);
  if (stable) {
    threading::parallel_for(offset.index_range(), 256, [&](const IndexRange range) {
      for (const int64_t index : range) {
        MutableSpan<int> group_indices = r_indices.slice(offset[index]);
        parallel_sort(group_indices.begin(), group_indices.end());
      }
    });
  }

  return {offset, r_indices};
}

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  const IndexMask &universe,
                                  Array<int> &r_offset,
                                  Array<int> &r_indices,
                                  const bool stable)
{
  BLI_assert(universe.last() < groups_ids.size());
  Array<int> group_indices(universe.size());
  array_utils::gather(groups_ids, universe, group_indices.as_mutable_span());
  const int total_groups = identifiers_to_indices(group_indices, stable);
  const bool fragmented = is_fragmented(group_indices, total_groups);
  r_offset.reinitialize(total_groups + 1);
  r_offset.as_mutable_span().fill(0);
  r_indices.reinitialize(universe.size());
  return from_indices(group_indices, r_offset, r_indices, fragmented, stable);
}

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offset,
                                  Array<int> &r_indices,
                                  const bool stable)
{
  return from_identifiers(groups_ids, IndexMask(groups_ids.size()), r_offset, r_indices, stable);
}

}  // namespace blender::grouped_indices
