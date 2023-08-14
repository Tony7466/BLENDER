/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_offset_indices.hh"
#include "BLI_sort.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_vector_set.hh"

#include "atomic_ops.h"

namespace blender::grouped_indices {

static GroupedSpan<int> make_stabil(const OffsetIndices<int> offsets, MutableSpan<int> indices)
{
  threading::parallel_for(offsets.index_range(), 256, [&](const IndexRange range) {
    for (const int64_t index : range) {
      MutableSpan<int> group_indices = indices.slice(offsets[index]);
      parallel_sort(group_indices.begin(), group_indices.end());
    }
  });
  return {offsets, indices};
}

static Span<int> front_indices_to_same_value(const Span<int> indices, const Span<int> values)
{
  const int value = values[indices.first()];
  const int &first_other = *std::find_if(
      indices.begin(), indices.end(), [&](const int index) { return values[index] != value; });
  return indices.take_front(&first_other - indices.begin());
}

static GroupedSpan<int> from_indices_large_groups(const Span<int> group_indices,
                                                  MutableSpan<int> r_counts_to_offsets,
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
      atomic_add_and_fetch_int32(&r_counts_to_offsets[group], step_size);
      indices = indices.drop_front(step_size);
    }
  });

  const OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(
      r_counts_to_offsets);
  Array<int> counts(offsets.size(), 0);
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    const IndexRange range = segment.shift(segment_size * segment_index);
    const Span<int> segment_indices = src_indices.as_span().slice_safe(range);
    for (Span<int> indices = segment_indices; !indices.is_empty();) {
      const Span<int> indices_of_current_group = front_indices_to_same_value(indices,
                                                                             group_indices);
      const int step_size = indices_of_current_group.size();
      const int group = group_indices[indices.first()];
      const int start = atomic_add_and_fetch_int32(&counts[group], step_size) - step_size;
      const IndexRange dst_range = offsets[group].slice(start, step_size);
      array_utils::copy(indices_of_current_group, r_indices.slice(dst_range));
      indices = indices.drop_front(step_size);
    }
  });

  return make_stabil(offsets, r_indices);
}

static GroupedSpan<int> reverse_copy(const OffsetIndices<int> offsets,
                                     const Span<int> src_indices,
                                     MutableSpan<int> dst_indices)
{
  Array<int> revers_indices(src_indices.size());
  Array<int> counts(offsets.size(), 0);
  threading::parallel_for(src_indices.index_range(), 1024, [&](const IndexRange range) {
    const Span<int> segment_src_indices = src_indices.slice(range);
    MutableSpan<int> segment_dst_indices = revers_indices.as_mutable_span().slice(range);
    array_utils::copy(segment_src_indices, segment_dst_indices);
    for (int &group_to_index : segment_dst_indices) {
      const int index_in_group = atomic_add_and_fetch_int32(&counts[group_to_index], 1) - 1;
      group_to_index = offsets[group_to_index][index_in_group];
    }
  });

  threading::parallel_for(dst_indices.index_range(), 2048, [&](const IndexRange range) {
    for (const int index : range) {
      dst_indices[revers_indices[index]] = index;
    }
  });

  return make_stabil(offsets, dst_indices);
}

static GroupedSpan<int> from_indices_many_groups(const Span<int> group_indices,
                                                 MutableSpan<int> r_counts_to_offsets,
                                                 MutableSpan<int> r_indices)
{
  offset_indices::build_reverse_offsets(group_indices, r_counts_to_offsets);
  return reverse_copy(OffsetIndices<int>(r_counts_to_offsets), group_indices, r_indices);
}

bool is_fragmented(const Span<int> group_indices, const int total_groups)
{
  /* The magic number of group sizes is derived in a practical way.
   * To get around the problems of accuracy and error in this kind of
   * group size eval, there is a epsilon 100 in down direction. */
  return group_indices.size() / total_groups <= 900;
}

GroupedSpan<int> from_indices(const Span<int> group_indices,
                              const bool fragmented,
                              MutableSpan<int> r_counts_to_offsets,
                              MutableSpan<int> r_indices)
{
  BLI_assert(!group_indices.is_empty());
  BLI_assert(group_indices.size() == r_indices.size());
  BLI_assert(std::all_of(r_counts_to_offsets.begin(), r_counts_to_offsets.end(), [](int value) {
    return value == 0;
  }));
  BLI_assert(*std::min_element(group_indices.begin(), group_indices.end()) >= 0);
  BLI_assert(*std::max_element(group_indices.begin(), group_indices.end()) <
             r_counts_to_offsets.size() - 1);
  if (fragmented) {
    return from_indices_many_groups(group_indices, r_counts_to_offsets, r_indices);
  }
  else {
    return from_indices_large_groups(group_indices, r_counts_to_offsets, r_indices);
  }
}

static int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices)
{
  const VectorSet<int> deduplicated_groups(r_identifiers_to_indices);
  threading::parallel_for(
      r_identifiers_to_indices.index_range(), 2048, [&](const IndexRange range) {
        for (int &value : r_identifiers_to_indices.slice(range)) {
          value = deduplicated_groups.index_of(value);
        }
      });
  return deduplicated_groups.size();
}

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices)
{
  Array<int> group_indices(groups_ids.size());
  array_utils::copy(groups_ids, group_indices.as_mutable_span());
  const int total_groups = identifiers_to_indices(group_indices);
  r_offsets.reinitialize(total_groups + 1);
  r_indices.reinitialize(groups_ids.size());
  r_offsets.as_mutable_span().fill(0);
  return from_indices(
      group_indices, is_fragmented(group_indices, total_groups), r_offsets, r_indices);
}

}  // namespace blender::grouped_indices
