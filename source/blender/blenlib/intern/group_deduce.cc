/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_array.hh"
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
      MutableSpan<int> group = indices.slice(offsets[index]);
      parallel_sort(group.begin(), group.end());
    }
  });
  return {offsets, indices};
}

static Span<int> take_first_equals(const Span<int> indices, const Span<int> values)
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

  Array<int> indices(group_indices.size());
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    MutableSpan<int> local_indices = indices.as_mutable_span().slice_safe(
        segment.shift(segment_size * segment_index));
    std::iota(local_indices.begin(), local_indices.end(), segment_index * segment_size);
    parallel_sort(local_indices.begin(), local_indices.end(), [&](const int a, const int b) {
      return group_indices[a] < group_indices[b];
    });

    int start = 0;
    while (true) {
      const Span<int> local = local_indices.drop_front(start);
      if (local.is_empty()) {
        break;
      }
      const int group = group_indices[local.first()];
      const int current_size = take_first_equals(local, group_indices).size();
      start += current_size;
      atomic_add_and_fetch_int32(&r_counts_to_offsets[group], current_size);
    }
  });

  OffsetIndices<int> offsets = offset_indices::accumulate_counts_to_offsets(r_counts_to_offsets);

  Array<int> counts(offsets.size(), 0);
  threading::parallel_for_each(IndexRange(total_segments), [&](const int segment_index) {
    MutableSpan<int> local_indices = indices.as_mutable_span().slice_safe(
        segment.shift(segment_size * segment_index));
    int start = 0;
    while (true) {
      const Span<int> local = local_indices.drop_front(start);
      if (local.is_empty()) {
        break;
      }
      const int group = group_indices[local.first()];
      const Span<int> src = take_first_equals(local, group_indices);
      start += src.size();
      const int current_start = atomic_add_and_fetch_int32(&counts[group], src.size()) -
                                src.size();
      const IndexRange finall = offsets[group].slice(current_start, src.size());
      MutableSpan<int> dst = r_indices.slice(finall);
      std::copy(src.begin(), src.end(), dst.begin());
    }
  });

  return make_stabil(offsets, r_indices);
}

static GroupedSpan<int> reverse_copy(const OffsetIndices<int> offsets,
                                     const Span<int> src_indices,
                                     MutableSpan<int> dst_indices)
{
  Array<int> group_indices(src_indices.size());
  Array<int> counts(offsets.size(), -1);
  threading::parallel_for(group_indices.index_range(), 1024, [&](const IndexRange range) {
    MutableSpan<int> local_group_indices = group_indices.as_mutable_span().slice(range);
    const Span<int> local_group_indices_src = src_indices.slice(range);
    std::copy(local_group_indices_src.begin(),
              local_group_indices_src.end(),
              local_group_indices.begin());
    for (int &group_index : local_group_indices) {
      const int index_in_group = atomic_add_and_fetch_int32(&counts[group_index], 1);
      group_index = offsets[group_index][index_in_group];
    }
  });

  threading::parallel_for(dst_indices.index_range(), 2048, [&](const IndexRange range) {
    for (const int index : range) {
      dst_indices[group_indices[index]] = index;
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

static int identifiers_to_indices(const Span<int> identifiers, MutableSpan<int> r_indices)
{
  const VectorSet<int> deduplicated_groups(identifiers);
  threading::parallel_for(identifiers.index_range(), 2048, [&](const IndexRange range) {
    for (const int64_t index : range) {
      r_indices[index] = deduplicated_groups.index_of(identifiers[index]);
    }
  });
  return deduplicated_groups.size();
}

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices)
{
  Array<int> groups_indices(groups_ids.size());
  const int group_total = identifiers_to_indices(groups_ids, groups_indices);
  r_offsets.reinitialize(group_total + 1);
  r_offsets.as_mutable_span().fill(0);
  r_indices.reinitialize(groups_ids.size());
  return from_indices(
      groups_indices, is_fragmented(groups_indices, group_total), r_offsets, r_indices);
}

}  // namespace blender::grouped_indices
