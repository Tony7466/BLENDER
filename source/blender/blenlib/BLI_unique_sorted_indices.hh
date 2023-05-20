/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <variant>

#include "BLI_vector.hh"

namespace blender::unique_sorted_indices {

template<typename T> inline bool non_empty_is_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  return indices.last() - indices.first() == indices.size() - 1;
}

template<typename T> inline IndexRange non_empty_as_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  BLI_assert(non_empty_is_range(indices));
  return IndexRange(indices.first(), indices.size());
}

template<typename T> inline std::optional<IndexRange> non_empty_as_range_try(const Span<T> indices)
{
  if (non_empty_is_range(indices)) {
    return non_empty_as_range(indices);
  }
  return std::nullopt;
}

template<typename T> inline int64_t find_size_of_next_range(const Span<T> indices)
{
  BLI_assert(!indices.is_empty());
  return std::lower_bound(
             indices.begin(),
             indices.end(),
             0,
             [indices, offset = indices[0]](const T &element, const int64_t /*dummy*/) {
               const int64_t element_index = &element - indices.begin();
               return element - offset == element_index;
             }) -
         indices.begin();
}

template<typename T>
inline int64_t find_size_until_next_range(const Span<T> indices, const int64_t min_range_size)
{
  BLI_assert(!indices.is_empty());
  int64_t current_range_size = 1;
  int64_t last_value = indices[0];
  for (const int64_t i : indices.index_range().drop_front(1)) {
    const T current_value = indices[i];
    if (current_value == last_value + 1) {
      current_range_size++;
      if (current_range_size >= min_range_size) {
        return i - min_range_size + 1;
      }
    }
    else {
      current_range_size = 1;
    }
    last_value = current_value;
  }
  return indices.size();
}

template<typename T>
inline int64_t split_to_ranges_and_spans(const Span<T> indices,
                                         const int64_t range_threshold,
                                         Vector<std::variant<IndexRange, Span<T>>> &r_segments)
{
  BLI_assert(range_threshold >= 1);
  const int64_t old_segments_num = r_segments.size();
  Span<T> remaining_indices = indices;
  while (!remaining_indices.is_empty()) {
    if (const std::optional<IndexRange> range = non_empty_as_range_try(remaining_indices)) {
      /* All remaining indices are range. */
      r_segments.append(*range);
      break;
    }
    if (non_empty_is_range(remaining_indices.take_front(range_threshold))) {
      /* Next segment is a range. Now find the place where the range ends. */
      const int64_t segment_size = find_size_of_next_range(remaining_indices);
      r_segments.append(IndexRange(remaining_indices[0], segment_size));
      remaining_indices = remaining_indices.drop_front(segment_size);
      continue;
    }
    /* Next segment is just indices. Now find the place where the next range starts. */
    const int64_t segment_size = find_size_until_next_range(remaining_indices, range_threshold);
    const Span<T> segment_indices = remaining_indices.take_front(segment_size);
    if (const std::optional<IndexRange> range = non_empty_as_range_try(segment_indices)) {
      r_segments.append(*range);
    }
    else {
      r_segments.append(segment_indices);
    }
    remaining_indices = remaining_indices.drop_front(segment_size);
  }
  return r_segments.size() - old_segments_num;
}

}  // namespace blender::unique_sorted_indices
