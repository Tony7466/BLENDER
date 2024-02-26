/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>

#include "BLI_linear_allocator.hh"
#include "BLI_utility_mixins.hh"

namespace blender::linear_allocator {

template<typename T, int64_t Capacity> struct UnorderedListSegment {
  UnorderedListSegment *next = nullptr;
  int64_t size = 0;
  std::array<TypedBuffer<T>, Capacity> values;
};

template<typename T, int64_t SegmentCapacity = 4> class UnorderedList : NonCopyable {
 private:
  using Segment = UnorderedListSegment<T, SegmentCapacity>;
  Segment *current_segment_ = nullptr;

 public:
  UnorderedList() = default;

  UnorderedList(UnorderedList &&other)
  {
    current_segment_ = other.current_segment_;
    other.current_segment_ = nullptr;
  }

  ~UnorderedList()
  {
    if constexpr (!std::is_trivially_destructible_v<T>) {
      for (Segment *segment = current_segment_; segment; segment = segment->next) {
        for (const int64_t i : IndexRange(segment->size)) {
          T &value = *segment->values[i];
          std::destroy_at(&value);
        }
      }
    }
  }

  UnorderedList &operator=(UnorderedList &&other)
  {
    if (this == &other) {
      return *this;
    }
    std::destroy_at(this);
    new (this) UnorderedList(std::move(other));
    return *this;
  }

  void append(LinearAllocator<> &allocator, const T &value)
  {
    this->append_as(allocator, value);
  }

  void append(LinearAllocator<> &allocator, T &&value)
  {
    this->append_as(allocator, std::move(value));
  }

  template<typename... Args> void append_as(LinearAllocator<> &allocator, Args &&...args)
  {
    if (current_segment_ == nullptr || current_segment_->size == SegmentCapacity) {
      static_assert(std::is_trivially_destructible_v<Segment>);
      Segment *new_segment = allocator.construct<Segment>().release();
      new_segment->next = current_segment_;
      current_segment_ = new_segment;
    }
    T *value = &*current_segment_->values[current_segment_->size++];
    new (value) T(std::forward<Args>(args)...);
  }

  template<typename Fn> void for_each(Fn &&fn) const
  {
    for (const Segment *segment = current_segment_; segment; segment = segment->next) {
      for (const int64_t i : IndexRange(segment->size)) {
        const T &value = *segment->values[i];
        fn(value);
      }
    }
  }

  template<typename Fn> void for_each(Fn &&fn)
  {
    for (Segment *segment = current_segment_; segment; segment = segment->next) {
      for (const int64_t i : IndexRange(segment->size)) {
        T &value = *segment->values[i];
        fn(value);
      }
    }
  }
};

}  // namespace blender::linear_allocator
