/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>

#include "BLI_linear_allocator.hh"
#include "BLI_struct_equality_utils.hh"
#include "BLI_utility_mixins.hh"

namespace blender::linear_allocator {

template<typename T, int64_t Capacity> struct UnorderedListSegment {
  UnorderedListSegment *next = nullptr;
  int64_t size = 0;
  std::array<TypedBuffer<T>, Capacity> values;
};

/**
 * This data structure can be used as a replacement of #Vector under some specific circumstances:
 * - The order of elements does not matter.
 * - No random access is necessary, just iteration over all values.
 * - The reallocation of #Vector when its capacity is reached is a bottleneck.
 * - Too large overallocations should be avoided.
 * - A separate #LinearAllocator should provide the memory instead of a global allocator or an
 *   allocator owned by the #UnorderedList.
 * - The `sizeof()` the list should be small.
 */
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

  class ConstIterator {
   private:
    const Segment *segment_ = nullptr;
    int64_t index_ = 0;

   public:
    ConstIterator(const Segment *segment, int64_t index = 0) : segment_(segment), index_(index) {}

    ConstIterator &operator++()
    {
      index_++;
      if (index_ == segment_->size) {
        segment_ = segment_->next;
        index_ = 0;
      }
      return *this;
    }

    const T &operator*() const
    {
      return *segment_->values[index_];
    }

    BLI_STRUCT_EQUALITY_OPERATORS_2(ConstIterator, segment_, index_)
  };

  class MutableIterator {
   private:
    Segment *segment_ = nullptr;
    int64_t index_ = 0;

   public:
    MutableIterator(Segment *segment, int64_t index = 0) : segment_(segment), index_(index) {}

    MutableIterator &operator++()
    {
      index_++;
      if (index_ == segment_->size) {
        segment_ = segment_->next;
        index_ = 0;
      }
      return *this;
    }

    T &operator*()
    {
      return *segment_->values[index_];
    }

    BLI_STRUCT_EQUALITY_OPERATORS_2(MutableIterator, segment_, index_)
  };

  ConstIterator begin() const
  {
    return ConstIterator(current_segment_, 0);
  }

  ConstIterator end() const
  {
    return ConstIterator(nullptr, 0);
  }

  MutableIterator begin()
  {
    return MutableIterator(current_segment_, 0);
  }

  MutableIterator end()
  {
    return MutableIterator(nullptr, 0);
  }
};

}  // namespace blender::linear_allocator
