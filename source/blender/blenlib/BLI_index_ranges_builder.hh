/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <array>

#include "BLI_index_range.hh"
#include "BLI_utility_mixins.hh"

namespace blender {

template<typename T, int64_t MaxRangesNum> class IndexRangesBuilder : NonCopyable, NonMovable {
 private:
  T *c_;
  /** Structure: [-1, start, end, start, end, start, end, ...]. */
  std::array<T, size_t(1) + size_t(MaxRangesNum) * 2> data_;

 public:
  IndexRangesBuilder()
  {
    static_assert(std::is_signed_v<T>);
    data_[0] = -1;
    c_ = data_.data();
  }

  bool add(const T index)
  {
    return this->add_range(index, index + 1);
  }

  bool add_range(const T start, const T end)
  {
    /* This is intentionally branchless. */
    const bool is_new_range = start > *c_;
    c_ += is_new_range;
    *c_ = start;
    c_ += is_new_range;
    *c_ = end;
    BLI_assert(c_ < data_.end());
    return is_new_range;
  }

  int64_t size() const
  {
    return (c_ - data_.data()) / 2;
  }

  bool is_empty() const
  {
    return c_ == data_.data();
  }

  IndexRange index_range() const
  {
    return IndexRange(this->size());
  }

  IndexRange operator[](const int64_t i) const
  {
    const T start = data_[size_t(1) + 2 * size_t(i)];
    const T end = data_[size_t(2) + 2 * size_t(i)];
    return IndexRange::from_begin_end(start, end);
  }
};

}  // namespace blender
