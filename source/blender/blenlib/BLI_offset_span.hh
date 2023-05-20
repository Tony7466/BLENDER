/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"

namespace blender {

template<typename T, typename BaseT> class OffsetSpan {
 private:
  T offset_ = 0;
  Span<BaseT> data_;

 public:
  OffsetSpan() = default;
  OffsetSpan(const T offset, const Span<BaseT> data) : offset_(offset), data_(data) {}

  Span<BaseT> base_span() const
  {
    return data_;
  }

  T offset() const
  {
    return offset_;
  }

  bool is_empty() const
  {
    return data_.is_empty();
  }

  int64_t size() const
  {
    return data_.size();
  }

  T last(const int64_t n = 0) const
  {
    return offset_ + data_.last(n);
  }

  IndexRange index_range() const
  {
    return data_.index_range();
  }

  T operator[](const int64_t i) const
  {
    return T(data_[i]) + offset_;
  }

  OffsetSpan slice(const IndexRange &range) const
  {
    return {offset_, data_.slice(range)};
  }

  OffsetSpan slice(const int64_t start, const int64_t size) const
  {
    return {offset_, data_.slice(start, size)};
  }

  class Iterator {
   private:
    T offset_;
    const BaseT *data_;

   public:
    Iterator(const T offset, const BaseT *data) : offset_(offset), data_(data) {}

    Iterator &operator++()
    {
      data_++;
      return *this;
    }

    T operator*() const
    {
      return T(*data_) + offset_;
    }

    friend bool operator!=(const Iterator &a, const Iterator &b)
    {
      BLI_assert(a.offset_ == b.offset_);
      return a.data_ != b.data_;
    }
  };

  Iterator begin() const
  {
    return {offset_, data_.begin()};
  }

  Iterator end() const
  {
    return {offset_, data_.end()};
  }
};

}  // namespace blender
