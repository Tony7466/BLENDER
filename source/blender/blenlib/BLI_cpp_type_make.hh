/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_cpp_type.hh"
#include "BLI_timeit.hh"
#include "BLI_utildefines.h"

namespace blender::cpp_type_util {

constexpr bool simd_try = !true;

template<typename T> void default_construct_cb(void *ptr)
{
  new (ptr) T;
}
template<typename T> void default_construct_indices_cb(void *ptr, const IndexMask &mask)
{
  if constexpr (std::is_trivially_constructible_v<T>) {
    return;
  }

  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::default_initialized_n(static_cast<T *>(ptr) + base_indices.first(),
                                       base_indices.size());
      }
      else {
        for (const int64_t i : indices.index_range()) {
          new (static_cast<T *>(ptr) + indices[i]) T;
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>([&](int64_t i) { new (static_cast<T *>(ptr) + i) T; });
  }
}

template<typename T> void value_initialize_cb(void *ptr)
{
  new (ptr) T();
}

template<typename T> void value_initialize_indices_cb(void *ptr, const IndexMask &mask)
{
  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::default_construct_n(static_cast<T *>(ptr) + base_indices.first(),
                                     base_indices.size());
      }
      else {
        for (const int64_t i : indices.index_range()) {
          new (static_cast<T *>(ptr) + indices[i]) T();
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>([&](int64_t i) { new (static_cast<T *>(ptr) + i) T(); });
  }
}

template<typename T> void destruct_cb(void *ptr)
{
  (static_cast<T *>(ptr))->~T();
}
template<typename T> void destruct_indices_cb(void *ptr, const IndexMask &mask)
{
  if (std::is_trivially_destructible_v<T>) {
    return;
  }
  T *ptr_ = static_cast<T *>(ptr);

  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::destruct_n(ptr_ + base_indices.first(), base_indices.size());
      }
      else {
        for (const int64_t i : indices.index_range()) {
          ptr_[indices[i]].~T();
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>([&](int64_t i) { ptr_[i].~T(); });
  }
}

template<typename T> void copy_assign_cb(const void *src, void *dst)
{
  *static_cast<T *>(dst) = *static_cast<const T *>(src);
}
template<typename T> void copy_assign_indices_cb(const void *src, void *dst, const IndexMask &mask)
{
  const T *src_ = static_cast<const T *>(src);
  T *dst_ = static_cast<T *>(dst);

  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::initialized_copy_n(
            src_ + base_indices.first(), base_indices.size(), dst_ + base_indices.first());
      }
      else {
        for (const int64_t i : indices.index_range()) {
          dst_[indices[i]] = src_[indices[i]];
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>([&](int64_t i) { dst_[i] = src_[i]; });
  }
}
template<typename T>
void copy_assign_compressed_cb(const void *src, void *dst, const IndexMask &mask)
{
  const T *src_ = static_cast<const T *>(src);
  T *dst_ = static_cast<T *>(dst);

  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices, const int64_t start_segment_pos) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::initialized_copy_n(
            src_ + base_indices.first(), base_indices.size(), dst_ + start_segment_pos);
      }
      else {
        for (const int64_t i : indices.index_range()) {
          dst_[start_segment_pos + i] = src_[indices[i]];
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>(
        [&](const int64_t i, const int64_t pos) { dst_[pos] = src_[i]; });
  }
}

template<typename T> void copy_construct_cb(const void *src, void *dst)
{
  blender::uninitialized_copy_n(static_cast<const T *>(src), 1, static_cast<T *>(dst));
}
template<typename T>
void copy_construct_indices_cb(const void *src, void *dst, const IndexMask &mask)
{
  const T *src_ = static_cast<const T *>(src);
  T *dst_ = static_cast<T *>(dst);

  std::stringstream name;
  if constexpr (simd_try) {
    name << __func__ << " simd;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_segment([&](const IndexMaskSegment indices) {
      const Span<int16_t> base_indices = indices.base_span();
      if (unique_sorted_indices::non_empty_is_range(base_indices)) {
        blender::uninitialized_copy_n(
            src_ + base_indices.first(), base_indices.size(), dst_ + base_indices.first());
      }
      else {
        for (const int64_t i : indices.index_range()) {
          new (dst_ + indices[i]) T(src_[indices[i]]);
        }
      }
    });
  }
  else {
    name << __func__ << " default;";
    SCOPED_TIMER_AVERAGED(name.str());
    mask.foreach_index_optimized<int64_t>([&](int64_t i) { new (dst_ + i) T(src_[i]); });
  }
}
template<typename T>
void copy_construct_compressed_cb(const void *src, void *dst, const IndexMask &mask)
{
  const T *src_ = static_cast<const T *>(src);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>(
      [&](const int64_t i, const int64_t pos) { new (dst_ + pos) T(src_[i]); });
}

template<typename T> void move_assign_cb(void *src, void *dst)
{
  blender::initialized_move_n(static_cast<T *>(src), 1, static_cast<T *>(dst));
}
template<typename T> void move_assign_indices_cb(void *src, void *dst, const IndexMask &mask)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) { dst_[i] = std::move(src_[i]); });
}

template<typename T> void move_construct_cb(void *src, void *dst)
{
  blender::uninitialized_move_n(static_cast<T *>(src), 1, static_cast<T *>(dst));
}
template<typename T> void move_construct_indices_cb(void *src, void *dst, const IndexMask &mask)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) { new (dst_ + i) T(std::move(src_[i])); });
}

template<typename T> void relocate_assign_cb(void *src, void *dst)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  *dst_ = std::move(*src_);
  src_->~T();
}
template<typename T> void relocate_assign_indices_cb(void *src, void *dst, const IndexMask &mask)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) {
    dst_[i] = std::move(src_[i]);
    src_[i].~T();
  });
}

template<typename T> void relocate_construct_cb(void *src, void *dst)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  new (dst_) T(std::move(*src_));
  src_->~T();
}
template<typename T>
void relocate_construct_indices_cb(void *src, void *dst, const IndexMask &mask)
{
  T *src_ = static_cast<T *>(src);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) {
    new (dst_ + i) T(std::move(src_[i]));
    src_[i].~T();
  });
}

template<typename T> void fill_assign_cb(const void *value, void *dst, int64_t n)
{
  const T &value_ = *static_cast<const T *>(value);
  T *dst_ = static_cast<T *>(dst);

  for (int64_t i = 0; i < n; i++) {
    dst_[i] = value_;
  }
}
template<typename T>
void fill_assign_indices_cb(const void *value, void *dst, const IndexMask &mask)
{
  const T &value_ = *static_cast<const T *>(value);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) { dst_[i] = value_; });
}

template<typename T> void fill_construct_cb(const void *value, void *dst, int64_t n)
{
  const T &value_ = *static_cast<const T *>(value);
  T *dst_ = static_cast<T *>(dst);

  for (int64_t i = 0; i < n; i++) {
    new (dst_ + i) T(value_);
  }
}
template<typename T>
void fill_construct_indices_cb(const void *value, void *dst, const IndexMask &mask)
{
  const T &value_ = *static_cast<const T *>(value);
  T *dst_ = static_cast<T *>(dst);

  mask.foreach_index_optimized<int64_t>([&](int64_t i) { new (dst_ + i) T(value_); });
}

template<typename T> void print_cb(const void *value, std::stringstream &ss)
{
  const T &value_ = *static_cast<const T *>(value);
  ss << value_;
}

template<typename T> bool is_equal_cb(const void *a, const void *b)
{
  const T &a_ = *static_cast<const T *>(a);
  const T &b_ = *static_cast<const T *>(b);
  return a_ == b_;
}

template<typename T> uint64_t hash_cb(const void *value)
{
  const T &value_ = *static_cast<const T *>(value);
  return get_default_hash(value_);
}

}  // namespace blender::cpp_type_util

namespace blender {

template<typename T, CPPTypeFlags Flags>
CPPType::CPPType(TypeTag<T> /*type*/,
                 TypeForValue<CPPTypeFlags, Flags> /*flags*/,
                 const StringRef debug_name)
{
  using namespace cpp_type_util;

  debug_name_ = debug_name;
  size_ = int64_t(sizeof(T));
  alignment_ = int64_t(alignof(T));
  is_trivial_ = std::is_trivial_v<T>;
  is_trivially_destructible_ = std::is_trivially_destructible_v<T>;
  if constexpr (std::is_default_constructible_v<T>) {
    default_construct_ = default_construct_cb<T>;
    default_construct_indices_ = default_construct_indices_cb<T>;
    value_initialize_ = value_initialize_cb<T>;
    value_initialize_indices_ = value_initialize_indices_cb<T>;
    static T default_value;
    default_value_ = &default_value;
  }
  if constexpr (std::is_destructible_v<T>) {
    destruct_ = destruct_cb<T>;
    destruct_indices_ = destruct_indices_cb<T>;
  }
  if constexpr (std::is_copy_assignable_v<T>) {
    copy_assign_ = copy_assign_cb<T>;
    copy_assign_indices_ = copy_assign_indices_cb<T>;
    copy_assign_compressed_ = copy_assign_compressed_cb<T>;
  }
  if constexpr (std::is_copy_constructible_v<T>) {
    copy_construct_ = copy_construct_cb<T>;
    copy_construct_indices_ = copy_construct_indices_cb<T>;
    copy_construct_compressed_ = copy_construct_compressed_cb<T>;
  }
  if constexpr (std::is_move_assignable_v<T>) {
    move_assign_ = move_assign_cb<T>;
    move_assign_indices_ = move_assign_indices_cb<T>;
  }
  if constexpr (std::is_move_constructible_v<T>) {
    move_construct_ = move_construct_cb<T>;
    move_construct_indices_ = move_construct_indices_cb<T>;
  }
  if constexpr (std::is_destructible_v<T>) {
    if constexpr (std::is_move_assignable_v<T>) {
      relocate_assign_ = relocate_assign_cb<T>;
      relocate_assign_indices_ = relocate_assign_indices_cb<T>;
    }
    if constexpr (std::is_move_constructible_v<T>) {
      relocate_construct_ = relocate_construct_cb<T>;
      relocate_construct_indices_ = relocate_construct_indices_cb<T>;
    }
  }
  if constexpr (std::is_copy_assignable_v<T>) {
    fill_assign_indices_ = fill_assign_indices_cb<T>;
  }
  if constexpr (std::is_copy_constructible_v<T>) {
    fill_construct_indices_ = fill_construct_indices_cb<T>;
  }
  if constexpr ((bool)(Flags & CPPTypeFlags::Hashable)) {
    hash_ = hash_cb<T>;
  }
  if constexpr ((bool)(Flags & CPPTypeFlags::Printable)) {
    print_ = print_cb<T>;
  }
  if constexpr ((bool)(Flags & CPPTypeFlags::EqualityComparable)) {
    is_equal_ = is_equal_cb<T>;
  }

  alignment_mask_ = uintptr_t(alignment_) - uintptr_t(1);
  has_special_member_functions_ = (default_construct_ && copy_construct_ && copy_assign_ &&
                                   move_construct_ && move_assign_ && destruct_);
}

}  // namespace blender

/** Create a new #CPPType that can be accessed through `CPPType::get<T>()`. */
#define BLI_CPP_TYPE_MAKE(TYPE_NAME, FLAGS) \
  template<> const blender::CPPType &blender::CPPType::get_impl<TYPE_NAME>() \
  { \
    static CPPType type{blender::TypeTag<TYPE_NAME>(), \
                        TypeForValue<CPPTypeFlags, FLAGS>(), \
                        STRINGIFY(TYPE_NAME)}; \
    return type; \
  }

/** Register a #CPPType created with #BLI_CPP_TYPE_MAKE. */
#define BLI_CPP_TYPE_REGISTER(TYPE_NAME) blender::CPPType::get<TYPE_NAME>()
