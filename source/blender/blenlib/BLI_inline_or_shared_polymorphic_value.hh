/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_any.hh"

namespace blender {

namespace detail {

template<typename BaseT> struct InlineOrSharedPolymorphicValueAnyExtraInfo {
  const BaseT *(*get_ptr)(const void *buffer);

  template<typename StorageT> static constexpr InlineOrSharedPolymorphicValueAnyExtraInfo get()
  {
    static_assert(std::is_base_of_v<BaseT, StorageT> ||
                  is_same_any_v<StorageT, const BaseT *, std::shared_ptr<const BaseT>>);
    if constexpr (std::is_base_of_v<BaseT, StorageT>) {
      return {[](const void *buffer) {
        return static_cast<const BaseT *>(static_cast<const StorageT *>(buffer));
      }};
    }
    else if constexpr (std::is_same_v<StorageT, const BaseT *>) {
      return {[](const void *buffer) { return *static_cast<const StorageT *>(buffer); }};
    }
    else if constexpr (std::is_same_v<StorageT, std::shared_ptr<const BaseT>>) {
      return {[](const void *buffer) { return static_cast<const StorageT *>(buffer)->get(); }};
    }
    else {
      BLI_assert_unreachable();
      return {};
    }
  }
};
}  // namespace detail

template<typename BaseT> class InlineOrSharedPolymorphicValue {
 private:
  using Storage = Any<blender::detail::InlineOrSharedPolymorphicValueAnyExtraInfo<BaseT>, 24, 8>;

  const BaseT *ptr_ = nullptr;
  Storage storage_;

 public:
  InlineOrSharedPolymorphicValue() = default;

  InlineOrSharedPolymorphicValue(const InlineOrSharedPolymorphicValue &other)
      : storage_(other.storage_)
  {
    ptr_ = this->ptr_from_storage();
  }

  InlineOrSharedPolymorphicValue(InlineOrSharedPolymorphicValue &&other) noexcept
      : storage_(std::move(other.storage_))
  {
    ptr_ = this->ptr_from_storage();
    other.storage_.reset();
    other.ptr_ = nullptr;
  }

  InlineOrSharedPolymorphicValue(const BaseT *ptr) : ptr_(ptr), storage_(ptr) {}

  InlineOrSharedPolymorphicValue(std::shared_ptr<const BaseT> ptr) : ptr_(ptr.get())
  {
    if (ptr) {
      storage_ = std::move(ptr);
    }
  }

  InlineOrSharedPolymorphicValue &operator=(const InlineOrSharedPolymorphicValue &other)
  {
    if (this != &other) {
      std::destroy_at(this);
      new (this) InlineOrSharedPolymorphicValue(other);
    }
    return *this;
  }

  InlineOrSharedPolymorphicValue &operator=(InlineOrSharedPolymorphicValue &&other)
  {
    if (this != &other) {
      std::destroy_at(this);
      new (this) InlineOrSharedPolymorphicValue(std::move(other));
    }
    return *this;
  }

  template<typename T, typename... Args> void emplace(Args &&...args)
  {
    static_assert(std::is_base_of_v<BaseT, T>);
    if constexpr (std::is_copy_constructible_v<T> && Storage::template is_inline_v<T>) {
      ptr_ = &storage_.template emplace<T>(std::forward<Args>(args)...);
    }
    else {
      std::shared_ptr<const BaseT> ptr = std::make_shared<T>(std::forward<Args>(args)...);
      ptr_ = &*ptr;
      storage_ = std::move(ptr);
    }
  }

  operator bool() const
  {
    return ptr_ != nullptr;
  }

  const BaseT *ptr() const
  {
    return ptr_;
  }

  BaseT *ptr()
  {
    return const_cast<BaseT *>(ptr_);
  }

  const BaseT *operator->() const
  {
    return this->ptr();
  }

  BaseT *operator->()
  {
    return this->ptr();
  }

 private:
  const BaseT *ptr_from_storage() const
  {
    if (storage_.has_value()) {
      return storage_.extra_info().get_ptr(storage_.get());
    }
    return nullptr;
  }
};

}  // namespace blender
