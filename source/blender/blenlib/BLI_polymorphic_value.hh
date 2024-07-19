/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_any.hh"

namespace blender {

namespace detail {

template<typename BaseT> struct PolymorphicValueAnyExtraInfo {
  const BaseT *(*get_ptr)(const void *buffer);

  template<typename StorageT> static constexpr PolymorphicValueAnyExtraInfo get()
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

template<typename BaseT> class PolymorphicValue {
 private:
  using Storage = Any<blender::detail::PolymorphicValueAnyExtraInfo<BaseT>, 24, 8>;

  const BaseT *ptr_ = nullptr;
  Storage storage_;

 public:
  PolymorphicValue() = default;

  PolymorphicValue(const PolymorphicValue &other) : storage_(other.storage_)
  {
    ptr_ = this->ptr_from_storage();
  }

  PolymorphicValue(PolymorphicValue &&other) noexcept : storage_(std::move(other.storage_))
  {
    ptr_ = this->ptr_from_storage();
    other.storage_.reset();
    other.ptr_ = nullptr;
  }

  PolymorphicValue(const BaseT *ptr) : ptr_(ptr), storage_(ptr) {}

  PolymorphicValue(std::shared_ptr<const BaseT> ptr) : ptr_(ptr.get())
  {
    if (ptr) {
      storage_ = std::move(ptr);
    }
  }

  PolymorphicValue &operator=(const PolymorphicValue &other)
  {
    if (this != &other) {
      std::destroy_at(this);
      new (this) PolymorphicValue(other);
    }
    return *this;
  }

  PolymorphicValue &operator=(PolymorphicValue &&other)
  {
    if (this != &other) {
      std::destroy_at(this);
      new (this) PolymorphicValue(std::move(other));
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
