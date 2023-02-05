/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_copy_on_write.hh"

namespace blender {

template<typename T> class COWUser {
 private:
  T *data_ = nullptr;

 public:
  COWUser() = default;

  COWUser(T *data) : data_(data)
  {
  }

  COWUser(const COWUser &other) : data_(other.data_)
  {
    this->add_user(data_);
  }

  COWUser(COWUser &&other) : data_(other.data_)
  {
    other.data_ = nullptr;
  }

  ~COWUser()
  {
    this->remove_user_and_delete_if_last(data_);
  }

  COWUser &operator=(const COWUser &other)
  {
    if (this == &other) {
      return *this;
    }

    this->remove_user_and_delete_if_last(data_);
    data_ = other.data_;
    this->add_user(data_);
    return *this;
  }

  COWUser &operator=(COWUser &&other)
  {
    if (this == &other) {
      return *this;
    }

    this->remove_user_and_delete_if_last(data_);
    data_ = other.data_;
    other.data_ = nullptr;
    return *this;
  }

  T *operator->()
  {
    BLI_assert(data_ != nullptr);
    return data_;
  }

  const T *operator->() const
  {
    BLI_assert(data_ != nullptr);
    return data_;
  }

  T &operator*()
  {
    BLI_assert(data_ != nullptr);
    return *data_;
  }

  const T &operator*() const
  {
    BLI_assert(data_ != nullptr);
    return *data_;
  }

  operator bool() const
  {
    return data_ != nullptr;
  }

  T *get()
  {
    return data_;
  }

  const T *get() const
  {
    return data_;
  }

  T *release()
  {
    T *data = data_;
    data_ = nullptr;
    return data;
  }

  void reset()
  {
    this->remove_user_and_delete_if_last(data_);
    data_ = nullptr;
  }

  bool has_value() const
  {
    return data_ != nullptr;
  }

  uint64_t hash() const
  {
    return get_default_hash(data_);
  }

  friend bool operator==(const COWUser &a, const COWUser &b)
  {
    return a.data_ == b.data_;
  }

 private:
  static void add_user(T *data)
  {
    if (data != nullptr) {
      data->add_user();
    }
  }

  static void remove_user_and_delete_if_last(T *data)
  {
    if (data != nullptr) {
      data->remove_user_and_delete_if_last();
    }
  }
};

}  // namespace blender
