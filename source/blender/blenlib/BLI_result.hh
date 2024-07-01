/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_assert.h"
#include <utility>
#include <variant>

namespace blender {

template<typename> class Err;

template<typename V, typename E> class Result {
  std::variant<V, Err<E>> data;

 public:
  Result() = delete;

  Result(V &value) : data{value} {}
  Result(V &&value) : data{std::move(value)} {}
  template<typename T> Result(T &&value) : data{std::forward<T>(value)} {}

  Result(Err<E> &error) : data{error} {}
  Result(Err<E> &&error) : data{std::move(error)} {}
  template<typename T> Result(Err<T> &&error) : data{Err<E>{std::forward<Err<T>>(error)}} {}

  bool is_value() const
  {
    return this->data.index() == 0;
  }

  bool is_error() const
  {
    return this->data.index() == 1;
  }

  V &value()
  {
    BLI_assert(this->is_value());
    return *std::get_if<0>(&this->data);
  }
  const V &value() const
  {
    BLI_assert(this->is_value());
    return *std::get_if<0>(&this->data);
  }

  V value_or(V &&default_value)
  {
    if (this->is_value()) {
      return std::move(this->value());
    }
    return std::forward<V>(default_value);
  }

  E &error()
  {
    BLI_assert(this->is_error());
    return std::get_if<1>(&this->data)->error;
  }
  const E &error() const
  {
    BLI_assert(this->is_error());
    return std::get_if<1>(&this->data)->error;
  }

  E error_or(E &&default_error)
  {
    if (this->is_error()) {
      return std::move(this->error());
    }
    return std::forward<E>(default_error);
  }
};

template<typename E> class Err {
  template<typename X> friend class Err;
  template<typename X, typename Y> friend class Result;

  E error;

 public:
  Err() = delete;

  explicit Err(E &error) : error{error} {}
  explicit Err(E &&error) : error{std::move(error)} {}
  template<typename T> explicit Err(T &&error) : error{std::forward<T>(error)} {}

  template<typename T> Err(Err<T> &&error) : error{std::forward<T>(error.error)} {}
};

/* The compiler needs this deduction guide for `Err` to properly coerce string
 * literals to its inner type. Otherwise things like this fail to compile:
 *
 * ```
 * Err<std::string> a = Err{"bye"};
 * Result<int, blender::StringRef> b = Err{"bye"};
 * ```
 */
Err(const char[]) -> Err<const char *>;

}  // namespace blender
