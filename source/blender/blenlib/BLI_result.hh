/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file \ingroup bli
 *
 * `blender::Result` is used for doing error handling explicitly via return
 * values. `Result` can be either a value `V` *or* an error `E`, but never both
 * at once. Its intended use is as the return type for functions that may
 * successfully return a value or may fail and return an error.
 *
 * ----
 * TODO: some things that should be decided on and written up, either here or in
 * the developer docs:
 *   - Guidelines for what an error type should look like.
 *   - Guidelines for when to use `Result` vs other error strategies like
 *     `BLI_assert`.
 *
 * There was discussion on both those topics in
 * https://devtalk.blender.org/t/developer-discussion-handling-errors/33054
 * ----
 *
 * As a simple example, let's say you want to write a function that parses a
 * string to an integer. Using `Result`, we can write the function so it either
 * returns an integer on success, or an enum indicating the cause of failure:
 *
 * ```
 * enum class ParseError {
 *   EmptyString,
 *   ContainsDecimalPoint,
 *   Other,
 * };
 *
 * Result<int, ParseError> parse_int(blender::StringRef text) {
 *   if (text.is_empty()) {
 *     return Err{ParseError::EmptyString};
 *   }
 *   if (text.find(".")) {
 *     return Err{ParseError::ContainsDecimalPoint};
 *   }
 *
 *   int parsed_int;
 *   // Code to actually parse the int omitted since this is just an example.
 *
 *   return parsed_int;
 * }
 * ```
 *
 * The return type in this case is `Result<int, ParseError>`, indicating that it
 * can either be a successfully parsed `int` or, upon failure, a `ParseError`.
 *
 * In the body of the function, the success case simply returns an `int`, which
 * auto-converts to a `Result` containing the `int` as a successful value. The
 * error cases return the error values wrapped in an `Err`, which auto-convert
 * to a `Result` containing those errors.
 *
 * The purpose of the `Err` wrapper is to make the error cases visually clear in
 * the code, and also to disambiguate values vs errors for the compiler (e.g. if
 * the value and error types are the same, or if those types can auto-convert
 * from one to the other).
 *
 * Now let's say we want to use the function we wrote above.  Here are a few
 * different ways you can do that, depending on the situation:
 *
 * ```
 * // Full handling.
 * Result<int, ParseError> i = parse_int(maybe_an_int_string);
 * if (i.is_value()) {
 *   printf("%d\n", i.value());
 * }
 * else {
 *   switch i.error() {
 *     ParseError::EmptyString:
 *       printf("Nothing to parse!\n");
 *       break;
 *     ParseError::ContainsDecimalPoint:
 *       printf("Fractional values not supported!\n");
 *       break;
 *     ParseError::Other:
 *       printf("Something else went wrong!\n");
 *       break;
 *   }
 * }
 *
 * // Propagate the error up the call chain (requires this function's return
 * // type to have a compatible error type.)
 * Result<int, ParseError> i = parse_int(maybe_an_int_string);
 * if (i.is_error()) {
 *   return Err{i.error()};
 * }
 * printf("%d\n", i.value());
 *
 * // Use a fallback of 5 in case of error.
 * int i = parse_int(maybe_an_int_string).value_or(5);
 * printf("%d\n", i.value());
 *
 * // If you know it cannot be an error.
 * // The `value()` method has a `BLI_assert` internally, so you don't
 * // need to write it yourself. In release builds this is undefined
 * // behavior if it turns out to be an error after all.
 * int i = parse_int("123").value();
 * printf("%d\n", i.value());
 * ```
 */

#include "BLI_assert.h"
#include <utility>
#include <variant>

namespace blender {

template<typename> class Err;

/**
 * A result that can either be a value on success or an error on failure.
 */
template<typename V, typename E> class Result {
  std::variant<V, Err<E>> data_;

 public:
  Result() = delete;

  Result(V &value) : data_{value} {}
  Result(V &&value) : data_{std::move(value)} {}
  template<typename T> Result(T &&value) : data_{std::forward<T>(value)} {}

  Result(Err<E> &error) : data_{error} {}
  Result(Err<E> &&error) : data_{std::move(error)} {}
  template<typename T> Result(Err<T> &&error) : data_{Err<E>{std::forward<Err<T>>(error)}} {}

  /**
   * Return whether the result contains a successful value.
   */
  bool is_value() const
  {
    return std::holds_alternative<V>(this->data_);
  }

  /**
   * Return whether the result contains an error value.
   */
  bool is_error() const
  {
    return std::holds_alternative<Err<E>>(this->data_);
  }

  /**
   * Return the success value contained by the result.
   *
   * In debug builds this asserts that the result contains a success value. In
   * release builds this is undefined behavior if the result contains an error.
   */
  V &value()
  {
    BLI_assert(this->is_value());
    return std::get<V>(this->data_);
  }
  const V &value() const
  {
    BLI_assert(this->is_value());
    return std::get<V>(this->data_);
  }

  /**
   * Return the success value contained in the result, or else a default value
   * if the result contains an error.
   *
   * Note that this moves the success value out of the result if there is one.
   */
  V value_or(V &&default_value)
  {
    if (this->is_value()) {
      return std::move(this->value());
    }
    return std::forward<V>(default_value);
  }

  /**
   * Return the error value contained by the result.
   *
   * In debug builds this asserts that the result contains an error.  In release
   * builds this is undefined behavior if the result contains a success value.
   */
  E &error()
  {
    BLI_assert(this->is_error());
    return std::get<Err<E>>(this->data_).error_;
  }
  const E &error() const
  {
    BLI_assert(this->is_error());
    return std::get<Err<E>>(this->data_).error_;
  }

  /**
   * Return the error contained in the result, or else a default error if the
   * result contains a success value.
   *
   * Note that this moves the error out of the result if there is one.
   */
  E error_or(E &&default_error)
  {
    if (this->is_error()) {
      return std::move(this->error());
    }
    return std::forward<E>(default_error);
  }
};

/**
 * A simple wrapper type used to construct a `Result` with an error value.
 */
template<typename E> class Err {
  template<typename X> friend class Err;
  template<typename X, typename Y> friend class Result;

  E error_;

 public:
  Err() = delete;

  explicit Err(E &error) : error_{error} {}
  explicit Err(E &&error) : error_{std::move(error)} {}
  template<typename T> explicit Err(T &&error) : error_{std::forward<T>(error)} {}

  /* Implicitly convert an `Err<A>` to an `Err<B>` if `A` can be implicitly
   * converted to a `B`.  This is needed for things like this to work:
   *
   * ```
   * Result<int, std::string> r = Err{"bye"};
   * ```
   */
  template<typename T> Err(Err<T> &&error) : error_{std::forward<T>(error.error_)} {}
};

/* The compiler needs this deduction guide to properly coerce string literals to
 * an `Err`s inner type. Otherwise things like this fail to compile:
 *
 * ```
 * Err<std::string> a = Err{"bye"};
 * Result<int, blender::StringRef> b = Err{"bye"};
 * ```
 */
Err(const char[]) -> Err<const char *>;

}  // namespace blender
