/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>
#include <type_traits>
#include <utility>

#include "BLI_utildefines.h"

/** \file
 * \ingroup bli
 *
 * A `FunctionRef<Signature>` is a non-owning reference to some callable object with a specific
 * signature. It can be used to pass some callback to another function.
 *
 * A `FunctionRef` is small and cheap to copy. Therefore it should generally be passed by value.
 *
 * Example signatures:
 *   `FunctionRef<void()>`        - A function without parameters and void return type.
 *   `FunctionRef<int(float)>`    - A function with a float parameter and an int return value.
 *   `FunctionRef<int(int, int)>` - A function with two int parameters and an int return value.
 *
 * There are multiple ways to achieve that, so here is a comparison of the different approaches:
 * 1. Pass function pointer and user data (as void *) separately:
 *    - The only method that is compatible with C interfaces.
 *    - Is cumbersome to work with in many cases, because one has to keep track of two parameters.
 *    - Not type safe at all, because of the void pointer.
 *    - It requires workarounds when one wants to pass a lambda into a function.
 * 2. Using `std::function`:
 *    - It works well with most callables and is easy to use.
 *    - Owns the callable, so it can be returned from a function more safely than other methods.
 *    - Requires that the callable is copyable.
 *    - Requires an allocation when the callable is too large (typically > 16 bytes).
 * 3. Using a template for the callable type:
 *    - Most efficient solution at runtime, because compiler knows the exact callable at the place
 *      where it is called.
 *    - Works well with all callables.
 *    - Requires the function to be in a header file.
 *    - It's difficult to constrain the signature of the function.
 * 4. Using `FunctionRef`:
 *    - Second most efficient solution at runtime.
 *    - It's easy to constrain the signature of the callable.
 *    - Does not require the function to be in a header file.
 *    - Works well with all callables.
 *    - It's a non-owning reference, so it *cannot* be stored safely in general.
 *
 * The fact that this is a non-owning reference makes `FunctionRef` very well suited for some use
 * cases, but one has to be a bit more careful when using it to make sure that the referenced
 * callable is not destructed.
 *
 * In particular, one must not construct a `FunctionRef` variable from a lambda directly as shown
 * below. This is because the lambda object goes out of scope after the line finished executing and
 * will be destructed. Calling the reference afterwards invokes undefined behavior. Also see
 * #StoredFunctionRef for how to protect yourself against this in some cases.
 *
 * Don't:
 *   FunctionRef<int()> ref = []() { return 0; };
 * Do:
 *   auto f = []() { return 0; };
 *   FuntionRef<int()> ref = f;
 *
 * It is fine to pass a lambda directly to a function:
 *
 *   void some_function(FunctionRef<int()> f);
 *   some_function([]() { return 0; });
 */

#include "BLI_memory_utils.hh"

namespace blender {

template<typename Function> class FunctionRef;

template<typename Ret, typename... Params> class FunctionRef<Ret(Params...)> {
 private:
  /**
   * A function pointer that knows how to call the referenced callable with the given parameters.
   */
  Ret (*callback_)(intptr_t callable, Params... params) = nullptr;

  /**
   * A pointer to the referenced callable object. This can be a C function, a lambda object or any
   * other callable.
   *
   * The value does not need to be initialized because it is not used unless `callback_` is set as
   * well, in which case it will be initialized as well.
   *
   * Use `intptr_t` to avoid warnings when casting to function pointers.
   */
  intptr_t callable_;

  template<typename Callable> static Ret callback_fn(intptr_t callable, Params... params)
  {
    return (*reinterpret_cast<Callable *>(callable))(std::forward<Params>(params)...);
  }

 public:
  FunctionRef() = default;

  FunctionRef(std::nullptr_t) {}

  /**
   * A `FunctionRef` itself is a callable as well. However, we don't want that this
   * constructor is called when `Callable` is a `FunctionRef`. If we would allow this, it
   * would be easy to accidentally create a `FunctionRef` that internally calls another
   * `FunctionRef`. Usually, when assigning a `FunctionRef` to another, we want that both
   * contain a reference to the same underlying callable afterwards.
   *
   * It is still possible to reference another `FunctionRef` by first wrapping it in
   * another lambda.
   */
  template<typename Callable,
           BLI_ENABLE_IF((
               !std::is_same_v<std::remove_cv_t<std::remove_reference_t<Callable>>, FunctionRef>)),
           BLI_ENABLE_IF((std::is_invocable_r_v<Ret, Callable, Params...>))>
  FunctionRef(Callable &&callable)
      : callback_(callback_fn<typename std::remove_reference_t<Callable>>),
        callable_(intptr_t(&callable))
  {
  }

  /**
   * Call the referenced function and forward all parameters to it.
   *
   * This invokes undefined behavior if the `FunctionRef` does not reference a function currently.
   */
  Ret operator()(Params... params) const
  {
    BLI_assert(callback_ != nullptr);
    return callback_(callable_, std::forward<Params>(params)...);
  }

  /**
   * Returns true, when the `FunctionRef` references a function currently.
   * If this returns false, the `FunctionRef` must not be called.
   */
  operator bool() const
  {
    /* Just checking `callback_` is enough to determine if the `FunctionRef` is in a state that it
     * can be called in. */
    return callback_ != nullptr;
  }
};

/**
 * This should be used instead of `FunctionRef` in a parameter list if the function-ref has to
 * outlive the current function. This is typically necessary if the function-ref is stored as
 * data-member when in the value returned by the function.
 *
 * Using #StoredFunctionRef does not offer perfect protection and may also disable some valid uses
 * which are pretty much impossible to detect automatically. However, it protects against the most
 * common issue.
 *
 * ```
 * class S {
 *   StoredFunctionRef<int()> f_;
 *
 *   S(StoredFunctionRef<int()> f) : f_(f) {}
 * }
 *
 * // This does not compile. It would lead to hard-to-find bugs, because #S::f_ would point to
 * // invalid memory. This can sometimes work accidentally, sometimes even only in debug but not
 * // release builds!
 * const S value{[#]() { return 42; }};
 *
 * // This works fine though.
 * auto func = [#]() { return 42; };
 * const S value{func};
 * ```
 */
template<typename Function> class StoredFunctionRef;
template<typename Ret, typename... Params>
class StoredFunctionRef<Ret(Params...)> : public FunctionRef<Ret(Params...)> {
 public:
  StoredFunctionRef() = default;
  StoredFunctionRef(std::nullptr_t) {}

  template<typename Callable>
  StoredFunctionRef(Callable &&callable)
      : FunctionRef<Ret(Params...)>(std::forward<Callable>(callable))
  {
    /* Check `is_reference_v` here only allows passing in l-value references. If the caller passes
     * in an r-value reference (e.g. a lambda or other callable directly) `Callable` will not be a
     * reference because of the `&&` in the parameter list. */
    static_assert(std::is_reference_v<Callable>,
                  "Passing in a lambda (or other function object) directly here does not work "
                  "because its life-time is not long enough. Store it in a separate variable "
                  "first and pass in a reference.");
  }
};

}  // namespace blender
