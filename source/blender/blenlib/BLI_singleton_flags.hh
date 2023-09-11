/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_any.hh"
#include "BLI_assert.h"
#include "BLI_string_ref.hh"

namespace blender::singleton {

namespace detail {

Any<> &singleton_ptr_for_write(const StringRef name);

void panic_non_debug(const StringRef name);
}  // namespace detail

template<typename T, typename... Args>
inline T &init_or_change(const StringRef name, Args &&...args)
{
  Any<> &value = detail::singleton_ptr_for_write(name);
  return value.emplace<T>(std::forward<Args>(args)...);
}

template<typename T> inline const T &find(const StringRef name)
{
  const Any<> &value = detail::singleton_ptr_for_write(name);
  if (!value.has_value()) {
    BLI_assert_unreachable();
    detail::panic_non_debug(name);
  }
  return value.get<T>();
}
}  // namespace blender::singleton