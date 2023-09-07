/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include "BLI_string_ref.hh"
#include "BLI_linear_allocator.hh"

namespace blender {

struct FlagInfo {
  void **flag;
  LinearAllocator<GuardedAllocator> &allocator;
};

FlagInfo singleton_prt(const StringRef name);

template<typename Flag, typename... Args>
inline Flag &singleton(const StringRef name, Args &&...args)
{
  FlagInfo flag_info = singleton_prt(name);
  if (*flag_info.flag == nullptr) {
    *flag_info.flag = flag_info.allocator.allocate(sizeof(Flag), alignof(Flag));
    new (*flag_info.flag) Flag(std::forward<Args>(args)...);
  }
  return *static_cast<Flag *>(*flag_info.flag);
}


}