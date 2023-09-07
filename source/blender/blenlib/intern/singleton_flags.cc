/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_map.hh"
#include "BLI_string_ref.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_singleton_flags.hh"

namespace blender {

FlagInfo singleton_prt(const StringRef name)
{
  static Map<StringRef, void *> flags;
  static LinearAllocator<> allocator;
  return FlagInfo{&flags.lookup_or_add(name, nullptr), allocator};
}

}