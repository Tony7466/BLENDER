/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include <iostream>

#include "BLI_any.hh"
#include "BLI_map.hh"
#include "BLI_string_ref.hh"

namespace blender::singleton::detail {

Any<> &singleton_ptr_for_write(const StringRef name)
{
  static Map<StringRef, Any<>> runtime_values;
  return runtime_values.lookup_or_add(name, {});
}

void panic_non_debug(const StringRef name)
{
  std::cout << __func__ << ": Used non initialized singleton value for key: " << name << std::endl;
}
}  // namespace blender::singleton::detail