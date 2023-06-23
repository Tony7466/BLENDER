/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "FN_lazy_function_serial_loop.hh"

#include "BLI_utildefines.h"

namespace blender::fn::lazy_function {

std::ostream &operator<<(std::ostream &stream, const SerialLoopStatus status)
{
  switch (status) {
    case SerialLoopStatus::UsePrevious:
      return stream << "UsePrevious";
    case SerialLoopStatus::UseCurrent:
      return stream << "UseCurrent";
    case SerialLoopStatus::Continue:
      return stream << "Continue";
  }
  BLI_assert_unreachable();
  return stream;
}

}  // namespace blender::fn::lazy_function
