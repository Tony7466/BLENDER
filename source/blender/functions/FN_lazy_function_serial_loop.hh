/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <iostream>

namespace blender::fn::lazy_function {

enum class SerialLoopStatus {
  /** Use the outputs from the previous iteration. */
  UsePrevious,
  /** Use the outputs from the current iteration. */
  UseCurrent,
  /** The final outputs are not computed yet, continue. */
  Continue,
};
std::ostream &operator<<(std::ostream &stream, SerialLoopStatus status);

}  // namespace blender::fn::lazy_function
