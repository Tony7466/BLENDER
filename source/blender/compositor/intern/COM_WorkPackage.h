/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_CXX_GUARDEDALLOC
#  include "MEM_guardedalloc.h"
#endif

#include "COM_Enums.h"

#include "DNA_vec_types.h"

#include <functional>
#include <ostream>

namespace blender::compositor {

/**
 * \brief contains data about work that can be scheduled
 * \see WorkScheduler
 */
struct WorkPackage {
  /**
   * \brief number of the chunk to be executed
   */
  unsigned int chunk_number;

  /**
   * Area of the execution group that the work package calculates.
   */
  rcti rect;

  /**
   * Custom function to execute when work package type is CustomFunction.
   */
  std::function<void()> execute_fn;

  /**
   * Called when work execution is finished.
   */
  std::function<void()> executed_fn;

#ifdef WITH_CXX_GUARDEDALLOC
  MEM_CXX_CLASS_ALLOC_FUNCS("COM:WorkPackage")
#endif
};

std::ostream &operator<<(std::ostream &os, const WorkPackage &work_package);

}  // namespace blender::compositor
