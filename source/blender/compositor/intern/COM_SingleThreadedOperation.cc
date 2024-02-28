/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "COM_SingleThreadedOperation.h"

namespace blender::compositor {

SingleThreadedOperation::SingleThreadedOperation()
{
  cached_instance_ = nullptr;
  flags_.complex = true;
  flags_.single_threaded = true;
}

void SingleThreadedOperation::init_execution()
{
  init_mutex();
}

void SingleThreadedOperation::deinit_execution()
{
  deinit_mutex();
  if (cached_instance_) {
    delete cached_instance_;
    cached_instance_ = nullptr;
  }
}

}  // namespace blender::compositor
