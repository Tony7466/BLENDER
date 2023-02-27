/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_resource_tracker.hh"
#include "vk_context.hh"

namespace blender::gpu {
bool SubmissionTracker::is_changed(VKContext &context)
{
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  const SubmissionID &current_id = command_buffer.submission_id_get();
  if (last_known_id_ != current_id) {
    last_known_id_ = current_id;
    return true;
  }
  return false;
}

}  // namespace blender::gpu
