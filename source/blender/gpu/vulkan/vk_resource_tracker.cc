/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_resource_tracker.hh"
#include "vk_context.hh"

namespace blender::gpu {

SubmissionTracker::Result SubmissionTracker::submission_tracker_pre_update(VKContext &context,
                                                                           const bool is_dirty)
{
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  const SubmissionID &current_id = command_buffer.submission_id_get();
  if (last_known_id_ != current_id) {
    last_known_id_ = current_id;
    return Result::FREE_AND_CREATE_NEW_RESOURCE;
  }
  if (is_dirty) {
    return Result::CREATE_NEW_RESOURCE;
  }
  return Result::USE_LAST_RESOURCE;
}

}  // namespace blender::gpu
