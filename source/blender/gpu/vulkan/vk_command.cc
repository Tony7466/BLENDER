/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command.hh"

namespace blender::gpu {
VKCommand::~VKCommand()
{
  if (type == Type::CopyImageToBuffer) {
    MEM_delete(copy_image_to_buffer.regions);
  }
  if (type == Type::CopyBufferToImage) {
    MEM_delete(copy_buffer_to_image.regions);
  }
  if (type == Type::CopyImage) {
    MEM_delete(copy_image.regions);
  }
  if (type == Type::BlitImage) {
    MEM_delete(blit_image.regions);
  }
  if (type == Type::ClearColorImage) {
    MEM_delete(clear_color_image.ranges);
  }
  if (type == Type::ClearAttachments) {
    MEM_delete(clear_attachments.attachments);
    MEM_delete(clear_attachments.areas);
  }
  if (type == Type::PipelineImageMemoryBarrier) {
    MEM_delete(pipeline_image_memory_barrier.image_memory_barriers);
  }
}

}  // namespace blender::gpu
