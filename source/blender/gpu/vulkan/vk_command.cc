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
}

}  // namespace blender::gpu
