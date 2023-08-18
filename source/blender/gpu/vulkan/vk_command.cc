/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include <sstream>

#include "vk_command.hh"

namespace blender::gpu {

void VKCommand::free()
{
  switch (type) {
    case Type::CopyImageToBuffer:
      MEM_delete(copy_image_to_buffer.regions);
      break;
    case Type::CopyBufferToImage:
      MEM_delete(copy_buffer_to_image.regions);
      break;
    case Type::CopyImage:
      MEM_delete(copy_image.regions);
      break;
    case Type::BlitImage:
      MEM_delete(blit_image.regions);
      break;
    case Type::ClearColorImage:
      MEM_delete(clear_color_image.ranges);
      break;
    case Type::ClearAttachments:
      MEM_delete(clear_attachments.attachments);
      MEM_delete(clear_attachments.areas);
      break;
    case Type::PipelineImageMemoryBarrier:
      MEM_delete(pipeline_image_memory_barrier.image_memory_barriers);
      break;

    case Type::BindPipeline:
    case Type::BindDescriptorSet:
    case Type::BindVertexBuffer:
    case Type::BindIndexBuffer:
    case Type::BeginRenderPass:
    case Type::EndRenderPass:
    case Type::PushConstants:
    case Type::FillBuffer:
    case Type::Draw:
    case Type::DrawIndexed:
    case Type::PipelineBarrier:
    case Type::Dispatch:
    case Type::DispatchIndirect:
      break;
  }
}

std::ostream &operator<<(std::ostream &stream, const VKCommand &command)
{
  stream << command.type;
  stream << "(";
  switch (command.type) {
    case VKCommand::Type::PipelineImageMemoryBarrier: {
      for (const VkImageMemoryBarrier &barrier :
           *command.pipeline_image_memory_barrier.image_memory_barriers)
      {
        stream << "image:" << barrier.image << ", ";
        stream << "old_layout" << barrier.oldLayout << ", ";
        stream << "new_layout" << barrier.newLayout << ", ";
      }
      break;
    }
    default:
      break;
  }
  stream << ")";
  return stream;
}

std::ostream &operator<<(std::ostream &stream, const VKCommand::Type &command_type)
{
#define CASE(t) \
  case t: \
    stream << STRINGIFY(t); \
    break;

  switch (command_type) {
    CASE(VKCommand::Type::CopyImageToBuffer)
    CASE(VKCommand::Type::CopyBufferToImage)
    CASE(VKCommand::Type::CopyImage)
    CASE(VKCommand::Type::BlitImage)
    CASE(VKCommand::Type::ClearColorImage)
    CASE(VKCommand::Type::ClearAttachments)
    CASE(VKCommand::Type::PipelineImageMemoryBarrier)
    CASE(VKCommand::Type::BindPipeline)
    CASE(VKCommand::Type::BindDescriptorSet)
    CASE(VKCommand::Type::BindVertexBuffer)
    CASE(VKCommand::Type::BindIndexBuffer)
    CASE(VKCommand::Type::BeginRenderPass)
    CASE(VKCommand::Type::EndRenderPass)
    CASE(VKCommand::Type::PushConstants)
    CASE(VKCommand::Type::FillBuffer)
    CASE(VKCommand::Type::Draw)
    CASE(VKCommand::Type::DrawIndexed)
    CASE(VKCommand::Type::PipelineBarrier)
    CASE(VKCommand::Type::Dispatch)
    CASE(VKCommand::Type::DispatchIndirect)
  }

  return stream;

#undef CASE
}

}  // namespace blender::gpu
