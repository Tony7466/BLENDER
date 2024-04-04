/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "../vk_command_buffer_wrapper.hh"
#include "vk_common.hh"

namespace blender::gpu::render_graph {
struct VKBlitImageNode : NonCopyable {
  static constexpr bool uses_image_resources = true;
  static constexpr bool uses_buffer_resources = false;
  static constexpr VkPipelineStageFlags pipeline_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;

  /**
   * Information stored inside the render graph node.
   */
  struct Data {
    VkImage src_image;
    VkImage dst_image;
    VkImageBlit region;
    VkFilter filter;
  };

  static void build_commands(VKCommandBufferInterface &command_buffer, const Data &data)
  {
    command_buffer.blit_image(data.src_image,
                              VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                              data.dst_image,
                              VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                              1,
                              &data.region,
                              data.filter);
  }
};
}  // namespace blender::gpu::render_graph
