/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_commands.hh"

namespace blender::gpu {

void VKCommandBufferWrapper::clear_color_image(VkImage image,
                                               VkImageLayout image_layout,
                                               const VkClearColorValue *p_color,
                                               uint32_t range_count,
                                               const VkImageSubresourceRange *p_ranges)
{
  BLI_assert_unreachable();
}

void VKCommandBufferWrapper::pipeline_barrier(
    VkPipelineStageFlags src_stage_mask,
    VkPipelineStageFlags dst_stage_mask,
    VkDependencyFlags dependency_flags,
    uint32_t memory_barrier_count,
    const VkMemoryBarrier *p_memory_barriers,
    uint32_t buffer_memory_barrier_count,
    const VkBufferMemoryBarrier *p_buffer_memory_barriers,
    uint32_t image_memory_barrier_count,
    const VkImageMemoryBarrier *p_image_memory_barriers)
{
  BLI_assert_unreachable();
}

}  // namespace blender::gpu
