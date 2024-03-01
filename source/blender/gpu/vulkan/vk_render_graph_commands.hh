/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {
class VKRenderGraphCommandBuffer {
 public:
  virtual void begin_recording() = 0;
  virtual void end_recording() = 0;
  virtual void pipeline_barrier(VkPipelineStageFlags src_stage_mask,
                                VkPipelineStageFlags dst_stage_mask,
                                VkDependencyFlags dependency_flags,
                                uint32_t memory_barrier_count,
                                const VkMemoryBarrier *p_memory_barriers,
                                uint32_t buffer_memory_barrier_count,
                                const VkBufferMemoryBarrier *p_buffer_memory_barriers,
                                uint32_t image_memory_barrier_count,
                                const VkImageMemoryBarrier *p_image_memory_barriers) = 0;
  virtual void clear_color_image(VkImage image,
                                 VkImageLayout image_layout,
                                 const VkClearColorValue *p_color,
                                 uint32_t range_count,
                                 const VkImageSubresourceRange *p_ranges) = 0;
};

class VKCommandBufferWrapper : public VKRenderGraphCommandBuffer {
 public:
  void begin_recording() override;
  void end_recording() override;
  void pipeline_barrier(VkPipelineStageFlags src_stage_mask,
                        VkPipelineStageFlags dst_stage_mask,
                        VkDependencyFlags dependency_flags,
                        uint32_t memory_barrier_count,
                        const VkMemoryBarrier *p_memory_barriers,
                        uint32_t buffer_memory_barrier_count,
                        const VkBufferMemoryBarrier *p_buffer_memory_barriers,
                        uint32_t image_memory_barrier_count,
                        const VkImageMemoryBarrier *p_image_memory_barriers) override;
  virtual void clear_color_image(VkImage image,
                                 VkImageLayout image_layout,
                                 const VkClearColorValue *p_color,
                                 uint32_t range_count,
                                 const VkImageSubresourceRange *p_ranges) override;
};

}  // namespace blender::gpu
