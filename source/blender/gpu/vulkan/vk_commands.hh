/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {
class VKPipeline;
class VKDescriptorSet;
class VKVertexBuffer;
class VKBufferWithOffset;
class VKFrameBuffer;
class VKPushConstants;
class VKStorageBuffer;
class VKBuffer;
class VKTexture;

/**
 * Interface containing all supported commands so they can be shared between a single
 * #VKCommandBuffer and #VKCommandBuffers.
 */
class VKCommands {
  virtual void bind(const VKPipeline &vk_pipeline, VkPipelineBindPoint bind_point) = 0;
  virtual void bind(const VKDescriptorSet &descriptor_set,
                    const VkPipelineLayout vk_pipeline_layout,
                    VkPipelineBindPoint bind_point) = 0;
  virtual void bind(const uint32_t binding,
                    const VKVertexBuffer &vertex_buffer,
                    const VkDeviceSize offset) = 0;
  /* Bind the given buffer as a vertex buffer. */
  virtual void bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer) = 0;
  virtual void bind(const uint32_t binding,
                    const VkBuffer &vk_vertex_buffer,
                    const VkDeviceSize offset) = 0;
  /* Bind the given buffer as an index buffer. */
  virtual void bind(const VKBufferWithOffset &index_buffer, VkIndexType index_type) = 0;

  virtual void begin_render_pass(VKFrameBuffer &framebuffer) = 0;
  virtual void end_render_pass(const VKFrameBuffer &framebuffer) = 0;

  /**
   * Add a push constant command to the command buffer.
   *
   * Only valid when the storage type of push_constants is StorageType::PUSH_CONSTANTS.
   */
  virtual void push_constants(const VKPushConstants &push_constants,
                              const VkPipelineLayout vk_pipeline_layout,
                              const VkShaderStageFlags vk_shader_stages) = 0;
  virtual void dispatch(int groups_x_len, int groups_y_len, int groups_z_len) = 0;
  virtual void dispatch(VKStorageBuffer &command_buffer) = 0;
  /** Copy the contents of a texture MIP level to the dst buffer. */
  virtual void copy(VKBuffer &dst_buffer,
                    VKTexture &src_texture,
                    Span<VkBufferImageCopy> regions) = 0;
  virtual void copy(VKTexture &dst_texture,
                    VKBuffer &src_buffer,
                    Span<VkBufferImageCopy> regions) = 0;
  virtual void copy(VKTexture &dst_texture, VKTexture &src_texture, Span<VkImageCopy> regions) = 0;
  virtual void copy(VKBuffer &dst_buffer, VkBuffer src_buffer, Span<VkBufferCopy> regions) = 0;
  virtual void blit(VKTexture &dst_texture, VKTexture &src_texture, Span<VkImageBlit> regions) = 0;
  virtual void blit(VKTexture &dst_texture,
                    VkImageLayout dst_layout,
                    VKTexture &src_texture,
                    VkImageLayout src_layout,
                    Span<VkImageBlit> regions) = 0;
  virtual void pipeline_barrier(VkPipelineStageFlags source_stages,
                                VkPipelineStageFlags destination_stages) = 0;
  virtual void pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers) = 0;

  /**
   * Clear color image resource.
   */
  virtual void clear(VkImage vk_image,
                     VkImageLayout vk_image_layout,
                     const VkClearColorValue &vk_clear_color,
                     Span<VkImageSubresourceRange> ranges) = 0;

  /**
   * Clear depth/stencil aspect of an image resource.
   */
  virtual void clear(VkImage vk_image,
                     VkImageLayout vk_image_layout,
                     const VkClearDepthStencilValue &vk_clear_color,
                     Span<VkImageSubresourceRange> ranges) = 0;

  /**
   * Clear attachments of the active framebuffer.
   */
  virtual void clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas) = 0;
  virtual void fill(VKBuffer &buffer, uint32_t data) = 0;

  virtual void draw(int v_first, int v_count, int i_first, int i_count) = 0;
  virtual void draw_indexed(int index_count,
                            int instance_count,
                            int first_index,
                            int vertex_offset,
                            int first_instance) = 0;

  virtual void draw_indirect(const VKStorageBuffer &buffer,
                             VkDeviceSize offset,
                             uint32_t draw_count,
                             uint32_t stride) = 0;
  virtual void draw_indexed_indirect(const VKStorageBuffer &buffer,
                                     VkDeviceSize offset,
                                     uint32_t draw_count,
                                     uint32_t stride) = 0;
};

}  // namespace blender::gpu
