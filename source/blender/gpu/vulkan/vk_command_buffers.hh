/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_command_buffer.hh"

namespace blender::gpu {

class VKCommandBuffers : public VKCommands, NonCopyable, NonMovable {
  enum class Type {
    DataTransfer = 0,
    Compute = 1,
    Graphics = 2,
    Max = 3,
  };

  bool initialized_ = false;
  /**
   * Timeout to use when waiting for fences in nanoseconds.
   *
   * Currently added as the fence will halt when there are no commands in the command buffer for
   * the second time. This should be solved and this timeout should be removed.
   */
  static constexpr uint64_t FenceTimeout = UINT64_MAX;

  /* Not owning handle */
  /** Handles to the command queue and device. Handle is owned by `GHOST_ContextVK`. */
  VkDevice vk_device_ = VK_NULL_HANDLE;
  VkQueue vk_queue_ = VK_NULL_HANDLE;

  /* Owning handles */
  /* Fence for CPU GPU synchronization when submitting the command buffers. */
  VkFence vk_fence_ = VK_NULL_HANDLE;

  /* TODO: General command buffer should not be used, but is added to help during the transition.*/
  VKCommandBuffer buffers_[(int)Type::Max];
  VKSubmissionID submission_id_;

 public:
  ~VKCommandBuffers();

  void init(const VKDevice &device);

  /**
   * Have these command buffers already been initialized?
   */
  bool is_initialized() const
  {
    return initialized_;
  }

  void bind(const VKPipeline &vk_pipeline, VkPipelineBindPoint bind_point) override;
  void bind(const VKDescriptorSet &descriptor_set,
            const VkPipelineLayout vk_pipeline_layout,
            VkPipelineBindPoint bind_point) override;
  void bind(const uint32_t binding,
            const VKVertexBuffer &vertex_buffer,
            const VkDeviceSize offset) override;
  /* Bind the given buffer as a vertex buffer. */
  void bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer) override;
  void bind(const uint32_t binding,
            const VkBuffer &vk_vertex_buffer,
            const VkDeviceSize offset) override;
  /* Bind the given buffer as an index buffer. */
  void bind(const VKBufferWithOffset &index_buffer, VkIndexType index_type) override;

  void begin_render_pass(VKFrameBuffer &framebuffer) override;
  void end_render_pass(const VKFrameBuffer &framebuffer) override;

  /**
   * Add a push constant command to the command buffer.
   *
   * Only valid when the storage type of push_constants is StorageType::PUSH_CONSTANTS.
   */
  void push_constants(const VKPushConstants &push_constants,
                      const VkPipelineLayout vk_pipeline_layout,
                      const VkShaderStageFlags vk_shader_stages) override;
  void dispatch(int groups_x_len, int groups_y_len, int groups_z_len) override;
  void dispatch(VKStorageBuffer &command_buffer) override;
  /** Copy the contents of a texture MIP level to the dst buffer. */
  void copy(VKBuffer &dst_buffer,
            VKTexture &src_texture,
            Span<VkBufferImageCopy> regions) override;
  void copy(VKTexture &dst_texture,
            VKBuffer &src_buffer,
            Span<VkBufferImageCopy> regions) override;
  void copy(VKTexture &dst_texture, VKTexture &src_texture, Span<VkImageCopy> regions) override;
  void copy(VKBuffer &dst_buffer, VkBuffer src_buffer, Span<VkBufferCopy> regions) override;
  void blit(VKTexture &dst_texture, VKTexture &src_texture, Span<VkImageBlit> regions) override;
  void blit(VKTexture &dst_texture,
            VkImageLayout dst_layout,
            VKTexture &src_texture,
            VkImageLayout src_layout,
            Span<VkImageBlit> regions) override;
  void pipeline_barrier(VkPipelineStageFlags source_stages,
                        VkPipelineStageFlags destination_stages) override;
  void pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers) override;

  /**
   * Clear color image resource.
   */
  void clear(VkImage vk_image,
             VkImageLayout vk_image_layout,
             const VkClearColorValue &vk_clear_color,
             Span<VkImageSubresourceRange> ranges) override;

  /**
   * Clear depth/stencil aspect of an image resource.
   */
  void clear(VkImage vk_image,
             VkImageLayout vk_image_layout,
             const VkClearDepthStencilValue &vk_clear_color,
             Span<VkImageSubresourceRange> ranges) override;

  /**
   * Clear attachments of the active framebuffer.
   */
  void clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas) override;
  void fill(VKBuffer &buffer, uint32_t data) override;

  void draw(int v_first, int v_count, int i_first, int i_count) override;
  void draw_indexed(int index_count,
                    int instance_count,
                    int first_index,
                    int vertex_offset,
                    int first_instance) override;

  void draw_indirect(const VKStorageBuffer &buffer,
                     VkDeviceSize offset,
                     uint32_t draw_count,
                     uint32_t stride) override;
  void draw_indexed_indirect(const VKStorageBuffer &buffer,
                             VkDeviceSize offset,
                             uint32_t draw_count,
                             uint32_t stride) override;

  void submit();

  const VKSubmissionID &submission_id_get() const
  {
    return submission_id_;
  }

 private:
  void init_fence();
  void init_command_buffers(const VKDevice &device);

  VKCommandBuffer &command_buffer_get(Type type)
  {
    return buffers_[(int)type];
  }

  /**
   * Ensure that no compute or draw commands are scheduled.
   *
   * To ensure correct operation all compute and draw commands should be flushed when adding a new
   * data transfer command.
   */
  void ensure_no_compute_or_draw_commands();

  /**
   * Ensure that no compute commands are scheduled.
   *
   * To ensure correct operation all compute commands should be flushed when adding a new draw
   * command.
   */
  void ensure_no_compute_commands();

  /**
   * Ensire that no draw_commands are scheduled.
   *
   * To ensure correct operation all draw commands should be flushed when adding a new compute
   * command.
   */
  void ensure_no_draw_commands();
};

}  // namespace blender::gpu
