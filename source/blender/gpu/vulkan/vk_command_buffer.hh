/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_buffer_types.hh"
#include "vk_resource_tracker.hh"

#include "BLI_utility_mixins.hh"

namespace blender::gpu {
class VKBuffer;
class VKDescriptorSet;
class VKFrameBuffer;
class VKIndexBuffer;
class VKPipeline;
class VKPushConstants;
class VKTexture;
class VKVertexBuffer;

/** Command buffer to keep track of the life-time of a command buffer. */
class VKCommandBuffer : NonCopyable, NonMovable {
  /** None owning handle to the command buffer and device. Handle is owned by `GHOST_ContextVK`. */
  VkDevice vk_device_ = VK_NULL_HANDLE;
  VkCommandBuffer vk_command_buffer_ = VK_NULL_HANDLE;
  VkQueue vk_queue_ = VK_NULL_HANDLE;

  /** Owning handles */
  VkFence vk_fence_ = VK_NULL_HANDLE;
  VKSubmissionID submission_id_;

 private:
  /*
   * Some vulkan command require an active frame buffer. Others require no active frame-buffer. As
   * our current API does not provide a solution for this we need to keep track of the actual state
   * and do the changes when recording the next command.
   *
   * This is a temporary solution to get things rolling.
   * TODO: In a future solution we should decide the scope of a command buffer.
   *
   * - command buffer per draw command.
   * - minimize command buffers and track render passes.
   * - add custom encoder to also track resource usages.
   *
   * Currently I expect the custom encoder has to be done eventually. But want to keep post-poning
   * the custom encoder for now to collect more use cases it should solve. (first pixel drawn on
   * screen).
   *
   * Some command can also be encoded in another way when encoded as a first command. For example
   * clearing a framebuffer textures isn't allowed inside a render pass, but clearing the
   * framebuffer textures via ops is allowed. When clearing a framebuffer texture directly after
   * beginning a render pass could be re-encoded to do this in the same command.
   *
   * So for now we track the state and temporary switch to another state if the command requires
   * it.
   */
  struct {
    /* Reference to the last_framebuffer where begin_render_pass was called for. */
    const VKFrameBuffer *framebuffer_ = nullptr;
    /* Is last_framebuffer_ currently bound. Each call should ensure the correct state. */
    bool framebuffer_active_ = false;
    /* Amount of times a check has been requested. */
    uint64_t checks_ = 0;
    /* Amount of times a check required to change the render pass. */
    uint64_t switches_ = 0;
  } state;

 public:
  virtual ~VKCommandBuffer();
  void init(const VkDevice vk_device, const VkQueue vk_queue, VkCommandBuffer vk_command_buffer);
  void begin_recording();
  void end_recording();
  void bind(const VKPipeline &vk_pipeline, VkPipelineBindPoint bind_point);
  void bind(const VKDescriptorSet &descriptor_set,
            const VkPipelineLayout vk_pipeline_layout,
            VkPipelineBindPoint bind_point);
  void bind(const uint32_t binding,
            const VKVertexBuffer &vertex_buffer,
            const VkDeviceSize offset);
  /* Bind the given buffer as a vertex buffer. */
  void bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer);
  void bind(const uint32_t binding, const VkBuffer &vk_vertex_buffer, const VkDeviceSize offset);

  void bind(const VKIndexBuffer &index_buffer, VkIndexType index_type);
  void begin_render_pass(const VKFrameBuffer &framebuffer);
  void end_render_pass(const VKFrameBuffer &framebuffer);

  /**
   * Add a push constant command to the command buffer.
   *
   * Only valid when the storage type of push_constants is StorageType::PUSH_CONSTANTS.
   */
  void push_constants(const VKPushConstants &push_constants,
                      const VkPipelineLayout vk_pipeline_layout,
                      const VkShaderStageFlags vk_shader_stages);
  void dispatch(int groups_x_len, int groups_y_len, int groups_z_len);
  /** Copy the contents of a texture MIP level to the dst buffer. */
  void copy(VKBuffer &dst_buffer, VKTexture &src_texture, Span<VkBufferImageCopy> regions);
  void copy(VKTexture &dst_texture, VKBuffer &src_buffer, Span<VkBufferImageCopy> regions);
  void pipeline_barrier(VkPipelineStageFlags source_stages,
                        VkPipelineStageFlags destination_stages);
  void pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers);
  /**
   * Clear color image resource.
   */
  void clear(VkImage vk_image,
             VkImageLayout vk_image_layout,
             const VkClearColorValue &vk_clear_color,
             Span<VkImageSubresourceRange> ranges);

  /**
   * Clear attachments of the active framebuffer.
   */
  void clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas);
  void fill(VKBuffer &buffer, uint32_t data);

  void draw(int v_first, int v_count, int i_first, int i_count);

  /**
   * Stop recording commands, encode + send the recordings to Vulkan, wait for the until the
   * commands have been executed and start the command buffer to accept recordings again.
   */
  void submit();

  const VKSubmissionID &submission_id_get() const
  {
    return submission_id_;
  }

 private:
  void encode_recorded_commands();
  void submit_encoded_commands();

  /**
   * Validate that there isn't a framebuffer being tracked (bound or not bound).
   *
   * Raises an assert in debug when a framebuffer is being tracked.
   */
  void validate_framebuffer_not_exists();

  /**
   * Validate that there is a framebuffer being tracked (bound or not bound).
   *
   * Raises an assert in debug when no framebuffer is being tracked.
   */
  void validate_framebuffer_exists();

  /**
   * Ensure that there is no framebuffer being tracked or the tracked framebuffer isn't bound.
   */
  void ensure_no_active_framebuffer();

  /**
   * Ensure that the tracked framebuffer is bound.
   */
  void ensure_active_framebuffer();
};

}  // namespace blender::gpu
