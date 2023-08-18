/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"

namespace blender::gpu {

struct VKCommand {
  enum class Type {
    BindPipeline,
    BindDescriptorSet,
    BindVertexBuffer,
    BindIndexBuffer,

    BeginRenderPass,
    EndRenderPass,

    PushConstants,

    FillBuffer,

    CopyImageToBuffer,
    CopyBufferToImage,
    CopyImage,
    BlitImage,

    ClearColorImage,
    ClearAttachments,

    Draw,
    DrawIndexed,
    DrawIndirect,
    DrawIndexedIndirect,

    PipelineBarrier,
    PipelineImageMemoryBarrier,

    Dispatch,
    DispatchIndirect,
  };
  Type type;

  union {
    struct {
      VkPipelineBindPoint vk_pipeline_bind_point;
      VkPipeline vk_pipeline;
    } bind_pipeline;

    struct {
      VkDescriptorSet vk_descriptor_set;
      VkPipelineLayout vk_pipeline_layout;
      VkPipelineBindPoint vk_pipeline_bind_point;
    } bind_descriptor_set;

    struct {
      uint32_t binding;
      VkBuffer vk_buffer;
      VkDeviceSize offset;
    } bind_vertex_buffer;

    struct {
      VkBuffer vk_buffer;
      VkDeviceSize offset;
      VkIndexType vk_index_type;
    } bind_index_buffer;

    struct {
      VkRenderPass vk_render_pass;
      VkFramebuffer vk_framebuffer;
      VkRect2D render_area;
    } begin_render_pass;

    struct {
    } end_render_pass;

    struct {
      VkPipelineLayout vk_pipeline_layout;
      VkShaderStageFlags vk_shader_stages;
      uint32_t offset;
      uint32_t size;
      const void *data;
    } push_constants;

    struct {
      VkBuffer vk_buffer;
      VkDeviceSize size;
      uint32_t clear_data;
    } fill_buffer;

    struct {
      VkImage vk_source;
      VkImageLayout vk_source_layout;
      VkBuffer vk_destination;
      Vector<VkBufferImageCopy> *regions;
    } copy_image_to_buffer;

    struct {
      VkBuffer vk_source;
      VkImage vk_destination;
      VkImageLayout vk_destination_layout;
      Vector<VkBufferImageCopy> *regions;
    } copy_buffer_to_image;

    struct {
      VkImage vk_destination;
      VkImageLayout vk_destination_layout;
      VkImage vk_source;
      VkImageLayout vk_source_layout;
      Vector<VkImageCopy> *regions;
    } copy_image;

    struct {
      VkImage vk_destination;
      VkImageLayout vk_destination_layout;
      VkImage vk_source;
      VkImageLayout vk_source_layout;
      Vector<VkImageBlit> *regions;
    } blit_image;

    struct {
      VkImage vk_image;
      VkImageLayout vk_image_layout;
      VkClearColorValue vk_clear_color_value;
      Vector<VkImageSubresourceRange> *ranges;
    } clear_color_image;

    struct {
      Vector<VkClearAttachment> *attachments;
      Vector<VkClearRect> *areas;
    } clear_attachments;

    struct {
      uint32_t vertex_count;
      uint32_t instance_count;
      uint32_t first_vertex;
      uint32_t first_instance;
    } draw;

    struct {
      uint32_t index_count;
      uint32_t instance_count;
      uint32_t first_index;
      uint32_t vertex_offset;
      uint32_t first_instance;
    } draw_indexed;

    struct {
      VkBuffer vk_buffer;
      VkDeviceSize offset;
      uint32_t draw_count;
      uint32_t stride;
    } draw_indirect;

    struct {
      VkBuffer vk_buffer;
      VkDeviceSize offset;
      uint32_t draw_count;
      uint32_t stride;
    } draw_indexed_indirect;

    struct {
      VkPipelineStageFlags source_stages;
      VkPipelineStageFlags destination_stages;
    } pipeline_barrier;

    struct {
      Vector<VkImageMemoryBarrier> *image_memory_barriers;
    } pipeline_image_memory_barrier;

    struct {
      uint32_t groups_x_len;
      uint32_t groups_y_len;
      uint32_t groups_z_len;
    } dispatch;

    struct {
      VkBuffer vk_buffer;
    } dispatch_indirect;
  };

  VKCommand(Type type) : type(type) {}
  void free();
};

std::ostream &operator<<(std::ostream &stream, const VKCommand::Type &command_type);
std::ostream &operator<<(std::ostream &stream, const VKCommand &command);

}  // namespace blender::gpu
