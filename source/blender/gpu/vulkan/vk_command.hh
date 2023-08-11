/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

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
  };

  VKCommand(Type type) : type(type) {}
  virtual ~VKCommand();
};
}  // namespace blender::gpu
