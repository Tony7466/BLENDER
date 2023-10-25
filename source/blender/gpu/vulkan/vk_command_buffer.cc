/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_buffer.hh"
#include "vk_backend.hh"
#include "vk_buffer.hh"
#include "vk_context.hh"
#include "vk_device.hh"
#include "vk_framebuffer.hh"
#include "vk_index_buffer.hh"
#include "vk_memory.hh"
#include "vk_pipeline.hh"
#include "vk_storage_buffer.hh"
#include "vk_texture.hh"
#include "vk_vertex_buffer.hh"

#include "BLI_assert.h"

namespace blender::gpu {

VKCommandBuffer::~VKCommandBuffer()
{
  if (vk_command_buffer_ != VK_NULL_HANDLE) {
    VKDevice &device = VKBackend::get().device_get();
    vkFreeCommandBuffers(
        device.device_get(), device.vk_command_pool_get(), 1, &vk_command_buffer_);
    vk_command_buffer_ = VK_NULL_HANDLE;
  }
}

bool VKCommandBuffer::is_initialized() const
{
  return vk_command_buffer_ != VK_NULL_HANDLE;
}

void VKCommandBuffer::init(const VKDevice &device, VkCommandBuffer vk_command_buffer)
{
  if (is_initialized()) {
    return;
  }

  vk_device_ = device.device_get();
  vk_command_buffer_ = vk_command_buffer;

  state.stage = Stage::Initial;
}

void VKCommandBuffer::begin_recording()
{
  ensure_no_active_framebuffer();
  if (is_in_stage(Stage::Submitted)) {
    stage_transfer(Stage::Submitted, Stage::Executed);
  }
  if (is_in_stage(Stage::Executed)) {
    vkResetCommandBuffer(vk_command_buffer_, 0);
    stage_transfer(Stage::Executed, Stage::Initial);
  }

  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  vkBeginCommandBuffer(vk_command_buffer_, &begin_info);
  stage_transfer(Stage::Initial, Stage::Recording);
  state.recorded_command_counts = 0;
}

void VKCommandBuffer::end_recording()
{
  ensure_no_active_framebuffer();
  vkEndCommandBuffer(vk_command_buffer_);
  stage_transfer(Stage::Recording, Stage::BetweenRecordingAndSubmitting);
}

void VKCommandBuffer::bind(const VKPipeline &pipeline, VkPipelineBindPoint bind_point) {}

void VKCommandBuffer::bind(const VKDescriptorSet &descriptor_set,
                           const VkPipelineLayout vk_pipeline_layout,
                           VkPipelineBindPoint bind_point)
{
}

void VKCommandBuffer::bind(const uint32_t binding,
                           const VKVertexBuffer &vertex_buffer,
                           const VkDeviceSize offset)
{
}

void VKCommandBuffer::bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer) {}

void VKCommandBuffer::bind(const uint32_t binding,
                           const VkBuffer &vk_vertex_buffer,
                           const VkDeviceSize offset)
{
}

void VKCommandBuffer::bind(const VKBufferWithOffset &index_buffer, VkIndexType index_type) {}

void VKCommandBuffer::begin_render_pass(VKFrameBuffer &framebuffer)
{
  validate_framebuffer_not_exists();
  state.framebuffer_ = &framebuffer;
}

void VKCommandBuffer::end_render_pass(const VKFrameBuffer &framebuffer)
{
  UNUSED_VARS_NDEBUG(framebuffer);
  BLI_assert(state.framebuffer_ == nullptr || state.framebuffer_ == &framebuffer);
  ensure_no_active_framebuffer();
  state.framebuffer_ = nullptr;
}

void VKCommandBuffer::push_constants(const VKPushConstants &push_constants,
                                     const VkPipelineLayout vk_pipeline_layout,
                                     const VkShaderStageFlags vk_shader_stages)
{
}

void VKCommandBuffer::fill(VKBuffer &buffer, uint32_t clear_data)
{
  ensure_no_active_framebuffer();
  vkCmdFillBuffer(vk_command_buffer_, buffer.vk_handle(), 0, buffer.size_in_bytes(), clear_data);
  state.recorded_command_counts++;
}

void VKCommandBuffer::copy(VKBuffer &dst_buffer,
                           VKTexture &src_texture,
                           Span<VkBufferImageCopy> regions)
{
}

void VKCommandBuffer::copy(VKTexture &dst_texture,
                           VKBuffer &src_buffer,
                           Span<VkBufferImageCopy> regions)
{
}

void VKCommandBuffer::copy(VKTexture &dst_texture,
                           VKTexture &src_texture,
                           Span<VkImageCopy> regions)
{
}

void VKCommandBuffer::copy(VKBuffer &dst_buffer, VkBuffer src_buffer, Span<VkBufferCopy> regions)
{
}

void VKCommandBuffer::blit(VKTexture &dst_texture,
                           VKTexture &src_texture,
                           Span<VkImageBlit> regions)
{
}

void VKCommandBuffer::blit(VKTexture &dst_texture,
                           VkImageLayout dst_layout,
                           VKTexture &src_texture,
                           VkImageLayout src_layout,
                           Span<VkImageBlit> regions)
{
}

void VKCommandBuffer::clear(VkImage vk_image,
                            VkImageLayout vk_image_layout,
                            const VkClearColorValue &vk_clear_color,
                            Span<VkImageSubresourceRange> ranges)
{
  ensure_no_active_framebuffer();
  vkCmdClearColorImage(vk_command_buffer_,
                       vk_image,
                       vk_image_layout,
                       &vk_clear_color,
                       ranges.size(),
                       ranges.data());
  state.recorded_command_counts++;
}

void VKCommandBuffer::clear(VkImage vk_image,
                            VkImageLayout vk_image_layout,
                            const VkClearDepthStencilValue &vk_clear_value,
                            Span<VkImageSubresourceRange> ranges)
{
  ensure_no_active_framebuffer();
  vkCmdClearDepthStencilImage(vk_command_buffer_,
                              vk_image,
                              vk_image_layout,
                              &vk_clear_value,
                              ranges.size(),
                              ranges.data());
  state.recorded_command_counts++;
}

void VKCommandBuffer::clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  vkCmdClearAttachments(
      vk_command_buffer_, attachments.size(), attachments.data(), areas.size(), areas.data());
  state.recorded_command_counts++;
}

void VKCommandBuffer::draw(int v_first, int v_count, int i_first, int i_count)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  vkCmdDraw(vk_command_buffer_, v_count, i_count, v_first, i_first);
  state.draw_counts++;
  state.recorded_command_counts++;
}

void VKCommandBuffer::draw_indexed(
    int index_count, int instance_count, int first_index, int vertex_offset, int first_instance)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  vkCmdDrawIndexed(
      vk_command_buffer_, index_count, instance_count, first_index, vertex_offset, first_instance);
  state.draw_counts++;
  state.recorded_command_counts++;
}

void VKCommandBuffer::draw_indirect(const VKStorageBuffer &buffer,
                                    VkDeviceSize offset,
                                    uint32_t draw_count,
                                    uint32_t stride)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  vkCmdDrawIndirect(vk_command_buffer_, buffer.vk_handle(), offset, draw_count, stride);
  state.draw_counts++;
  state.recorded_command_counts++;
}

void VKCommandBuffer::draw_indexed_indirect(const VKStorageBuffer &buffer,

                                            VkDeviceSize offset,
                                            uint32_t draw_count,
                                            uint32_t stride)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  vkCmdDrawIndexedIndirect(vk_command_buffer_, buffer.vk_handle(), offset, draw_count, stride);
  state.draw_counts++;
  state.recorded_command_counts++;
}

void VKCommandBuffer::pipeline_barrier(VkPipelineStageFlags source_stages,
                                       VkPipelineStageFlags destination_stages)
{
}

void VKCommandBuffer::pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers) {}

void VKCommandBuffer::dispatch(int groups_x_len, int groups_y_len, int groups_z_len) {}

void VKCommandBuffer::dispatch(VKStorageBuffer &command_buffer) {}

void VKCommandBuffer::commands_submitted()
{
  stage_transfer(Stage::BetweenRecordingAndSubmitting, Stage::Submitted);
}

/* -------------------------------------------------------------------- */
/** \name FrameBuffer/RenderPass state tracking
 * \{ */

void VKCommandBuffer::validate_framebuffer_not_exists()
{
  BLI_assert_msg(state.framebuffer_ == nullptr && state.framebuffer_active_ == false,
                 "State error: expected no framebuffer being tracked.");
}

void VKCommandBuffer::validate_framebuffer_exists()
{
  BLI_assert_msg(state.framebuffer_, "State error: expected framebuffer being tracked.");
}

void VKCommandBuffer::ensure_no_active_framebuffer()
{
  state.checks_++;
  if (state.framebuffer_ && state.framebuffer_active_) {
    vkCmdEndRenderPass(vk_command_buffer_);
    state.framebuffer_active_ = false;
    state.switches_++;
    state.recorded_command_counts++;
  }
}

void VKCommandBuffer::ensure_active_framebuffer()
{
  BLI_assert(state.framebuffer_);
  state.checks_++;
  if (!state.framebuffer_active_) {
    VkRenderPassBeginInfo render_pass_begin_info = {};
    render_pass_begin_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    state.framebuffer_->vk_render_pass_ensure();
    render_pass_begin_info.renderPass = state.framebuffer_->vk_render_pass_get();
    render_pass_begin_info.framebuffer = state.framebuffer_->vk_framebuffer_get();
    render_pass_begin_info.renderArea = state.framebuffer_->vk_render_areas_get()[0];
    /* We don't use clear ops, but vulkan wants to have at least one. */
    VkClearValue clear_value = {};
    render_pass_begin_info.clearValueCount = 1;
    render_pass_begin_info.pClearValues = &clear_value;

    vkCmdBeginRenderPass(vk_command_buffer_, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);
    state.framebuffer_active_ = true;
    state.switches_++;
    state.recorded_command_counts++;
  }
}

/** \} */

}  // namespace blender::gpu
