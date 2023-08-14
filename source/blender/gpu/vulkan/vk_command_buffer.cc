/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_buffer.hh"
#include "vk_buffer.hh"
#include "vk_context.hh"
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
  if (vk_device_ != VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS;
    vkDestroyFence(vk_device_, vk_fence_, vk_allocation_callbacks);
    vk_fence_ = VK_NULL_HANDLE;
  }
}

void VKCommandBuffer::init(const VkDevice vk_device,
                           const VkQueue vk_queue,
                           VkCommandBuffer vk_command_buffer)
{
  vk_device_ = vk_device;
  vk_queue_ = vk_queue;
  vk_command_buffer_ = vk_command_buffer;
  submission_id_.reset();
  state.stage = Stage::Initial;

  /* When a the last GHOST context is destroyed the device is deallocate. A moment later the GPU
   * context is destroyed. The first step is to activate it. Activating would retrieve the device
   * from GHOST which in that case is a #VK_NULL_HANDLE. */
  if (vk_device == VK_NULL_HANDLE) {
    return;
  }

  if (vk_fence_ == VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    vkCreateFence(vk_device_, &fenceInfo, vk_allocation_callbacks, &vk_fence_);
  }
  else {
    vkResetFences(vk_device_, 1, &vk_fence_);
  }
}

void VKCommandBuffer::begin_recording()
{
  if (is_in_stage(Stage::Executed)) {
    vkResetCommandBuffer(vk_command_buffer_, 0);
    stage_transfer(Stage::Executed, Stage::Initial);
  }

  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  vkBeginCommandBuffer(vk_command_buffer_, &begin_info);
  stage_transfer(Stage::Initial, Stage::Recording);
}

void VKCommandBuffer::end_recording()
{
  vkEndCommandBuffer(vk_command_buffer_);
  stage_transfer(Stage::Recording, Stage::BetweenRecordingAndSubmitting);
}

void VKCommandBuffer::bind(const VKPipeline &pipeline, VkPipelineBindPoint vk_bind_point)
{
  VKCommand &command = add_command(VKCommand::Type::BindPipeline);
  command.bind_pipeline.vk_pipeline = pipeline.vk_handle();
  command.bind_pipeline.vk_pipeline_bind_point = vk_bind_point;
}

void VKCommandBuffer::bind(const VKDescriptorSet &descriptor_set,
                           const VkPipelineLayout vk_pipeline_layout,
                           VkPipelineBindPoint bind_point)
{
  VKCommand &command = add_command(VKCommand::Type::BindDescriptorSet);
  command.bind_descriptor_set.vk_descriptor_set = descriptor_set.vk_handle();
  command.bind_descriptor_set.vk_pipeline_bind_point = bind_point;
  command.bind_descriptor_set.vk_pipeline_layout = vk_pipeline_layout;
}

void VKCommandBuffer::bind(const uint32_t binding,
                           const VKVertexBuffer &vertex_buffer,
                           const VkDeviceSize offset)
{
  bind(binding, vertex_buffer.vk_handle(), offset);
}

void VKCommandBuffer::bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer)
{
  bind(binding, vertex_buffer.buffer.vk_handle(), vertex_buffer.offset);
}

void VKCommandBuffer::bind(const uint32_t binding,
                           const VkBuffer &vk_vertex_buffer,
                           const VkDeviceSize offset)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();

  VKCommand &command = add_command(VKCommand::Type::BindVertexBuffer);
  command.bind_vertex_buffer.binding = binding;
  command.bind_vertex_buffer.vk_buffer = vk_vertex_buffer;
  command.bind_vertex_buffer.offset = offset;
}

void VKCommandBuffer::bind(const VKBufferWithOffset &index_buffer, VkIndexType index_type)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();

  VKCommand &command = add_command(VKCommand::Type::BindIndexBuffer);
  command.bind_index_buffer.vk_buffer = index_buffer.buffer.vk_handle();
  command.bind_index_buffer.offset = index_buffer.offset;
  command.bind_index_buffer.vk_index_type = index_type;
}

void VKCommandBuffer::begin_render_pass(const VKFrameBuffer &framebuffer)
{
  validate_framebuffer_not_exists();
  state.framebuffer_ = &framebuffer;
}

void VKCommandBuffer::end_render_pass(const VKFrameBuffer &framebuffer)
{
  UNUSED_VARS_NDEBUG(framebuffer);
  validate_framebuffer_exists();
  BLI_assert(state.framebuffer_ == &framebuffer);
  ensure_no_active_framebuffer();
  state.framebuffer_ = nullptr;
}

void VKCommandBuffer::push_constants(const VKPushConstants &push_constants,
                                     const VkPipelineLayout vk_pipeline_layout,
                                     const VkShaderStageFlags vk_shader_stages)
{
  BLI_assert(push_constants.layout_get().storage_type_get() ==
             VKPushConstants::StorageType::PUSH_CONSTANTS);

  VKCommand &command = add_command(VKCommand::Type::PushConstants);
  command.push_constants.vk_pipeline_layout = vk_pipeline_layout;
  command.push_constants.vk_shader_stages = vk_shader_stages;
  command.push_constants.offset = push_constants.offset();
  command.push_constants.size = push_constants.layout_get().size_in_bytes();
  command.push_constants.data = push_constants.data();
}

void VKCommandBuffer::fill(VKBuffer &buffer, uint32_t clear_data)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::FillBuffer);
  command.fill_buffer.vk_buffer = buffer.vk_handle();
  command.fill_buffer.size = buffer.size_in_bytes();
  command.fill_buffer.clear_data = clear_data;
}

void VKCommandBuffer::copy(VKBuffer &dst_buffer,
                           VKTexture &src_texture,
                           Span<VkBufferImageCopy> regions)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::CopyImageToBuffer);
  command.copy_image_to_buffer.vk_source = src_texture.vk_image_handle();
  command.copy_image_to_buffer.vk_source_layout = src_texture.current_layout_get();
  command.copy_image_to_buffer.vk_destination = dst_buffer.vk_handle();
  command.copy_image_to_buffer.regions = MEM_new<Vector<VkBufferImageCopy>>(__func__, regions);
}

void VKCommandBuffer::copy(VKTexture &dst_texture,
                           VKBuffer &src_buffer,
                           Span<VkBufferImageCopy> regions)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::CopyBufferToImage);
  command.copy_buffer_to_image.vk_source = src_buffer.vk_handle();
  command.copy_buffer_to_image.vk_destination = dst_texture.vk_image_handle();
  command.copy_buffer_to_image.vk_destination_layout = dst_texture.current_layout_get();
  command.copy_buffer_to_image.regions = MEM_new<Vector<VkBufferImageCopy>>(__func__, regions);
}

void VKCommandBuffer::copy(VKTexture &dst_texture,
                           VKTexture &src_texture,
                           Span<VkImageCopy> regions)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::CopyImage);
  command.copy_image.vk_source = src_texture.vk_image_handle();
  command.copy_image.vk_source_layout = src_texture.current_layout_get();
  command.copy_image.vk_destination = dst_texture.vk_image_handle();
  command.copy_image.vk_destination_layout = dst_texture.current_layout_get();
  command.copy_image.regions = MEM_new<Vector<VkImageCopy>>(__func__, regions);
}

void VKCommandBuffer::blit(VKTexture &dst_texture,
                           VKTexture &src_texture,
                           Span<VkImageBlit> regions)
{
  blit(dst_texture,
       dst_texture.current_layout_get(),
       src_texture,
       src_texture.current_layout_get(),
       regions);
}

void VKCommandBuffer::blit(VKTexture &dst_texture,
                           VkImageLayout dst_layout,
                           VKTexture &src_texture,
                           VkImageLayout src_layout,
                           Span<VkImageBlit> regions)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::BlitImage);
  command.blit_image.vk_source = src_texture.vk_image_handle();
  command.blit_image.vk_source_layout = src_layout;
  command.blit_image.vk_destination = dst_texture.vk_image_handle();
  command.blit_image.vk_destination_layout = dst_layout;
  command.blit_image.regions = MEM_new<Vector<VkImageBlit>>(__func__, regions);
}

void VKCommandBuffer::clear(VkImage vk_image,
                            VkImageLayout vk_image_layout,
                            const VkClearColorValue &vk_clear_color,
                            Span<VkImageSubresourceRange> ranges)
{
  ensure_no_active_framebuffer();

  VKCommand &command = add_command(VKCommand::Type::ClearColorImage);
  command.clear_color_image.vk_image = vk_image;
  command.clear_color_image.vk_image_layout = vk_image_layout;
  command.clear_color_image.vk_clear_color_value = vk_clear_color;
  command.clear_color_image.ranges = MEM_new<Vector<VkImageSubresourceRange>>(__func__, ranges);
}

void VKCommandBuffer::clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::ClearAttachments);
  command.clear_attachments.attachments = MEM_new<Vector<VkClearAttachment>>(__func__,
                                                                             attachments);
  command.clear_attachments.areas = MEM_new<Vector<VkClearRect>>(__func__, areas);
}

void VKCommandBuffer::draw(int v_first, int v_count, int i_first, int i_count)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();

  VKCommand &command = add_command(VKCommand::Type::Draw);
  command.draw.vertex_count = v_count;
  command.draw.instance_count = i_count;
  command.draw.first_vertex = v_first;
  command.draw.first_instance = i_first;

  state.draw_counts++;
}

void VKCommandBuffer::draw(
    int index_count, int instance_count, int first_index, int vertex_offset, int first_instance)
{
  validate_framebuffer_exists();
  ensure_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::DrawIndexed);
  command.draw_indexed.index_count = index_count;
  command.draw_indexed.instance_count = instance_count;
  command.draw_indexed.first_index = first_index;
  command.draw_indexed.vertex_offset = vertex_offset;
  command.draw_indexed.first_instance = first_instance;

  state.draw_counts++;
}

void VKCommandBuffer::pipeline_barrier(VkPipelineStageFlags source_stages,
                                       VkPipelineStageFlags destination_stages)
{
  if (state.framebuffer_) {
    ensure_active_framebuffer();
  }

  VKCommand &command = add_command(VKCommand::Type::PipelineBarrier);
  command.pipeline_barrier.source_stages = source_stages;
  command.pipeline_barrier.destination_stages = destination_stages;
}

void VKCommandBuffer::pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::PipelineImageMemoryBarrier);
  command.pipeline_image_memory_barrier.image_memory_barriers =
      MEM_new<Vector<VkImageMemoryBarrier>>(__func__, image_memory_barriers);
}

void VKCommandBuffer::dispatch(int groups_x_len, int groups_y_len, int groups_z_len)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::Dispatch);
  command.dispatch.groups_x_len = groups_x_len;
  command.dispatch.groups_y_len = groups_y_len;
  command.dispatch.groups_z_len = groups_z_len;
}

void VKCommandBuffer::dispatch(VKStorageBuffer &command_buffer)
{
  ensure_no_active_framebuffer();
  VKCommand &command = add_command(VKCommand::Type::DispatchIndirect);
  command.dispatch_indirect.vk_buffer = command_buffer.vk_handle();
}

void VKCommandBuffer::submit()
{
  ensure_no_active_framebuffer();
  // debug_print();
  encode_recorded_commands();
  submit_encoded_commands();
}

void VKCommandBuffer::encode_recorded_commands()
{
  begin_recording();
  for (VKCommand &command : commands_) {
    switch (command.type) {
      case VKCommand::Type::BindPipeline: {
        vkCmdBindPipeline(vk_command_buffer_,
                          command.bind_pipeline.vk_pipeline_bind_point,
                          command.bind_pipeline.vk_pipeline);
        break;
      }

      case VKCommand::Type::BindDescriptorSet: {
        vkCmdBindDescriptorSets(vk_command_buffer_,
                                command.bind_descriptor_set.vk_pipeline_bind_point,
                                command.bind_descriptor_set.vk_pipeline_layout,
                                0,
                                1,
                                &command.bind_descriptor_set.vk_descriptor_set,
                                0,
                                0);
        break;
      }

      case VKCommand::Type::BindVertexBuffer: {
        vkCmdBindVertexBuffers(vk_command_buffer_,
                               command.bind_vertex_buffer.binding,
                               1,
                               &command.bind_vertex_buffer.vk_buffer,
                               &command.bind_vertex_buffer.offset);
        break;
      }

      case VKCommand::Type::BindIndexBuffer: {
        vkCmdBindIndexBuffer(vk_command_buffer_,
                             command.bind_index_buffer.vk_buffer,
                             command.bind_index_buffer.offset,
                             command.bind_index_buffer.vk_index_type);
        break;
      }

      case VKCommand::Type::BeginRenderPass: {
        VkRenderPassBeginInfo render_pass_begin_info = {};
        render_pass_begin_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        render_pass_begin_info.renderPass = command.begin_render_pass.vk_render_pass;
        render_pass_begin_info.framebuffer = command.begin_render_pass.vk_framebuffer;
        render_pass_begin_info.renderArea = command.begin_render_pass.render_area;

        /* We don't use clear ops, but vulkan wants to have at least one. */
        VkClearValue clear_value = {};
        render_pass_begin_info.clearValueCount = 1;
        render_pass_begin_info.pClearValues = &clear_value;

        vkCmdBeginRenderPass(
            vk_command_buffer_, &render_pass_begin_info, VK_SUBPASS_CONTENTS_INLINE);

        break;
      }

      case VKCommand::Type::EndRenderPass: {
        vkCmdEndRenderPass(vk_command_buffer_);
        break;
      }

      case VKCommand::Type::PushConstants: {
        vkCmdPushConstants(vk_command_buffer_,
                           command.push_constants.vk_pipeline_layout,
                           command.push_constants.vk_shader_stages,
                           command.push_constants.offset,
                           command.push_constants.size,
                           command.push_constants.data);
        break;
      }

      case VKCommand::Type::FillBuffer: {
        vkCmdFillBuffer(vk_command_buffer_,
                        command.fill_buffer.vk_buffer,
                        0,
                        command.fill_buffer.size,
                        command.fill_buffer.clear_data);
        break;
      }

      case VKCommand::Type::CopyImageToBuffer: {
        vkCmdCopyImageToBuffer(vk_command_buffer_,
                               command.copy_image_to_buffer.vk_source,
                               command.copy_image_to_buffer.vk_source_layout,
                               command.copy_image_to_buffer.vk_destination,
                               command.copy_image_to_buffer.regions->size(),
                               command.copy_image_to_buffer.regions->data());
        break;
      }

      case VKCommand::Type::CopyBufferToImage: {
        vkCmdCopyBufferToImage(vk_command_buffer_,
                               command.copy_buffer_to_image.vk_source,
                               command.copy_buffer_to_image.vk_destination,
                               command.copy_buffer_to_image.vk_destination_layout,
                               command.copy_image_to_buffer.regions->size(),
                               command.copy_image_to_buffer.regions->data());
        break;
      }

      case VKCommand::Type::BlitImage: {
        vkCmdBlitImage(vk_command_buffer_,
                       command.blit_image.vk_source,
                       command.blit_image.vk_source_layout,
                       command.blit_image.vk_destination,
                       command.blit_image.vk_destination_layout,
                       command.blit_image.regions->size(),
                       command.blit_image.regions->data(),
                       VK_FILTER_LINEAR);
        break;
      }

      case VKCommand::Type::CopyImage: {
        vkCmdCopyImage(vk_command_buffer_,
                       command.copy_image.vk_source,
                       command.copy_image.vk_source_layout,
                       command.copy_image.vk_destination,
                       command.copy_image.vk_destination_layout,
                       command.copy_image.regions->size(),
                       command.copy_image.regions->data());
        break;
      }

      case VKCommand::Type::ClearColorImage: {
        vkCmdClearColorImage(vk_command_buffer_,
                             command.clear_color_image.vk_image,
                             command.clear_color_image.vk_image_layout,
                             &command.clear_color_image.vk_clear_color_value,
                             command.clear_color_image.ranges->size(),
                             command.clear_color_image.ranges->data());
        break;
      }

      case VKCommand::Type::ClearAttachments: {
        vkCmdClearAttachments(vk_command_buffer_,
                              command.clear_attachments.attachments->size(),
                              command.clear_attachments.attachments->data(),
                              command.clear_attachments.areas->size(),
                              command.clear_attachments.areas->data());
        break;
      }

      case VKCommand::Type::Draw: {
        vkCmdDraw(vk_command_buffer_,
                  command.draw.vertex_count,
                  command.draw.instance_count,
                  command.draw.first_vertex,
                  command.draw.first_instance);
        break;
      }

      case VKCommand::Type::DrawIndexed: {
        vkCmdDrawIndexed(vk_command_buffer_,
                         command.draw_indexed.index_count,
                         command.draw_indexed.instance_count,
                         command.draw_indexed.first_index,
                         command.draw_indexed.vertex_offset,
                         command.draw_indexed.first_instance);
        break;
      }

      case VKCommand::Type::PipelineBarrier: {
        vkCmdPipelineBarrier(vk_command_buffer_,
                             command.pipeline_barrier.source_stages,
                             command.pipeline_barrier.destination_stages,
                             0,
                             0,
                             nullptr,
                             0,
                             nullptr,
                             0,
                             nullptr);
        break;
      }

      case VKCommand::Type::PipelineImageMemoryBarrier: {
        vkCmdPipelineBarrier(vk_command_buffer_,
                             VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_DEPENDENCY_BY_REGION_BIT,
                             0,
                             nullptr,
                             0,
                             nullptr,
                             command.pipeline_image_memory_barrier.image_memory_barriers->size(),
                             command.pipeline_image_memory_barrier.image_memory_barriers->data());
        break;
      }

      case VKCommand::Type::Dispatch: {
        vkCmdDispatch(vk_command_buffer_,
                      command.dispatch.groups_x_len,
                      command.dispatch.groups_y_len,
                      command.dispatch.groups_z_len);
        break;
      }

      case VKCommand::Type::DispatchIndirect: {
        vkCmdDispatchIndirect(vk_command_buffer_, command.dispatch_indirect.vk_buffer, 0);
        break;
      }
    }
    command.free();
  }

  end_recording();
  commands_.clear();
}

void VKCommandBuffer::submit_encoded_commands()
{
  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &vk_command_buffer_;

  vkQueueSubmit(vk_queue_, 1, &submit_info, vk_fence_);
  submission_id_.next();
  stage_transfer(Stage::BetweenRecordingAndSubmitting, Stage::Submitted);

  /* Wait for execution to finish. */
  vkWaitForFences(vk_device_, 1, &vk_fence_, VK_TRUE, FenceTimeout);
  vkResetFences(vk_device_, 1, &vk_fence_);
  stage_transfer(Stage::Submitted, Stage::Executed);
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
    add_command(VKCommand::Type::EndRenderPass);

    state.framebuffer_active_ = false;
    state.switches_++;
  }
}

void VKCommandBuffer::ensure_active_framebuffer()
{
  BLI_assert(state.framebuffer_);
  state.checks_++;
  if (!state.framebuffer_active_) {
    VKCommand &command = add_command(VKCommand::Type::BeginRenderPass);
    command.begin_render_pass.vk_render_pass = state.framebuffer_->vk_render_pass_get();
    command.begin_render_pass.vk_framebuffer = state.framebuffer_->vk_framebuffer_get();
    command.begin_render_pass.render_area = state.framebuffer_->vk_render_areas_get()[0];

    state.framebuffer_active_ = true;
    state.switches_++;
  }
}

/** \} */

VKCommand &VKCommandBuffer::add_command(VKCommand::Type type)
{
  VKCommand command = {type};
  memset(static_cast<void *>(&command), 0, sizeof(VKCommand));
  command.type = type;
  commands_.append(command);
  return commands_.last();
}

void VKCommandBuffer::debug_print() const
{
  std::ostream &os = std::cout;

  os << "VKCommandBuffer::debug_print(draw_count:" << state.draw_counts << ")\n";
  for (const VKCommand &command : commands_) {
    os << "  - " << command << "\n";
  }
}

}  // namespace blender::gpu
