/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_command_buffers.hh"
#include "vk_device.hh"
#include "vk_memory.hh"

#include "BLI_assert.h"

namespace blender::gpu {

VKCommandBuffers::~VKCommandBuffers()
{
  if (vk_fence_ != VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS;
    vkDestroyFence(vk_device_, vk_fence_, vk_allocation_callbacks);
    vk_fence_ = VK_NULL_HANDLE;
  }
}

void VKCommandBuffers::init(const VKDevice &device)
{
  if (initialized_) {
    return;
  }
  initialized_ = true;

  vk_device_ = device.device_get();
  vk_queue_ = device.queue_get();

  /* When a the last GHOST context is destroyed the device is deallocate. A moment later the GPU
   * context is destroyed. The first step is to activate it. Activating would retrieve the device
   * from GHOST which in that case is a #VK_NULL_HANDLE. */
  if (vk_device_ == VK_NULL_HANDLE) {
    return;
  }

  init_command_buffers(device);
  init_fence();
  submission_id_.reset();
}

static void init_command_buffer(const VKDevice &device,
                                VKCommandBuffer &command_buffer,
                                VkCommandBuffer vk_command_buffer,
                                const char *name)
{
  command_buffer.init(device, vk_command_buffer);
  command_buffer.begin_recording();
  debug::object_label(vk_command_buffer, name);
}

void VKCommandBuffers::init_command_buffers(const VKDevice &device)
{
  VkCommandBuffer vk_command_buffers[4] = {VK_NULL_HANDLE};
  VkCommandBufferAllocateInfo alloc_info = {};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = device.vk_command_pool_get();
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = (uint32_t)Type::Max;
  vkAllocateCommandBuffers(vk_device_, &alloc_info, vk_command_buffers);

  init_command_buffer(device,
                      command_buffer_get(Type::DataTransfer),
                      vk_command_buffers[(int)Type::DataTransfer],
                      "Data Transfer Command Buffer");
  init_command_buffer(device,
                      command_buffer_get(Type::Compute),
                      vk_command_buffers[(int)Type::Compute],
                      "Compute Command Buffer");
  init_command_buffer(device,
                      command_buffer_get(Type::Graphics),
                      vk_command_buffers[(int)Type::Graphics],
                      "Graphics Command Buffer");
}

void VKCommandBuffers::init_fence()
{
  if (vk_fence_ == VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    vkCreateFence(vk_device_, &fenceInfo, vk_allocation_callbacks, &vk_fence_);
  }
}

static void submit_command_buffer(VkDevice vk_device,
                                  VkQueue vk_queue,
                                  VKCommandBuffer &command_buffer,
                                  VkFence vk_fence,
                                  uint64_t timeout)
{
  VkCommandBuffer handles[1];
  command_buffer.end_recording();

  handles[0] = command_buffer.vk_command_buffer();
  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = handles;

  vkQueueSubmit(vk_queue, 1, &submit_info, vk_fence);
  command_buffer.commands_submitted();

  vkWaitForFences(vk_device, 1, &vk_fence, VK_TRUE, timeout);
  vkResetFences(vk_device, 1, &vk_fence);

  command_buffer.begin_recording();
}

void VKCommandBuffers::submit()
{
  bool work_submitted = false;

  /* TODO data transfers should be queued together with compute or draw commands. */
  VKCommandBuffer &data_transfer = command_buffer_get(Type::DataTransfer);
  if (data_transfer.has_recorded_commands()) {
    submit_command_buffer(vk_device_, vk_queue_, data_transfer, vk_fence_, FenceTimeout);
    work_submitted = true;
  }

  VKCommandBuffer &compute = command_buffer_get(Type::Compute);
  if (compute.has_recorded_commands()) {
    submit_command_buffer(vk_device_, vk_queue_, compute, vk_fence_, FenceTimeout);
    work_submitted = true;
  }

  VKCommandBuffer &graphics = command_buffer_get(Type::Graphics);
  if (graphics.has_recorded_commands()) {
    submit_command_buffer(vk_device_, vk_queue_, graphics, vk_fence_, FenceTimeout);
    work_submitted = true;
  }

  if (work_submitted) {
    submission_id_.next();
  }
}

void VKCommandBuffers::ensure_no_compute_or_draw_commands()
{
  if (command_buffer_get(Type::Compute).has_recorded_commands() ||
      command_buffer_get(Type::Graphics).has_recorded_commands())
  {
    submit();
  }
}

void VKCommandBuffers::ensure_no_compute_commands()
{
  if (command_buffer_get(Type::Compute).has_recorded_commands()) {
    submit();
  }
}

void VKCommandBuffers::ensure_no_draw_commands()
{
  if (command_buffer_get(Type::Graphics).has_recorded_commands()) {
    submit();
  }
}

/**
 * \name Vulkan commands
 * \{
 */

void VKCommandBuffers::bind(const VKPipeline &vk_pipeline, VkPipelineBindPoint bind_point)
{
  Type type;
  if (bind_point == VK_PIPELINE_BIND_POINT_COMPUTE) {
    ensure_no_draw_commands();
    type = Type::Compute;
  }
  if (bind_point == VK_PIPELINE_BIND_POINT_GRAPHICS) {
    ensure_no_compute_commands();
    type = Type::Graphics;
  }
  command_buffer_get(type).bind(vk_pipeline, bind_point);
}

void VKCommandBuffers::bind(const VKDescriptorSet &descriptor_set,
                            const VkPipelineLayout vk_pipeline_layout,
                            VkPipelineBindPoint bind_point)
{
  Type type;
  if (bind_point == VK_PIPELINE_BIND_POINT_COMPUTE) {
    ensure_no_draw_commands();
    type = Type::Compute;
  }
  if (bind_point == VK_PIPELINE_BIND_POINT_GRAPHICS) {
    ensure_no_compute_commands();
    type = Type::Graphics;
  }
  command_buffer_get(type).bind(descriptor_set, vk_pipeline_layout, bind_point);
}

void VKCommandBuffers::bind(const uint32_t binding,
                            const VKVertexBuffer &vertex_buffer,
                            const VkDeviceSize offset)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).bind(binding, vertex_buffer, offset);
}

void VKCommandBuffers::bind(const uint32_t binding, const VKBufferWithOffset &vertex_buffer)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).bind(binding, vertex_buffer);
}

void VKCommandBuffers::bind(const uint32_t binding,
                            const VkBuffer &vk_vertex_buffer,
                            const VkDeviceSize offset)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).bind(binding, vk_vertex_buffer, offset);
}

void VKCommandBuffers::bind(const VKBufferWithOffset &index_buffer, VkIndexType index_type)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).bind(index_buffer, index_type);
}

void VKCommandBuffers::begin_render_pass(VKFrameBuffer &framebuffer)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).begin_render_pass(framebuffer);
}

void VKCommandBuffers::end_render_pass(const VKFrameBuffer &framebuffer)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).end_render_pass(framebuffer);
}

void VKCommandBuffers::push_constants(const VKPushConstants &push_constants,
                                      const VkPipelineLayout vk_pipeline_layout,
                                      const VkShaderStageFlags vk_shader_stages)
{
  Type type;
  if (vk_shader_stages == VK_SHADER_STAGE_COMPUTE_BIT) {
    ensure_no_draw_commands();
    type = Type::Compute;
  }
  else {
    ensure_no_compute_commands();
    type = Type::Graphics;
  }
  command_buffer_get(type).push_constants(push_constants, vk_pipeline_layout, vk_shader_stages);
}

void VKCommandBuffers::dispatch(int groups_x_len, int groups_y_len, int groups_z_len)
{
  ensure_no_draw_commands();
  command_buffer_get(Type::Compute).dispatch(groups_x_len, groups_y_len, groups_z_len);
}

void VKCommandBuffers::dispatch(VKStorageBuffer &command_buffer)
{
  ensure_no_draw_commands();
  command_buffer_get(Type::Compute).dispatch(command_buffer);
}

void VKCommandBuffers::copy(VKBuffer &dst_buffer,
                            VKTexture &src_texture,
                            Span<VkBufferImageCopy> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).copy(dst_buffer, src_texture, regions);
}

void VKCommandBuffers::copy(VKTexture &dst_texture,
                            VKBuffer &src_buffer,
                            Span<VkBufferImageCopy> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).copy(dst_texture, src_buffer, regions);
}

void VKCommandBuffers::copy(VKTexture &dst_texture,
                            VKTexture &src_texture,
                            Span<VkImageCopy> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).copy(dst_texture, src_texture, regions);
}

void VKCommandBuffers::copy(VKBuffer &dst_buffer, VkBuffer src_buffer, Span<VkBufferCopy> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).copy(dst_buffer, src_buffer, regions);
}

void VKCommandBuffers::blit(VKTexture &dst_texture,
                            VKTexture &src_texture,
                            Span<VkImageBlit> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).blit(dst_texture, src_texture, regions);
}

void VKCommandBuffers::blit(VKTexture &dst_texture,
                            VkImageLayout dst_layout,
                            VKTexture &src_texture,
                            VkImageLayout src_layout,
                            Span<VkImageBlit> regions)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer)
      .blit(dst_texture, dst_layout, src_texture, src_layout, regions);
}

void VKCommandBuffers::pipeline_barrier(VkPipelineStageFlags source_stages,
                                        VkPipelineStageFlags destination_stages)
{
  /* TODO: Command isn't used.*/
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).pipeline_barrier(source_stages, destination_stages);
}

void VKCommandBuffers::pipeline_barrier(Span<VkImageMemoryBarrier> image_memory_barriers)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).pipeline_barrier(image_memory_barriers);
}

void VKCommandBuffers::clear(VkImage vk_image,
                             VkImageLayout vk_image_layout,
                             const VkClearColorValue &vk_clear_color,
                             Span<VkImageSubresourceRange> ranges)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).clear(vk_image, vk_image_layout, vk_clear_color, ranges);
}

void VKCommandBuffers::clear(VkImage vk_image,
                             VkImageLayout vk_image_layout,
                             const VkClearDepthStencilValue &vk_clear_depth_stencil,
                             Span<VkImageSubresourceRange> ranges)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer)
      .clear(vk_image, vk_image_layout, vk_clear_depth_stencil, ranges);
}

void VKCommandBuffers::clear(Span<VkClearAttachment> attachments, Span<VkClearRect> areas)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).clear(attachments, areas);
}

void VKCommandBuffers::fill(VKBuffer &buffer, uint32_t data)
{
  ensure_no_compute_or_draw_commands();
  command_buffer_get(Type::DataTransfer).fill(buffer, data);
}

void VKCommandBuffers::draw(int v_first, int v_count, int i_first, int i_count)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).draw(v_first, v_count, i_first, i_count);
}

void VKCommandBuffers::draw_indexed(
    int index_count, int instance_count, int first_index, int vertex_offset, int first_instance)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics)
      .draw_indexed(index_count, instance_count, first_index, vertex_offset, first_instance);
}

void VKCommandBuffers::draw_indirect(const VKStorageBuffer &buffer,
                                     VkDeviceSize offset,
                                     uint32_t draw_count,
                                     uint32_t stride)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).draw_indirect(buffer, offset, draw_count, stride);
}

void VKCommandBuffers::draw_indexed_indirect(const VKStorageBuffer &buffer,
                                             VkDeviceSize offset,
                                             uint32_t draw_count,
                                             uint32_t stride)
{
  ensure_no_compute_commands();
  command_buffer_get(Type::Graphics).draw_indexed_indirect(buffer, offset, draw_count, stride);
}

/** \} */

}  // namespace blender::gpu