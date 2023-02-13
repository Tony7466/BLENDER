/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_command_buffer.hh"
#include "vk_context.hh"
#include "vk_memory.hh"

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

  if (vk_fence_ == VK_NULL_HANDLE) {
    VK_ALLOCATION_CALLBACKS;
    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    vkCreateFence(vk_device_, &fenceInfo, vk_allocation_callbacks, &vk_fence_);
  }
}

void VKCommandBuffer::begin_recording()
{
  vkWaitForFences(vk_device_, 1, &vk_fence_, VK_TRUE, UINT64_MAX);
  vkResetFences(vk_device_, 1, &vk_fence_);
  vkResetCommandBuffer(vk_command_buffer_, 0);

  VkCommandBufferBeginInfo begin_info = {};
  begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  vkBeginCommandBuffer(vk_command_buffer_, &begin_info);
}

void VKCommandBuffer::end_recording()
{
  vkEndCommandBuffer(vk_command_buffer_);
}

void VKCommandBuffer::bind(const VKPipeline &pipeline, VkPipelineBindPoint bind_point)
{
  vkCmdBindPipeline(vk_command_buffer_, bind_point, pipeline.vk_handle());
}
void VKCommandBuffer::bind(const VKDescriptorSet &descriptor_set,
                           const VkPipelineLayout vk_pipeline_layout,
                           VkPipelineBindPoint bind_point)
{
  VkDescriptorSet vk_descriptor_set = descriptor_set.vk_handle();
  vkCmdBindDescriptorSets(
      vk_command_buffer_, bind_point, vk_pipeline_layout, 0, 1, &vk_descriptor_set, 0, 0);
}

void VKCommandBuffer::dispatch(int groups_x_len, int groups_y_len, int groups_z_len)
{
  vkCmdDispatch(vk_command_buffer_, groups_x_len, groups_y_len, groups_z_len);
}

void VKCommandBuffer::submit()
{
  end_recording();
  encode_recorded_commands();
  submit_encoded_commands();
  begin_recording();
}

void VKCommandBuffer::encode_recorded_commands()
{
  /* TODO: Intentionally not implemented. For the graphics pipeline we want to extract the
   * resources and its usages so we can encode multiple commands in the same command buffer. This
   * will also require to change the begin/end recording to be only inside this function. */
}

void VKCommandBuffer::submit_encoded_commands()
{
  VkSubmitInfo submit_info = {};
  submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit_info.commandBufferCount = 1;
  submit_info.pCommandBuffers = &vk_command_buffer_;

  vkQueueSubmit(vk_queue_, 1, &submit_info, vk_fence_);
}

}  // namespace blender::gpu
