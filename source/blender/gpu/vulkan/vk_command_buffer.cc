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
