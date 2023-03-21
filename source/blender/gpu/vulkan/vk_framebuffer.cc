/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_framebuffer.hh"
#include "vk_texture.hh"

namespace blender::gpu {

/* -------------------------------------------------------------------- */
/** \name Creation & Deletion
 * \{ */

VKFrameBuffer::VKFrameBuffer(const char *name) : FrameBuffer(name)
{
  immutable_ = false;
}

VKFrameBuffer::VKFrameBuffer(const char *name,
                             VkFramebuffer vk_framebuffer,
                             VkRenderPass /*vk_render_pass*/,
                             VkExtent2D vk_extent)
    : FrameBuffer(name)
{
  immutable_ = true;
  /* Never update an internal frame-buffer. */
  dirty_attachments_ = false;
  width_ = vk_extent.width;
  height_ = vk_extent.height;
  vk_framebuffer_ = vk_framebuffer;

  viewport_[0] = scissor_[0] = 0;
  viewport_[1] = scissor_[1] = 0;
  viewport_[2] = scissor_[2] = width_;
  viewport_[3] = scissor_[3] = height_;
}

VKFrameBuffer::~VKFrameBuffer()
{
  if (!immutable_ && vk_framebuffer_ != VK_NULL_HANDLE) {
    vkDestroyFramebuffer(vk_device_, vk_framebuffer_, NULL);
  }
}

/** \} */

void VKFrameBuffer::bind(bool /*enabled_srgb*/)
{
  update_attachments();

  // VKContext &context = *VKContext::get();
  // context.framebuffer_bind(*this);
}

bool VKFrameBuffer::check(char /*err_out*/[256])
{
  return false;
}

void VKFrameBuffer::clear(eGPUFrameBufferBits buffers,
                          const float clear_col[4],
                          float clear_depth,
                          uint clear_stencil)
{
  Vector<VkClearAttachment> clear_attachments;

  if (buffers & (GPU_DEPTH_BIT | GPU_STENCIL_BIT)) {
    VkClearAttachment clear_attachment = {};
    clear_attachment.aspectMask = (buffers & GPU_DEPTH_BIT ? VK_IMAGE_ASPECT_DEPTH_BIT : 0) |
                                  (buffers & GPU_STENCIL_BIT ? VK_IMAGE_ASPECT_STENCIL_BIT : 0);
    clear_attachment.clearValue.depthStencil.depth = clear_depth;
    clear_attachment.clearValue.depthStencil.stencil = clear_stencil;
    clear_attachments.append(clear_attachment);
  }

  if (buffers & GPU_COLOR_BIT) {
    for (int color_slot = 0; color_slot < GPU_FB_MAX_COLOR_ATTACHMENT; color_slot++) {
      GPUAttachment &attachment = attachments_[GPU_FB_COLOR_ATTACHMENT0 + color_slot];
      if (attachment.tex == nullptr) {
        continue;
      }
      VkClearAttachment clear_attachment = {};
      clear_attachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      clear_attachment.colorAttachment = color_slot;
      copy_v4_v4(clear_attachment.clearValue.color.float32, clear_col);
      clear_attachments.append(clear_attachment);
    }
  }

  VkClearRect clear_rect = {};
  /* Extract to function? I expect I need this multiple times. */
  clear_rect.rect.offset.x = 1;
  clear_rect.rect.offset.y = 1;
  clear_rect.rect.extent.width = width_;
  clear_rect.rect.extent.height = height_;
  clear_rect.baseArrayLayer = 0;
  clear_rect.layerCount = 1;

  VKContext &context = *VKContext::get();
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  command_buffer.clear(clear_attachments, Span<VkClearRect>(&clear_rect, 1));
}

void VKFrameBuffer::clear_multi(const float (* /*clear_col*/)[4])
{
}

void VKFrameBuffer::clear_attachment(GPUAttachmentType /*type*/,
                                     eGPUDataFormat /*data_format*/,
                                     const void * /*clear_value*/)
{
}

void VKFrameBuffer::attachment_set_loadstore_op(GPUAttachmentType /*type*/,
                                                eGPULoadOp /*load_action*/,
                                                eGPUStoreOp /*store_action*/)
{
}

void VKFrameBuffer::read(eGPUFrameBufferBits /*planes*/,
                         eGPUDataFormat /*format*/,
                         const int /*area*/[4],
                         int /*channel_len*/,
                         int /*slot*/,
                         void * /*r_data*/)
{
}

void VKFrameBuffer::blit_to(eGPUFrameBufferBits /*planes*/,
                            int /*src_slot*/,
                            FrameBuffer * /*dst*/,
                            int /*dst_slot*/,
                            int /*dst_offset_x*/,
                            int /*dst_offset_y*/)
{
}

void VKFrameBuffer::update_attachments()
{
  if (immutable_) {
    return;
  }
  if (!dirty_attachments_) {
    return;
  }

  render_pass_free();
  render_pass_create();

  /*
    remove_all_attachments();
    Vec<VkAttachmentDescription> attachment_descriptors;

    for (GPUAttachmentType type = GPU_FB_MAX_ATTACHMENT - 1; type >= 0; --type) {
      GPUAttachment &attachment = attachments_[type];
      switch (type) {
        case GPU_FB_DEPTH_ATTACHMENT:
        case GPU_FB_DEPTH_STENCIL_ATTACHMENT:
      }
    }
    */

  dirty_attachments_ = false;
}
void VKFrameBuffer::render_pass_create()
{
  BLI_assert(!immutable_);
}

void VKFrameBuffer::render_pass_free()
{
  BLI_assert(!immutable_);
}

}  // namespace blender::gpu
