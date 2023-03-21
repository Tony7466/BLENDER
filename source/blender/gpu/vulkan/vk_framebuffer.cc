/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#include "vk_framebuffer.hh"
#include "vk_memory.hh"
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
                             VkRenderPass vk_render_pass,
                             VkExtent2D vk_extent)
    : FrameBuffer(name)
{
  immutable_ = true;
  /* Never update an internal frame-buffer. */
  dirty_attachments_ = false;
  width_ = vk_extent.width;
  height_ = vk_extent.height;
  vk_framebuffer_ = vk_framebuffer;
  vk_render_pass_ = vk_render_pass;

  viewport_[0] = scissor_[0] = 0;
  viewport_[1] = scissor_[1] = 0;
  viewport_[2] = scissor_[2] = width_;
  viewport_[3] = scissor_[3] = height_;
}

VKFrameBuffer::~VKFrameBuffer()
{
  if (!immutable_) {
    render_pass_free();
  }
}

/** \} */

void VKFrameBuffer::bind(bool /*enabled_srgb*/)
{
  update_attachments();

  VKContext &context = *VKContext::get();
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  VkRect2D render_area{};
  render_area.offset.x = 0;
  render_area.offset.y = 0;
  render_area.extent.width = width_;
  render_area.extent.height = height_;
  command_buffer.bind(vk_render_pass_, vk_framebuffer_, render_area);
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

  dirty_attachments_ = false;
}
void VKFrameBuffer::render_pass_create()
{
  BLI_assert(!immutable_);
  BLI_assert(vk_render_pass_ == VK_NULL_HANDLE);
  BLI_assert(vk_framebuffer_ == VK_NULL_HANDLE);

  VK_ALLOCATION_CALLBACKS

  /* Track first attachment for size.*/
  GPUAttachmentType first_attachment = GPU_FB_MAX_ATTACHMENT;

  Vector<VkAttachmentDescription> attachment_descriptions;
  Vector<VkAttachmentReference> color_attachments;
  Vector<VkImageView> image_views;

  for (int color_slot = 0; color_slot < GPU_FB_MAX_COLOR_ATTACHMENT; color_slot++) {
    GPUAttachment &attachment = attachments_[GPU_FB_COLOR_ATTACHMENT0 + color_slot];
    if (attachment.tex == nullptr) {
      continue;
    }

    if (first_attachment == GPU_FB_MAX_ATTACHMENT) {
      first_attachment = GPU_FB_COLOR_ATTACHMENT0 + color_slot;
    }

    VKTexture &texture = *static_cast<VKTexture *>(unwrap(attachment.tex));
    /* We might want to split framebuffer and render target....*/
    texture.ensure_allocated();
    image_views.append(texture.vk_image_view_handle());

    VkAttachmentDescription attachment_description{};
    attachment_description.format = to_vk_format(texture.format_get());
    attachment_description.samples = VK_SAMPLE_COUNT_1_BIT;
    attachment_description.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachment_description.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachment_description.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachment_description.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachment_description.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachment_description.finalLayout = VK_IMAGE_LAYOUT_GENERAL;
    attachment_descriptions.append(attachment_description);

    VkAttachmentReference color_attachment{};
    color_attachment.attachment = color_slot;
    color_attachment.layout = VK_IMAGE_LAYOUT_GENERAL;
    color_attachments.append(color_attachment);
  }

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = color_attachments.size();
  subpass.pColorAttachments = color_attachments.data();

  VkRenderPassCreateInfo render_pass_info{};
  render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
  render_pass_info.attachmentCount = attachment_descriptions.size();
  render_pass_info.pAttachments = attachment_descriptions.data();
  render_pass_info.subpassCount = 1;
  render_pass_info.pSubpasses = &subpass;

  VKContext &context = *VKContext::get();
  vkCreateRenderPass(
      context.device_get(), &render_pass_info, vk_allocation_callbacks, &vk_render_pass_);

  if (first_attachment != GPU_FB_MAX_ATTACHMENT) {
    GPUAttachment &attachment = attachments_[first_attachment];
    BLI_assert(attachment.tex);

    int size[3];
    GPU_texture_get_mipmap_size(attachment.tex, attachment.mip, size);
    this->size_set(size[0], size[1]);
  }
  else {
    this->size_set(0, 0);
  }

  VkFramebufferCreateInfo framebuffer_create_info{};
  framebuffer_create_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
  framebuffer_create_info.renderPass = vk_render_pass_;
  framebuffer_create_info.attachmentCount = image_views.size();
  framebuffer_create_info.pAttachments = image_views.data();
  framebuffer_create_info.width = width_;
  framebuffer_create_info.height = height_;
  framebuffer_create_info.layers = 1;

  vkCreateFramebuffer(
      context.device_get(), &framebuffer_create_info, vk_allocation_callbacks, &vk_framebuffer_);
}

void VKFrameBuffer::render_pass_free()
{
  BLI_assert(!immutable_);
  if (vk_render_pass_ == VK_NULL_HANDLE) {
    return;
  }
  VK_ALLOCATION_CALLBACKS

  VKContext &context = *VKContext::get();
  vkDestroyRenderPass(context.device_get(), vk_render_pass_, vk_allocation_callbacks);
  vkDestroyFramebuffer(context.device_get(), vk_framebuffer_, vk_allocation_callbacks);
  vk_render_pass_ = VK_NULL_HANDLE;
  vk_framebuffer_ = VK_NULL_HANDLE;
}

}  // namespace blender::gpu
