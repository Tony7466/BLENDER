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
  context.activate_framebuffer(*this);
}

VkRect2D VKFrameBuffer::vk_render_area_get() const
{
  VkRect2D render_area{};
  render_area.offset.x = 0;
  render_area.offset.y = 0;
  render_area.extent.width = width_;
  render_area.extent.height = height_;
  return render_area;
}

bool VKFrameBuffer::check(char /*err_out*/[256])
{
  return false;
}

void VKFrameBuffer::build_clear_attachments_depth_stencil(
    const eGPUFrameBufferBits buffers,
    float clear_depth,
    uint32_t clear_stencil,
    Vector<VkClearAttachment> &r_attachments) const
{
  VkClearAttachment clear_attachment = {};
  clear_attachment.aspectMask = (buffers & GPU_DEPTH_BIT ? VK_IMAGE_ASPECT_DEPTH_BIT : 0) |
                                (buffers & GPU_STENCIL_BIT ? VK_IMAGE_ASPECT_STENCIL_BIT : 0);
  clear_attachment.clearValue.depthStencil.depth = clear_depth;
  clear_attachment.clearValue.depthStencil.stencil = clear_stencil;
  r_attachments.append(clear_attachment);
}

void VKFrameBuffer::build_clear_attachments_color(const float (*clear_colors)[4],
                                                  const bool multi_clear_colors,
                                                  Vector<VkClearAttachment> &r_attachments) const
{
  int color_index = 0;
  for (int color_slot = 0; color_slot < GPU_FB_MAX_COLOR_ATTACHMENT; color_slot++) {
    const GPUAttachment &attachment = attachments_[GPU_FB_COLOR_ATTACHMENT0 + color_slot];
    if (attachment.tex == nullptr) {
      continue;
    }
    VkClearAttachment clear_attachment = {};
    clear_attachment.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    clear_attachment.colorAttachment = color_slot;
    copy_v4_v4(clear_attachment.clearValue.color.float32, clear_colors[color_index]);
    r_attachments.append(clear_attachment);

    color_index += multi_clear_colors ? 1 : 0;
  }
}

void VKFrameBuffer::clear(const Vector<VkClearAttachment> &attachments) const
{
  VkClearRect clear_rect = {};
  clear_rect.rect = vk_render_area_get();
  clear_rect.baseArrayLayer = 0;
  clear_rect.layerCount = 1;

  VKContext &context = *VKContext::get();
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  command_buffer.clear(attachments, Span<VkClearRect>(&clear_rect, 1));
}

void VKFrameBuffer::clear(const eGPUFrameBufferBits buffers,
                          const float clear_color[4],
                          float clear_depth,
                          uint clear_stencil)
{
  Vector<VkClearAttachment> attachments;
  if (buffers & (GPU_DEPTH_BIT | GPU_STENCIL_BIT)) {
    build_clear_attachments_depth_stencil(buffers, clear_depth, clear_stencil, attachments);
  }
  if (buffers & GPU_COLOR_BIT) {
    float clear_color_single[4];
    copy_v4_v4(clear_color_single, clear_color);
    build_clear_attachments_color(&clear_color_single, false, attachments);
  }
  clear(attachments);
}

void VKFrameBuffer::clear_multi(const float (*clear_color)[4])
{
  Vector<VkClearAttachment> attachments;
  build_clear_attachments_color(clear_color, true, attachments);
  clear(attachments);
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
  VkAttachmentReference depth_attachment = {};
  bool has_depth_attachment = false;

  for (int type = GPU_FB_DEPTH_ATTACHMENT; type < GPU_FB_MAX_ATTACHMENT; type++) {
    GPUAttachment &attachment = attachments_[type];
    if (attachment.tex == nullptr) {
      continue;
    }

    if (first_attachment == GPU_FB_MAX_ATTACHMENT) {
      first_attachment = static_cast<GPUAttachmentType>(type);
    }

    VKTexture &texture = *static_cast<VKTexture *>(unwrap(attachment.tex));
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

    /* TODO: should we also add unused attachments so the attachment reflect the internal lists.*/
    int attachment_index = attachment_descriptions.size() - 1;

    switch (type) {
      case GPU_FB_DEPTH_ATTACHMENT:
        BLI_assert(!has_depth_attachment);
        has_depth_attachment = true;
        depth_attachment.attachment = attachment_index;
        depth_attachment.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        break;

      case GPU_FB_DEPTH_STENCIL_ATTACHMENT:
        BLI_assert(!has_depth_attachment);
        has_depth_attachment = true;
        depth_attachment.attachment = attachment_index;
        depth_attachment.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
        break;

      case GPU_FB_COLOR_ATTACHMENT0:
      case GPU_FB_COLOR_ATTACHMENT1:
      case GPU_FB_COLOR_ATTACHMENT2:
      case GPU_FB_COLOR_ATTACHMENT3:
      case GPU_FB_COLOR_ATTACHMENT4:
      case GPU_FB_COLOR_ATTACHMENT5:
      case GPU_FB_COLOR_ATTACHMENT6:
      case GPU_FB_COLOR_ATTACHMENT7:
        VkAttachmentReference color_attachment{};
        color_attachment.attachment = attachment_index;
        color_attachment.layout = VK_IMAGE_LAYOUT_GENERAL;
        color_attachments.append(color_attachment);
    }
  }

  VkSubpassDescription subpass{};
  subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
  subpass.colorAttachmentCount = color_attachments.size();
  subpass.pColorAttachments = color_attachments.data();
  if (has_depth_attachment) {
    subpass.pDepthStencilAttachment = &depth_attachment;
  }

  VkRenderPassCreateInfo render_pass_info = {};
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

  /* We might want to split framebuffer and render target....*/
  VkFramebufferCreateInfo framebuffer_create_info = {};
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
