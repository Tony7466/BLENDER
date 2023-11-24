/* SPDX-FileCopyrightText: 2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_framebuffer.hh"
#include "vk_backend.hh"
#include "vk_common.hh"
#include "vk_context.hh"
#include "vk_image_views.hh"
#include "vk_memory.hh"
#include "vk_renderpass.hh"
#include "vk_state_manager.hh"
#include "vk_texture.hh"

namespace blender::gpu {
/* -------------------------------------------------------------------- */
/** \name Creation & Deletion
 * \{ */

VKFrameBuffer::VKFrameBuffer(const char *name) : FrameBuffer(name)
{
  size_set(1, 1);
  srgb_ = false;
  enabled_srgb_ = true;
  renderpass_.emplace(VKRenderPass());
  cache_init();
}

VKFrameBuffer::~VKFrameBuffer()
{
  renderpass_.reset();
  free();
}

/** \} */

void VKFrameBuffer::bind(bool enabled_srgb)
{
  VKContext &context = *VKContext::get();
  /* Updating attachments can issue pipeline barriers, this should be done outside the render pass.
   * When done inside a render pass there should be a self-dependency between sub-passes on the
   * active render pass. As the active render pass isn't aware of the new render pass (and should
   * not) it is better to deactivate it before updating the attachments. For more information check
   * `VkSubpassDependency`. */
  if (context.has_active_framebuffer()) {
    context.deactivate_framebuffer();
  }

  context.activate_framebuffer(*this);
  if (enabled_srgb_ != enabled_srgb) {
    dirty_attachments_8_ = default_attachments_;
    enabled_srgb_ = enabled_srgb;
  }
  Shader::set_framebuffer_srgb_target(enabled_srgb && srgb_);
}

Array<VkViewport, 16> VKFrameBuffer::vk_viewports_get() const
{
  Array<VkViewport, 16> viewports(this->multi_viewport_ ? GPU_MAX_VIEWPORTS : 1);

  int index = 0;
  for (VkViewport &viewport : viewports) {
    viewport.x = viewport_[index][0];
    viewport.y = viewport_[index][1];
    viewport.width = viewport_[index][2];
    viewport.height = viewport_[index][3];
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    index++;
  }
  return viewports;
}

Array<VkRect2D, 16> VKFrameBuffer::vk_render_areas_get() const
{
  Array<VkRect2D, 16> render_areas(this->multi_viewport_ ? GPU_MAX_VIEWPORTS : 1);

  for (VkRect2D &render_area : render_areas) {
    if (scissor_test_get()) {
      int scissor_rect[4];
      scissor_get(scissor_rect);
      render_area.offset.x = clamp_i(scissor_rect[0], 0, width_);
      render_area.offset.y = clamp_i(scissor_rect[1], 0, height_);
      render_area.extent.width = clamp_i(scissor_rect[2], 1, width_ - scissor_rect[0]);
      render_area.extent.height = clamp_i(scissor_rect[3], 1, height_ - scissor_rect[1]);
    }
    else {
      render_area.offset.x = 0;
      render_area.offset.y = 0;
      render_area.extent.width = width_;
      render_area.extent.height = height_;
    }
  }
  return render_areas;
}

bool VKFrameBuffer::check(char /*err_out*/[256])
{
  return true;
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
    eGPUDataFormat data_format = to_data_format(GPU_texture_format(attachment.tex));
    clear_attachment.clearValue.color = to_vk_clear_color_value(data_format,
                                                                &clear_colors[color_index]);
    r_attachments.append(clear_attachment);

    color_index += multi_clear_colors ? 1 : 0;
  }
}

/* -------------------------------------------------------------------- */
/** \name Clear
 * \{ */

void VKFrameBuffer::clear(const Vector<VkClearAttachment> &attachments) const
{
  if (attachments.is_empty()) {
    return;
  }
  VkClearRect clear_rect = {};
  clear_rect.rect = vk_render_areas_get()[0];
  clear_rect.baseArrayLayer = 0;
  clear_rect.layerCount = 1;

  VKContext &context = *VKContext::get();
  VKCommandBuffers &command_buffers = context.command_buffers_get();
  command_buffers.clear(attachments, Span<VkClearRect>(&clear_rect, 1));
}

void VKFrameBuffer::clear(const eGPUFrameBufferBits buffers,
                          const float clear_color[4],
                          float clear_depth,
                          uint clear_stencil)
{
  Vector<VkClearAttachment> attachments;
  if (buffers & (GPU_DEPTH_BIT | GPU_STENCIL_BIT)) {
    VKContext &context = *VKContext::get();
    eGPUWriteMask needed_mask = GPU_WRITE_NONE;
    if (buffers & GPU_DEPTH_BIT) {
      needed_mask |= GPU_WRITE_DEPTH;
    }
    if (buffers & GPU_STENCIL_BIT) {
      needed_mask |= GPU_WRITE_STENCIL;
    }

    /* Clearing depth via vkCmdClearAttachments requires a render pass with write depth or stencil
     * enabled. When not enabled, clearing should be done via texture directly. */
    if ((context.state_manager_get().state.write_mask & needed_mask) == needed_mask) {
      build_clear_attachments_depth_stencil(buffers, clear_depth, clear_stencil, attachments);
    }
    else {
      VKTexture *depth_texture = unwrap(unwrap(depth_tex()));
      if (depth_texture != nullptr) {
        if (G.debug & G_DEBUG_GPU) {
          std::cout
              << "PERFORMANCE: impact clearing depth texture in render pass that doesn't allow "
                 "depth writes.\n";
        }
        depth_attachment_layout_ensure(context, VK_IMAGE_LAYOUT_GENERAL);
        depth_texture->clear_depth_stencil(buffers, clear_depth, clear_stencil);
      }
    }
  }
  if (buffers & GPU_COLOR_BIT) {
    float clear_color_single[4];
    copy_v4_v4(clear_color_single, clear_color);
    build_clear_attachments_color(&clear_color_single, false, attachments);
  }

  if (!attachments.is_empty()) {
    clear(attachments);
  }
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
  /* Clearing of a single attachment was added to implement `clear_multi` in OpenGL. As
   * `clear_multi` is supported in Vulkan it isn't needed to implement this method.
   */
  BLI_assert_unreachable();
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Load/Store operations
 * \{ */

void VKFrameBuffer::attachment_set_loadstore_op(GPUAttachmentType /*type*/, GPULoadStore /*ls*/)
{
  NOT_YET_IMPLEMENTED;
}

/** \} */
void VKFrameBuffer::config(const GPUAttachment *config, int config_len)
{
  cache_init();
  const GPUAttachment &depth_attachment = config[0];
  Span<GPUAttachment> color_attachments(config + 1, config_len - 1);

  GPUAttachmentType type = GPU_FB_COLOR_ATTACHMENT0;
  for (const GPUAttachment &attachment : color_attachments) {
    attachment_set(type, attachment);
    ++type;
  }

  if (depth_attachment.mip == -1) {
    /* GPU_ATTACHMENT_LEAVE */
    attachment_set(GPU_FB_DEPTH_STENCIL_ATTACHMENT, depth_attachment);
  }
  else if (depth_attachment.tex == nullptr) {
    /* GPU_ATTACHMENT_NONE: Need to clear both targets. */
    attachment_set(GPU_FB_DEPTH_STENCIL_ATTACHMENT, depth_attachment);
  }
  else {
    GPUAttachmentType type = GPU_texture_has_stencil_format(depth_attachment.tex) ?
                                 GPU_FB_DEPTH_STENCIL_ATTACHMENT :
                                 GPU_FB_DEPTH_ATTACHMENT;
    attachment_set(type, depth_attachment);
  }
}
/* -------------------------------------------------------------------- */
/** \name Sub-pass transition
 * \{ */

void VKFrameBuffer::subpass_transition(const GPUAttachmentState /*depth_attachment_state*/,
                                       Span<GPUAttachmentState> /*color_attachment_states*/)
{
  NOT_YET_IMPLEMENTED;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Read back
 * \{ */

void VKFrameBuffer::read(eGPUFrameBufferBits plane,
                         eGPUDataFormat format,
                         const int area[4],
                         int /*channel_len*/,
                         int slot,
                         void *r_data)
{
  VKContext &context = *VKContext::get();
  GPUAttachment *attachment = nullptr;
  switch (plane) {
    case GPU_COLOR_BIT:
      attachment = &attachments_[GPU_FB_COLOR_ATTACHMENT0 + slot];
      color_attachment_layout_ensure(context, slot, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      break;

    case GPU_DEPTH_BIT:
      attachment = attachments_[GPU_FB_DEPTH_ATTACHMENT].tex ?
                       &attachments_[GPU_FB_DEPTH_ATTACHMENT] :
                       &attachments_[GPU_FB_DEPTH_STENCIL_ATTACHMENT];
      depth_attachment_layout_ensure(context, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      break;

    default:
      BLI_assert_unreachable();
      return;
  }

  VKTexture *texture = unwrap(unwrap(attachment->tex));
  BLI_assert_msg(texture,
                 "Trying to read back texture from framebuffer, but no texture is available in "
                 "requested slot.");
  if (texture == nullptr) {
    return;
  }

  IndexRange layers(max_ii(attachment->layer, 0), 1);
  texture->read_sub(0, format, area, layers, r_data);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Blit operations
 * \{ */

static void blit_aspect(VKCommandBuffers &command_buffer,
                        VKTexture &dst_texture,
                        VKTexture &src_texture,
                        int dst_offset_x,
                        int dst_offset_y,
                        VkImageAspectFlags image_aspect)
{
  /* Prefer texture copy, as some platforms don't support using D32_SFLOAT_S8_UINT to be used as
   * a blit destination. */
  if (dst_offset_x == 0 && dst_offset_y == 0 &&
      dst_texture.device_format_get() == src_texture.device_format_get() &&
      src_texture.width_get() == dst_texture.width_get() &&
      src_texture.height_get() == dst_texture.height_get())
  {
    src_texture.copy_to(dst_texture, image_aspect);
    return;
  }

  VkImageBlit image_blit = {};
  image_blit.srcSubresource.aspectMask = image_aspect;
  image_blit.srcSubresource.mipLevel = 0;
  image_blit.srcSubresource.baseArrayLayer = 0;
  image_blit.srcSubresource.layerCount = 1;
  image_blit.srcOffsets[0].x = 0;
  image_blit.srcOffsets[0].y = 0;
  image_blit.srcOffsets[0].z = 0;
  image_blit.srcOffsets[1].x = src_texture.width_get();
  image_blit.srcOffsets[1].y = src_texture.height_get();
  image_blit.srcOffsets[1].z = 1;

  image_blit.dstSubresource.aspectMask = image_aspect;
  image_blit.dstSubresource.mipLevel = 0;
  image_blit.dstSubresource.baseArrayLayer = 0;
  image_blit.dstSubresource.layerCount = 1;
  image_blit.dstOffsets[0].x = min_ii(dst_offset_x, dst_texture.width_get());
  image_blit.dstOffsets[0].y = min_ii(dst_offset_y, dst_texture.height_get());
  image_blit.dstOffsets[0].z = 0;
  image_blit.dstOffsets[1].x = min_ii(dst_offset_x + src_texture.width_get(),
                                      dst_texture.width_get());
  image_blit.dstOffsets[1].y = min_ii(dst_offset_y + src_texture.height_get(),
                                      dst_texture.height_get());
  image_blit.dstOffsets[1].z = 1;

  command_buffer.blit(dst_texture, src_texture, Span<VkImageBlit>(&image_blit, 1));
}

void VKFrameBuffer::blit_to(eGPUFrameBufferBits planes,
                            int src_slot,
                            FrameBuffer *dst,
                            int dst_slot,
                            int dst_offset_x,
                            int dst_offset_y)
{
  BLI_assert(dst);
  BLI_assert_msg(ELEM(planes, GPU_COLOR_BIT, GPU_DEPTH_BIT),
                 "VKFrameBuffer::blit_to only supports a single color or depth aspect.");
  UNUSED_VARS_NDEBUG(planes);

  VKContext &context = *VKContext::get();
  VKCommandBuffers &command_buffers = context.command_buffers_get();
  if (!context.has_active_framebuffer()) {
    BLI_assert_unreachable();
    return;
  }

  VKFrameBuffer &dst_framebuffer = *unwrap(dst);
  if (planes & GPU_COLOR_BIT) {
    const GPUAttachment &src_attachment = attachments_[GPU_FB_COLOR_ATTACHMENT0 + src_slot];
    const GPUAttachment &dst_attachment =
        dst_framebuffer.attachments_[GPU_FB_COLOR_ATTACHMENT0 + dst_slot];
    if (src_attachment.tex && dst_attachment.tex) {
      VKTexture &src_texture = *unwrap(unwrap(src_attachment.tex));
      VKTexture &dst_texture = *unwrap(unwrap(dst_attachment.tex));
      color_attachment_layout_ensure(context, src_slot, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      dst_framebuffer.color_attachment_layout_ensure(
          context, dst_slot, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

      blit_aspect(command_buffers,
                  dst_texture,
                  src_texture,
                  dst_offset_x,
                  dst_offset_y,
                  VK_IMAGE_ASPECT_COLOR_BIT);
      color_attachment_layout_ensure(context, src_slot, VK_IMAGE_LAYOUT_MAX_ENUM);
      dst_framebuffer.color_attachment_layout_ensure(context, dst_slot, VK_IMAGE_LAYOUT_MAX_ENUM);
    }
  }

  if (planes & GPU_DEPTH_BIT) {
    /* Retrieve source texture. */
    const GPUAttachment &src_attachment = attachments_[GPU_FB_DEPTH_STENCIL_ATTACHMENT].tex ?
                                              attachments_[GPU_FB_DEPTH_STENCIL_ATTACHMENT] :
                                              attachments_[GPU_FB_DEPTH_ATTACHMENT];
    const GPUAttachment &dst_attachment =
        dst_framebuffer.attachments_[GPU_FB_DEPTH_STENCIL_ATTACHMENT].tex ?
            dst_framebuffer.attachments_[GPU_FB_DEPTH_STENCIL_ATTACHMENT] :
            dst_framebuffer.attachments_[GPU_FB_DEPTH_ATTACHMENT];
    if (src_attachment.tex && dst_attachment.tex) {
      VKTexture &src_texture = *unwrap(unwrap(src_attachment.tex));
      VKTexture &dst_texture = *unwrap(unwrap(dst_attachment.tex));
      depth_attachment_layout_ensure(context, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
      dst_framebuffer.depth_attachment_layout_ensure(context,
                                                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

      blit_aspect(command_buffers,
                  dst_texture,
                  src_texture,
                  dst_offset_x,
                  dst_offset_y,
                  VK_IMAGE_ASPECT_DEPTH_BIT);
      depth_attachment_layout_ensure(context, VK_IMAGE_LAYOUT_MAX_ENUM);
      dst_framebuffer.depth_attachment_layout_ensure(context, VK_IMAGE_LAYOUT_MAX_ENUM);
    }
  }
}

void VKFrameBuffer::cache_init()
{
  free();
  vk_framebuffer_create_info_.attachmentCount = 0;
  width_ = 0;
  height_ = 0;

  default_attachments_ = dirty_attachments_8_ = 0;
  renderpass_->cache_init();
};

void VKFrameBuffer::attachment_set(GPUAttachmentType type,
                                   const GPUAttachment &new_attachment,
                                   bool config)

{
  /* Render passes do not require instances, so they should be generated ahead of schedule and be
   * static. */
  BLI_assert(type < GPU_FB_MAX_ATTACHMENT);
  GPUAttachment &attachment = attachments_[type];
  bool leave = false;
  if (new_attachment.mip == -1) {
    if (attachment.tex != nullptr) {
      /* GPU_ATTACHMENT_LEAVE */
      leave = true;
    }
    else if (type >= GPU_FB_COLOR_ATTACHMENT0) {
      return; /* Ignore */
    }
  }

  /* Add the parameter 'config' to indicate whether a new render pass should be generated. */
  config = (width_ == 0) || config;
  if (!config) {
    if (attachment.tex == new_attachment.tex && attachment.layer == new_attachment.layer &&
        attachment.mip == new_attachment.mip && (type >= GPU_FB_COLOR_ATTACHMENT0))
    {
      return; /* Exact same texture already bound here. */
    }
  }
  if (!leave && attachment.tex) {
    /* The texture entity has nothing to do with render pass regeneration. */
    reinterpret_cast<Texture *>(attachment.tex)->detach_from(this);
  }

  VkAttachmentReference2 *attachment_reference = nullptr;
  bool dirty_flag = false;
  bool dirty_view = false;
  bool view_set = false;
  /* If we change to `LEAVE`, there will be no new settings, but if the render pass already has
   * that description, it will be saved. */
  auto *tex = (leave) ? attachment.tex : new_attachment.tex;
  int atta_layer = (leave) ? attachment.layer : new_attachment.layer;
  int atta_mip = (leave) ? attachment.mip : new_attachment.mip;
  if (config) {
    if ((renderpass_->attachments_.idx_[renderpass_->info_id_][type] != VK_ATTACHMENT_UNUSED) &&
        (renderpass_->attachments_.idx_[renderpass_->info_id_][type] != VK_ATTACHMENT_EMPTY))
    {
      renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount--;
    };
    /* Set the current information on the `render_pass_info_id_` side and compare it with the
     * information cached in the reverse id. */
    if (type >= GPU_FB_COLOR_ATTACHMENT0) {
      int type_index = static_cast<int>(type) - static_cast<int>(GPU_FB_COLOR_ATTACHMENT0);
      if (vk_framebuffer_create_info_.attachmentCount > 0) {
        /* If the frame buffer information is not the default value, a color attachment change is
         * forced. */
        view_set = true;
      }
      /* Specify the order of reference as an index, starting with the description of each subpass.
       */
      attachment_reference = renderpass_->attachments_.reference_get(type_index,
                                                                     renderpass_->info_id_);
      renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount = max_ii(
          renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount, type_index + 1);

      attachment_reference->aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      attachment_reference->layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

      /* Whether to increment the number of attachments actually used. */
      if (tex) {
        attachment_reference->attachment =
            renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount++;
      }
      else {
        attachment_reference->attachment = VK_ATTACHMENT_UNUSED;
      }
      /* Take a cache to look up the order in the render pass information from the number of
          GPUAttachmentTypes. */
      renderpass_->attachments_.idx_[renderpass_->info_id_][type] =
          attachment_reference->attachment;
      if (renderpass_->attachments_.idx_[renderpass_->info_id_counter()][type] !=
          attachment_reference->attachment)
      {
        /* we have been asked to change the number of attachments for this frame buffer.  */
        dirty_flag = true;
      }
    }
    else {
      /* Depth configuration should always be done last. */
      view_set = true;
      attachment_reference = &renderpass_->attachments_.depth_references_[renderpass_->info_id_];
      if (tex) {
        /* Use depth-attachment. */
        renderpass_->attachments_.idx_[renderpass_->info_id_][type] =
            renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount++;
        VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
        if (GPU_texture_has_stencil_format(tex)) {
          BLI_assert(ELEM(type, GPU_FB_DEPTH_STENCIL_ATTACHMENT));
        }
        else if (GPU_texture_has_depth_format(tex)) {
          BLI_assert(ELEM(type, GPU_FB_DEPTH_ATTACHMENT));
        }

        attachment_reference->aspectMask = to_vk_image_aspect_flag_bits(
            texture.device_format_get());
        attachment_reference->layout = vk_aspect_to_layout(attachment_reference->aspectMask);
        attachment_reference->attachment =
            renderpass_->attachments_.idx_[renderpass_->info_id_][type];
        renderpass_->subpass_[renderpass_->info_id_].pDepthStencilAttachment =
            attachment_reference;
        if (renderpass_->attachments_.idx_[renderpass_->info_id_counter()][type] !=
            attachment_reference->attachment)
        {
          dirty_flag = true;
        }
      }
      else {
        BLI_assert(renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount > 0);
        if (renderpass_->attachments_.idx_[renderpass_->info_id_][type] != VK_ATTACHMENT_UNUSED) {
          renderpass_->attachments_.idx_[renderpass_->info_id_][type] = VK_ATTACHMENT_UNUSED;
        }
        renderpass_->subpass_[renderpass_->info_id_].pDepthStencilAttachment = nullptr;
        if (renderpass_->subpass_[renderpass_->info_id_].pDepthStencilAttachment !=
            renderpass_->subpass_[renderpass_->info_id_counter()].pDepthStencilAttachment)
        {
          /* A change has been requested for the use of depth. */
          dirty_flag = true;
        }
      }
    }
  }
  else {
    /* If configuration is not required, there are countless situations in which regeneration is
     * not necessary. */
    if (type >= GPU_FB_COLOR_ATTACHMENT0) {
      int type_index = static_cast<int>(type) - static_cast<int>(GPU_FB_COLOR_ATTACHMENT0);
      attachment_reference =
          &renderpass_->attachments_.references_[renderpass_->info_id_][type_index];
    }
    else {
      attachment_reference = &renderpass_->attachments_.depth_references_[renderpass_->info_id_];
    }
  }
  /* Create render pass. */
  if (tex) {
    if (atta_layer > 0) {
      BLI_assert(GPU_texture_is_cube(tex) || GPU_texture_is_array(tex));
    }
    if (!leave) {
      reinterpret_cast<Texture *>(tex)->attach_to(this, type);
    }

    dirty_view |= image_view_ensure(tex, atta_mip, atta_layer, attachment_reference->attachment);

    VkAttachmentDescription2 &attachment_description =
        renderpass_->attachments_
            .descriptions_[renderpass_->info_id_][attachment_reference->attachment];
    if (config) {
      renderpass_->attachments_.description_set(
          tex, *attachment_reference, attachment_description, renderpass_->render_pass_enum_);
      VkAttachmentDescription2 &attachment_description_cache =
          renderpass_->attachments_
              .descriptions_[renderpass_->info_id_counter()][attachment_reference->attachment];
      if ((attachment_description.finalLayout != attachment_description_cache.finalLayout) ||
          (attachment_description.format != attachment_description_cache.format))
      {
        /* A change in the attachment format or transition structure is required. */
        dirty_flag = true;
      }
    }
    else {
      VkAttachmentDescription2 &attachment_description_cache =
          renderpass_->attachments_
              .descriptions_[renderpass_->info_id_counter()][attachment_reference->attachment];
      renderpass_->attachments_.description_set(tex,
                                                *attachment_reference,
                                                attachment_description_cache,
                                                renderpass_->render_pass_enum_);
      if ((attachment_description.finalLayout != attachment_description_cache.finalLayout) ||
          (attachment_description.format != attachment_description_cache.format))
      {
        BLI_assert_msg(false,
                       "A request is being made that is not static.This is not expected.\n");
      }
    }
  }
  if (view_set) {
    vk_framebuffer_create_info_.attachmentCount =
        renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount;
    if ((renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount !=
         renderpass_->vk_create_info_[renderpass_->info_id_counter()].attachmentCount) ||
        (renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount !=
         renderpass_->subpass_[renderpass_->info_id_counter()].colorAttachmentCount))
    {
      /* The number of attachments is changed. */
      dirty_flag = true;
      renderpass_->dependency_set(
          renderpass_->subpass_[renderpass_->info_id_].pDepthStencilAttachment != nullptr);
    }
    if (!dirty_flag) {
      /* No regeneration is required. */
      renderpass_->info_id_ = renderpass_->info_id_counter();
    }
  }
  if (atta_mip > -1) {
    if (!leave) {
      attachment = new_attachment;
    }
    dirty_view |= true;  // dirty_flag;
    if ((attachment_reference->attachment != VK_ATTACHMENT_UNUSED) && dirty_view) {
      dirty_attachments_8_ |= (1 << attachment_reference->attachment);
      if (config) {
        default_attachments_ |= (1 << attachment_reference->attachment);
      }
    }
  }
  renderpass_->dirty_ |= dirty_flag;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Update attachments
 * \{ */

void VKFrameBuffer::renderpass_ensure()
{
  if (dirty_attachments_) {
    dirty_attachments_8_ = default_attachments_;
    if (default_attachments_ == 0) {
      renderpass_->ensure();
      create();
      return;
    }
    dirty_attachments_ = false;
  };

  if (!dirty_attachments_8_) {
    return;
  }

  renderpass_->ensure();
  ensure();
  dirty_attachments_8_ = 0;
}
void VKFrameBuffer::create()
{
  /*
   * If we need to create a frame buffer.
     1: Render Pass Change
     2: ImageView Change
     3: When it comes to viewport changes, use dynamic state, don't regenerate frame buffers.
   */

  thread_local int size[3] = {0, 0, 0};
  size[0] = -1;
  Vector<VkImageView> data;
  for (int i = 0; i < vk_framebuffer_create_info_.attachmentCount; i++) {
    data.append(image_views_[i].lock()->vk_handle());
    if (size[0] == -1) {
      int type = 0;
      if (renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount > 0) {
        for (int j = 0; j < 8; j++) {
          if (renderpass_->subpass_[renderpass_->info_id_].pColorAttachments[j].attachment ==
              VK_ATTACHMENT_UNUSED)
          {
            continue;
          }
          type = renderpass_->attachments_.type_get(
              renderpass_->subpass_[renderpass_->info_id_].pColorAttachments[i].attachment,
              renderpass_->info_id_);
          const GPUAttachment &attachment = attachments_[type];
          VKTexture *tex = reinterpret_cast<VKTexture *>(attachment.tex);
          tex->mip_size_get(attachment.mip, size);
          vk_framebuffer_create_info_.layers = (attachment.layer == -1) ? tex->vk_layer_count(1) :
                                                                          1;
          break;
        }
      }

      if (size[0] == -1) {
        type = renderpass_->attachments_.type_get(
            renderpass_->subpass_[renderpass_->info_id_].pDepthStencilAttachment->attachment,
            renderpass_->info_id_);
        const GPUAttachment &attachment = attachments_[type];
        VKTexture *tex = reinterpret_cast<VKTexture *>(attachment.tex);
        tex->mip_size_get(attachment.mip, size);
        vk_framebuffer_create_info_.layers = (attachment.layer == -1) ? tex->vk_layer_count(1) : 1;
      }
    }
  }

  BLI_assert(size[0] > 0);
  size_set(size[0], size[1]);
  vk_framebuffer_create_info_.renderPass = renderpass_->vk_render_pass_;
  vk_framebuffer_create_info_.width = width_;
  vk_framebuffer_create_info_.height = height_;
  vk_framebuffer_create_info_.pAttachments = data.data();
  /* TODO:Multilayered rendering*/
  viewport_reset();
  scissor_reset();
  free();
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateFramebuffer(device.device_get(),
                      &vk_framebuffer_create_info_,
                      vk_allocation_callbacks,
                      &vk_framebuffer_);
  debug::object_label(vk_framebuffer_, name_);
};

void VKFrameBuffer::free()
{
  if (vk_framebuffer_ == VK_NULL_HANDLE) {
    return;
  }
  VKDevice &device = VKBackend::get().device_get();
  if (device.is_initialized()) {
    device.discard_frame_buffer(vk_framebuffer_);
  }
  vk_framebuffer_ = VK_NULL_HANDLE;
}

void VKFrameBuffer::ensure()
{
  if (dirty_attachments_8_ == 0) {
    renderpass_->ensure();
    return;
  }

  bool flush = false;
  int N = renderpass_->vk_create_info_[renderpass_->info_id_].attachmentCount;

  for (int i = 0; i < N; i++) {
    if ((dirty_attachments_8_ >> i) & 1) {
      int type = renderpass_->attachments_.type_get(i, renderpass_->info_id_);
      const GPUAttachment &attachment = attachments_[type];
      if (!attachment.tex) {
        flush = true;
        continue;
      }
      VkAttachmentReference2 *attachment_reference = renderpass_->attachments_.reference_get(
          ((type < 2) ? -1 : renderpass_->attachments_.idx_[renderpass_->info_id_][type]),
          renderpass_->info_id_);
      if (attachment_reference->attachment == VK_ATTACHMENT_UNUSED) {
        continue;
      }
      flush |= image_view_ensure(
          attachment.tex, attachment.mip, attachment.layer, attachment_reference->attachment);
    }
  }
  dirty_attachments_8_ = 0;
  flush |= renderpass_->ensure();
  if (flush || vk_framebuffer_ == VK_NULL_HANDLE) {
    create();
  }

  return;
}

bool VKFrameBuffer::image_view_ensure(GPUTexture *tex, int mip, int layer, int attachment_index)
{
  VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
  if (texture.is_format_dirty(eImageViewUsage::Attachment, enabled_srgb_)) {
    VkAttachmentDescription2 &attachment_description =
        renderpass_->attachments_.descriptions_[renderpass_->info_id_][attachment_index];
    renderpass_->dirty_ |= !renderpass_->attachments_.check_format(
        texture, attachment_description, enabled_srgb_);
  };
  std::weak_ptr<VKImageView> image_view = texture.image_view_get(enabled_srgb_, mip, layer);

  if (image_views_[attachment_index].expired() ||
      !vk_image_view_equal(image_views_[attachment_index], image_view))
  {
    image_views_[attachment_index] = image_view;
    return true;
  }
  return false;
}

void VKFrameBuffer::color_attachment_layout_ensure(VKContext &context,
                                                   int color_attachment,
                                                   VkImageLayout requested_layout)
{
  VKTexture *color_texture = unwrap(unwrap(color_tex(color_attachment)));
  if (color_texture == nullptr) {
    return;
  }
  if (requested_layout == VK_IMAGE_LAYOUT_MAX_ENUM) {
    requested_layout = color_texture->best_layout_get();
  }
  if (color_texture->current_layout_get() == requested_layout) {
    return;
  }

  color_texture->layout_ensure(context, requested_layout);
}

void VKFrameBuffer::depth_attachment_layout_ensure(VKContext &context,
                                                   VkImageLayout requested_layout)
{
  VKTexture *depth_texture = unwrap(unwrap(depth_tex()));
  if (depth_texture == nullptr) {
    return;
  }
  if (requested_layout == VK_IMAGE_LAYOUT_MAX_ENUM) {
    requested_layout = depth_texture->best_layout_get();
  }
  if (depth_texture->current_layout_get() == requested_layout) {
    return;
  }
  depth_texture->layout_ensure(context, requested_layout);
}

void VKFrameBuffer::update_size()
{
  if (!dirty_attachments_) {
    return;
  }

  for (int i = 0; i < GPU_FB_MAX_ATTACHMENT; i++) {
    GPUAttachment &attachment = attachments_[i];
    if (attachment.tex) {
      int size[3];
      GPU_texture_get_mipmap_size(attachment.tex, attachment.mip, size);
      size_set(size[0], size[1]);
      return;
    }
  }
  size_set(1, 1);
}

void VKFrameBuffer::update_srgb()
{
  for (int i : IndexRange(GPU_FB_MAX_COLOR_ATTACHMENT)) {
    VKTexture *texture = unwrap(unwrap(color_tex(i)));
    if (texture) {
      srgb_ = (texture->format_flag_get() & GPU_FORMAT_SRGB) != 0;
      return;
    }
  }
}

int VKFrameBuffer::color_attachments_resource_size() const
{
  return renderpass_->subpass_[renderpass_->info_id_].colorAttachmentCount;
}

VkRenderPass VKFrameBuffer::vk_render_pass_get()
{
  renderpass_->ensure();
  return renderpass_->vk_render_pass_;
}
/** \} */

}  // namespace blender::gpu
