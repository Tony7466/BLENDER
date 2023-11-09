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
#include "vk_state_manager.hh"
#include "vk_texture.hh"

namespace blender::gpu {
/* Dependencies when transitioning from render target to render target. There are no transitions.*/
static VkSubpassDependency2 dependencies[2] = {
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     0,
     VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_ACCESS_MEMORY_READ_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     VK_SUBPASS_EXTERNAL,
     0,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
};

/* Dependencies when transitioning from ShaderRead to ShaderRead. Color:3trans + Depth:3trans =
 * 6trans*/
static VkSubpassDependency2 dependencies_shader_read[6] = {
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     VK_SUBPASS_EXTERNAL,
     0,
     VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_ACCESS_SHADER_READ_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     0,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     VK_SUBPASS_EXTERNAL,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
     VK_ACCESS_SHADER_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     VK_SUBPASS_EXTERNAL,
     0,
     VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
     VK_ACCESS_SHADER_READ_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     0,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT,
     VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     VK_SUBPASS_EXTERNAL,
     VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_PIPELINE_STAGE_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
     VK_ACCESS_SHADER_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT,
     0}};

static IndexRange layer_range_get(GPUTexture *tex, int layer)
{
  /* At the time of VkImage generation, the multi-layer type is recognized. */
  IndexRange layers_range(0, 0);
  if (layer > -1 && GPU_texture_is_cube(tex) && !GPU_texture_is_array(tex)) {
    /**
     * TODO:: When MultiView is enabled, the layer is set to 1,
     * making it a multilayer render that relies on the attributes of the sub-pass
     * description.
     * Availability of CubeMap is another issue.
     */
    layers_range = IndexRange(max_ii(layer, 0), 1);
  }
  else if (layer > -1) {
    layers_range = IndexRange(max_ii(layer, 0), 1);
  }
  return layers_range;
};

/* -------------------------------------------------------------------- */
/** \name Creation & Deletion
 * \{ */

VKFrameBuffer::VKFrameBuffer(const char *name) : FrameBuffer(name)
{
  size_set(1, 1);
  srgb_ = false;
  enabled_srgb_ = true;
  render_pass_free();
  flush();
}

VKFrameBuffer::~VKFrameBuffer()
{
  render_pass_free();
  frame_buffer_free();
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
    /* Clearing depth via vkCmdClearAttachments requires a render pass with write depth enabled.
     * When not enabled, clearing should be done via texture directly. */
    if (context.state_manager_get().state.write_mask & GPU_WRITE_DEPTH) {
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
  flush();
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
      dst_texture.format_get() == src_texture.format_get() &&
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

void VKFrameBuffer::flush()
{
  render_pass_info_id_cache_ = render_pass_info_id_;
  render_pass_info_id_ = (render_pass_info_id_ + 1) % 2;
  dirty_render_pass_ = false;

  default_attachments_ = dirty_attachments_8_ = 0;
  /* Number of ShaderOutputs.*/
  subpass_[render_pass_info_id_].colorAttachmentCount = 0;
  subpass_[render_pass_info_id_].pColorAttachments =
      attachment_references_[render_pass_info_id_].data();
  subpass_[render_pass_info_id_].pDepthStencilAttachment = nullptr;
  /* Number of non-null ImageViews. */
  vk_render_pass_info_[render_pass_info_id_].attachmentCount = 0;
  vk_render_pass_info_[render_pass_info_id_].pAttachments =
      attachment_descriptions_[render_pass_info_id_].data();
  vk_render_pass_info_[render_pass_info_id_].subpassCount = 1;
  vk_render_pass_info_[render_pass_info_id_].pSubpasses = &subpass_[render_pass_info_id_];
  render_pass_enum_ = VKRenderPassTransition::ALL;
  vk_framebuffer_create_info_.attachmentCount = 0;
  layers = 1;
  sep_layer = 0;
  width_ = 0;
  height_ = 0;
  for (auto &i : attachment_idx[render_pass_info_id_]) {
    i = -1;
  }
  frame_buffer_free();
};

void VKFrameBuffer::dependency_set(bool use_depth)
{
  if (render_pass_enum_ == VKRenderPassTransition::ALL) {
    return;
  }
  BLI_assert(ELEM(render_pass_enum_, VKRenderPassTransition::A2A, VKRenderPassTransition::S2S));
  if (subpass_[render_pass_info_id_].colorAttachmentCount == 0) {
    if (render_pass_enum_ == VKRenderPassTransition::A2A) {
      vk_render_pass_info_[render_pass_info_id_].dependencyCount = 1;
      vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies[1];
    }
    else {
      vk_render_pass_info_[render_pass_info_id_].dependencyCount = 3;
      vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies_shader_read[3];
    }
  }
  else {
    if (use_depth) {
      if (render_pass_enum_ == VKRenderPassTransition::A2A) {
        vk_render_pass_info_[render_pass_info_id_].dependencyCount = 2;
        vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies[0];
      }
      else {
        vk_render_pass_info_[render_pass_info_id_].dependencyCount = 6;
        vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies_shader_read[0];
      }
    }
    else {
      if (render_pass_enum_ == VKRenderPassTransition::A2A) {
        vk_render_pass_info_[render_pass_info_id_].dependencyCount = 1;
        vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies[0];
      }
      else {
        vk_render_pass_info_[render_pass_info_id_].dependencyCount = 3;
        vk_render_pass_info_[render_pass_info_id_].pDependencies = &dependencies_shader_read[0];
      }
    }
  }
}

void VKFrameBuffer::set_attachment_description(GPUTexture *tex,
                                               const VkAttachmentReference2 &attachment_reference,
                                               VkAttachmentDescription2 &attachment_description)
{
  /*
   * From the best layout for each Texture, we get the type of transition
   * that should be and determine the type of transition as this render pass.
   * So far, it can be expressed in a simple transition structure.
   */
  VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
  VKRenderPassTransition trans_ty = VKRenderPassTransition::ALL;
  switch (texture.render_pass_type_get()) {
    case eRenderpassType::ShaderBinding:
      trans_ty = VKRenderPassTransition::S2S;
      attachment_description.finalLayout = attachment_description.initialLayout =
          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      break;
    case eRenderpassType::Attachment:
      trans_ty = VKRenderPassTransition::A2A;
      BLI_assert(attachment_reference.layout != VK_IMAGE_LAYOUT_UNDEFINED);
      attachment_description.finalLayout = attachment_description.initialLayout =
          attachment_reference.layout;
      break;
    case eRenderpassType::Storage:
      trans_ty = VKRenderPassTransition::G2G;
      attachment_description.finalLayout = attachment_description.initialLayout =
          VK_IMAGE_LAYOUT_GENERAL;
      break;
    case eRenderpassType::Any:
      BLI_assert_unreachable();
  };
  if (render_pass_enum_ == VKRenderPassTransition::ALL) {
    render_pass_enum_ = trans_ty;
  }
  else {
    /* Is the transition structure unified by several attachments? */
    render_pass_enum_ = (trans_ty != render_pass_enum_) ? VKRenderPassTransition::MIX : trans_ty;
  }
  attachment_description.format = to_vk_format(texture.format_get());
}

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
    /* Set the current information on the `render_pass_info_id_` side and compare it with the
     * information cached in the reverse id. */
    if (type >= GPU_FB_COLOR_ATTACHMENT0) {
      BLI_assert(attachment_idx[render_pass_info_id_][type] == -1);

      if (vk_framebuffer_create_info_.attachmentCount > 0) {
        /* If the frame buffer information is not the default value, a color attachment change is
         * forced. */
        view_set = true;
      }
      /* Specify the order of reference as an index, starting with the description of each subpass.
       */
      attachment_reference =
          &attachment_references_[render_pass_info_id_]
                                 [subpass_[render_pass_info_id_].colorAttachmentCount];
      attachment_reference->aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      attachment_reference->layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
      /* The number of color attachments described in shaders is incremented here. */
      subpass_[render_pass_info_id_].colorAttachmentCount++;

      /* Whether to increment the number of attachments actually used. */
      if (tex) {
        attachment_reference->attachment =
            vk_render_pass_info_[render_pass_info_id_].attachmentCount++;
      }
      else {
        attachment_reference->attachment = VK_ATTACHMENT_UNUSED;
      }
      /* Take a cache to look up the order in the render pass information from the number of
          GPUAttachmentTypes. */
      attachment_idx[render_pass_info_id_][type] = attachment_reference->attachment;
      if (attachment_idx[render_pass_info_id_cache_][type] != attachment_reference->attachment) {
        /* we have been asked to change the number of attachments for this frame buffer.  */
        dirty_flag = true;
      }
    }
    else {
      /* Depth configuration should always be done last. */
      view_set = true;
      attachment_idx[render_pass_info_id_][type] = -1;
      attachment_reference = &depth_attachment_references_[render_pass_info_id_];
      if (tex) {
        /* Use depth-attachment. */
        if (attachment_idx[render_pass_info_id_][type] == VK_ATTACHMENT_UNUSED) {
          attachment_idx[render_pass_info_id_][type] =
              vk_render_pass_info_[render_pass_info_id_].attachmentCount++;
        }
        VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
        if (GPU_texture_has_stencil_format(tex)) {
          BLI_assert(ELEM(type, GPU_FB_DEPTH_STENCIL_ATTACHMENT));
        }
        else if (GPU_texture_has_depth_format(tex)) {
          BLI_assert(ELEM(type, GPU_FB_DEPTH_ATTACHMENT));
        }

        attachment_reference->aspectMask = vk_format_to_aspect(to_vk_format(texture.format_get()));
        attachment_reference->layout = vk_aspect_to_layout(attachment_reference->aspectMask);
        attachment_reference->attachment = attachment_idx[render_pass_info_id_][type];
        subpass_[render_pass_info_id_].pDepthStencilAttachment = attachment_reference;
        if (attachment_idx[render_pass_info_id_cache_][type] != attachment_reference->attachment) {
          dirty_flag = true;
        }
      }
      else {
        BLI_assert(subpass_[render_pass_info_id_].colorAttachmentCount > 0);
        if (attachment_idx[render_pass_info_id_][type] != VK_ATTACHMENT_UNUSED) {
          attachment_idx[render_pass_info_id_][type] = VK_ATTACHMENT_UNUSED;
          vk_render_pass_info_[render_pass_info_id_].attachmentCount--;
        }
        subpass_[render_pass_info_id_].pDepthStencilAttachment = nullptr;
        if (subpass_[render_pass_info_id_].pDepthStencilAttachment !=
            subpass_[render_pass_info_id_cache_].pDepthStencilAttachment)
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
      attachment_reference = &attachment_references_[render_pass_info_id_]
                                                    [attachment_idx[render_pass_info_id_][type]];
    }
    else {
      attachment_reference = &depth_attachment_references_[render_pass_info_id_];
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

    auto layer_range = layer_range_get(tex, atta_layer);
    dirty_view |= image_view_ensure(tex, atta_mip, layer_range, attachment_reference->attachment);

    VkAttachmentDescription2 &attachment_description =
        attachment_descriptions_[render_pass_info_id_][attachment_reference->attachment];
    if (config) {
      set_attachment_description(tex, *attachment_reference, attachment_description);
      VkAttachmentDescription2 &attachment_description_cache =
          attachment_descriptions_[render_pass_info_id_cache_][attachment_reference->attachment];
      if ((attachment_description.finalLayout != attachment_description_cache.finalLayout) ||
          (attachment_description.format != attachment_description_cache.format))
      {
        /* A change in the attachment format or transition structure is required. */
        dirty_flag = true;
      }
    }
    else {
      VkAttachmentDescription2 &attachment_description_cache =
          attachment_descriptions_[render_pass_info_id_cache_][attachment_reference->attachment];
      set_attachment_description(tex, *attachment_reference, attachment_description_cache);
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
        vk_render_pass_info_[render_pass_info_id_].attachmentCount;
    if ((vk_render_pass_info_[render_pass_info_id_].attachmentCount !=
         vk_render_pass_info_[render_pass_info_id_cache_].attachmentCount) ||
        (subpass_[render_pass_info_id_].colorAttachmentCount !=
         subpass_[render_pass_info_id_cache_].colorAttachmentCount))
    {
      /* The number of attachments is changed. */
      dirty_flag = true;
      dependency_set(subpass_[render_pass_info_id_].pDepthStencilAttachment != nullptr);
    }
    if (!dirty_flag) {
      /* No regeneration is required. */
      render_pass_info_id_ = render_pass_info_id_cache_;
      render_pass_info_id_cache_ = (render_pass_info_id_ + 1) % 2;
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
  dirty_render_pass_ |= dirty_flag;
}

void VKFrameBuffer::multiview_set()
{
  VkSubpassDependency2 *dependency = (render_pass_enum_ == VKRenderPassTransition::A2A) ?
                                         &dependencies[0] :
                                     (subpass_[render_pass_info_id_].colorAttachmentCount == 0) ?
                                         &dependencies_shader_read[4] :
                                         &dependencies_shader_read[1];
  dependency->dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
  VkSubpassDependency2 *dependency2 = nullptr;
  if (subpass_[render_pass_info_id_].colorAttachmentCount > 0 &&
      subpass_[render_pass_info_id_].pDepthStencilAttachment != nullptr)
  {
    dependency2 = (render_pass_enum_ == VKRenderPassTransition::A2A) ?
                      &dependencies[1] :
                      &dependencies_shader_read[4];
    dependency2->dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
  }
  vk_render_pass_info_[render_pass_info_id_].correlatedViewMaskCount = 0;
  vk_render_pass_info_[render_pass_info_id_].pCorrelatedViewMasks = VK_NULL_HANDLE;
  subpass_[render_pass_info_id_].viewMask = 0;
  const uint32_t correlation_mask = 0;

  /* TODO:Multi-view processing */
  if (false) {
    for (int i = 0; i < layers; i++) {
      subpass_[render_pass_info_id_].viewMask = (subpass_[render_pass_info_id_].viewMask |
                                                 (0b1 << i));
    }
    dependency->dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT;
    if (dependency2) {
      dependency2->dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT;
    }
    vk_render_pass_info_[render_pass_info_id_].correlatedViewMaskCount = 1;
    vk_render_pass_info_[render_pass_info_id_].pCorrelatedViewMasks = &correlation_mask;
  }

  if (subpass_[render_pass_info_id_].viewMask != subpass_[render_pass_info_id_cache_].viewMask) {
    subpass_[render_pass_info_id_cache_].viewMask = subpass_[render_pass_info_id_].viewMask;
    dirty_render_pass_ = true;
  }
}

int VKFrameBuffer::attchment_type_get(int view_index) const
{
  int i = 0;
  for (const auto &index : attachment_idx[render_pass_info_id_]) {
    if (index == view_index) {
      return i;
    }
    i++;
  }
  BLI_assert_unreachable();
  return -1;
};
/** \} */

/* -------------------------------------------------------------------- */
/** \name Update attachments
 * \{ */

void VKFrameBuffer::vk_render_pass_ensure()
{
  if (vk_framebuffer_create_info_.pAttachments == nullptr) {
    return;
  }
  if (dirty_attachments_) {
    dirty_attachments_8_ = default_attachments_;
    if (default_attachments_ == 0) {
      dirty_render_pass_ = true;
      render_pass_create();
      return;
    }
    dirty_attachments_ = false;
  };

  if (!dirty_attachments_8_) {
    return;
  }

  render_pass_create();
  framebuffer_ensure();
  dirty_attachments_8_ = 0;
}

void VKFrameBuffer::render_pass_create()
{
  VK_ALLOCATION_CALLBACKS
  multiview_set();
  if (!dirty_render_pass_) {
    return;
  }
  render_pass_free();
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateRenderPass2(device.device_get(),
                      &vk_render_pass_info_[render_pass_info_id_],
                      vk_allocation_callbacks,
                      &vk_render_pass_);
  vk_framebuffer_create_info_.renderPass = vk_render_pass_;
  dirty_attachments_8_ = 0;
  framebuffer_create();
  dirty_render_pass_ = false;
}

void VKFrameBuffer::render_pass_free()
{
  if (vk_render_pass_ == VK_NULL_HANDLE) {
    return;
  }

  VKDevice &device = VKBackend::get().device_get();
  if (device.is_initialized()) {
    device.discard_render_pass(vk_render_pass_);
  }
  vk_render_pass_ = VK_NULL_HANDLE;
  dirty_render_pass_ = true;
}

void VKFrameBuffer::framebuffer_create()
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
      if (subpass_[render_pass_info_id_].colorAttachmentCount > 0) {
        for (int j = 0; j < 8; j++) {
          if (subpass_[render_pass_info_id_].pColorAttachments[j].attachment ==
              VK_ATTACHMENT_UNUSED) {
            continue;
          }
          type = attchment_type_get(
              subpass_[render_pass_info_id_].pColorAttachments[i].attachment);
          const GPUAttachment &attachment = attachments_[type];
          reinterpret_cast<Texture *>(attachment.tex)->mip_size_get(attachment.mip, size);
          break;
        }
        if (size[0] == -1) {
          type = attchment_type_get(
              subpass_[render_pass_info_id_].pDepthStencilAttachment->attachment);
          const GPUAttachment &attachment = attachments_[type];
          reinterpret_cast<Texture *>(attachment.tex)->mip_size_get(attachment.mip, size);
        }
      }
    }
  }

  BLI_assert(size[0] > 0);
  size_set(size[0], size[1]);

  vk_framebuffer_create_info_.width = width_;
  vk_framebuffer_create_info_.height = height_;
  vk_framebuffer_create_info_.layers = 1;
  vk_framebuffer_create_info_.pAttachments = data.data();
  /* TODO:Multilayered rendering*/
  viewport_reset();
  scissor_reset();
  frame_buffer_free();
  VK_ALLOCATION_CALLBACKS
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateFramebuffer(device.device_get(),
                      &vk_framebuffer_create_info_,
                      vk_allocation_callbacks,
                      &vk_framebuffer_);
  debug::object_label(vk_framebuffer_, name_);
};

void VKFrameBuffer::frame_buffer_free()
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

void VKFrameBuffer::framebuffer_ensure()
{
  if (dirty_attachments_8_ == 0) {
    if (vk_render_pass_ == VK_NULL_HANDLE) {
      dirty_render_pass_ = true;
      render_pass_create();
    };
    return;
  }

  bool flush = false;
  int N = vk_render_pass_info_[render_pass_info_id_].attachmentCount;

  for (int i = 0; i < N; i++) {
    if ((dirty_attachments_8_ >> i) & 1) {
      int type = attchment_type_get(i);
      const GPUAttachment &attachment = attachments_[type];
      if (!attachment.tex) {
        flush = true;
        continue;
      }
      auto layer_range = layer_range_get(attachment.tex, attachment.layer);
      flush |= image_view_ensure(attachment.tex, attachment.mip, layer_range, i);
    }
  }
  dirty_attachments_8_ = 0;

  if (flush || vk_framebuffer_ == VK_NULL_HANDLE) {
    if (vk_render_pass_ == VK_NULL_HANDLE) {
      dirty_render_pass_ = true;
      render_pass_create();
    }
    else {
      framebuffer_create();
    }
  }
  return;
}

bool VKFrameBuffer::image_view_ensure(GPUTexture *tex,
                                      int mip,
                                      IndexRange &layer_range,
                                      int attachment_index)
{
  VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
  srgb_ = (texture.format_flag_get() & GPU_FORMAT_SRGB) != 0;

  bool use_srgb = srgb_ && enabled_srgb_;
  std::weak_ptr<VKImageView> image_view = texture.image_view_get(use_srgb, mip, layer_range);

  if (image_views_[attachment_index].expired() ||
      vk_image_view_equal(image_views_[attachment_index], image_view))
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
  return subpass_[render_pass_info_id_].colorAttachmentCount;
}

VkRenderPass VKFrameBuffer::vk_render_pass_get()
{
  framebuffer_ensure();
  return vk_render_pass_;
}

void VKFrameBuffer::set_dirty_attchments(VKTexture *texture)
{
  for (int i = 0; i < subpass_[render_pass_info_id_].colorAttachmentCount; i++) {
    if (subpass_[render_pass_info_id_].pColorAttachments[i].attachment == VK_ATTACHMENT_UNUSED) {
      continue;
    }
    int type = attchment_type_get(subpass_[render_pass_info_id_].pColorAttachments[i].attachment);
    const GPUAttachment &attachment = attachments_[type];
    if (attachment.tex) {
      VKTexture *texture_ = reinterpret_cast<VKTexture *>(attachment.tex);
      if (texture == texture_) {
        dirty_attachments_8_ |=
            (1 << subpass_[render_pass_info_id_].pColorAttachments[i].attachment);
        return;
      }
    }
  }
  if (subpass_[render_pass_info_id_].pDepthStencilAttachment) {
    int type = attchment_type_get(
        subpass_[render_pass_info_id_].pDepthStencilAttachment->attachment);
    VKTexture *texture_ = reinterpret_cast<VKTexture *>(attachments_[type].tex);
    if (texture_ == texture) {
      dirty_attachments_8_ |=
          (1 << subpass_[render_pass_info_id_].pDepthStencilAttachment->attachment);
    }
  }
}

/** \} */

}  // namespace blender::gpu
