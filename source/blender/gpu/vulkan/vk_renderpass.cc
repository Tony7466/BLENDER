/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_renderpass.hh"
#include "vk_backend.hh"
#include "vk_framebuffer.hh"
#include "vk_memory.hh"
#include "vk_subpass.hh"
#include "vk_texture.hh"

namespace blender::gpu {
namespace vk_subpass {
/* Dependencies when transitioning from render target to render target. There are no transitions.*/
enum class DependencyType {
  BASIC_COLOR,
  BASIC_DEPTH,
  SHADER_READ_BEGIN,
  SHADER_READ_ATTACH,
  SHADER_READ_END,
  SHADER_READ_DEPATH_BEGIN,
  SHADER_READ_DEPATH_ATTACH,
  SHADER_READ_DEPATH_END,
  MULTIVIEW_BEGIN,
  MULTIVIEW_END,
  MULTIVIEW_SHADER_READ_BEGIN,
  MULTIVIEW_SHADER_READ_ATTACH,
  MULTIVIEW_SHADER_READ_END,
  MULTIVIEW_SHADER_READ_DEPATH_BEGIN,
  MULTIVIEW_SHADER_READ_DEPATH_ATTACH,
  MULTIVIEW_SHADER_READ_DEPATH_END,
  DEPENDENCY_TYPE_ALL
};
/**
 * Most passes have only one subpass, so the dependencies are static.
 * No need for constructs or copies.
 */
const static VkSubpassDependency2 dependencies[static_cast<uint64_t>(
    DependencyType::DEPENDENCY_TYPE_ALL)] = {
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
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     0,
     0,
     VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_ACCESS_MEMORY_READ_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
     VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT,
     0},
    {VK_STRUCTURE_TYPE_SUBPASS_DEPENDENCY_2,
     VK_NULL_HANDLE,
     VK_SUBPASS_EXTERNAL,
     0,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
     VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
     VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT,
     0},
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
     0,
     0,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_READ_BIT,
     VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT,
     VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT,
     0},
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
     VK_DEPENDENCY_BY_REGION_BIT | VK_DEPENDENCY_VIEW_LOCAL_BIT,
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
     0},
};
}  // namespace vk_subpass

VKRenderPass::VKRenderPass()
{
#ifdef VK_PIPELINE_REFACTOR
  VKDevice &device = VKBackend::get().device_get();
  pipeline_free(device);
#endif
};

bool VKRenderPass::ensure()
{
  if (vk_render_pass_ == VK_NULL_HANDLE || dirty_) {
    dirty_ = true;
    create();
    return true;
  };
  return false;
}

void VKRenderPass::create()
{
  if (!dirty_) {
    return;
  }
  free();
  VK_ALLOCATION_CALLBACKS
  is_clear_pass_ = false;
  for (int i = 0; i < vk_create_info_[info_id_].attachmentCount; i++) {
    if (!ELEM(vk_create_info_[info_id_].pAttachments[i].loadOp, VK_ATTACHMENT_LOAD_OP_LOAD)) {
      is_clear_pass_ = true;
      break;
    }
  }
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateRenderPass2(
      device.device_get(), &vk_create_info_[info_id_], vk_allocation_callbacks, &vk_render_pass_);
  dirty_ = false;
}

void VKRenderPass::free()
{
  VK_ALLOCATION_CALLBACKS
  VKDevice &device = VKBackend::get().device_get();
#ifdef VK_PIPELINE_REFACTOR
  pipeline_free(device);
#endif
  if (vk_render_pass_ == VK_NULL_HANDLE) {
    return;
  }

  if (device.is_initialized()) {
    VKContext *context = VKContext::get();
    if (context == nullptr) {
      vkDestroyRenderPass(device.device_get(), vk_render_pass_, vk_allocation_callbacks);
    }
    else {
      context->discard_render_pass(vk_render_pass_);
    }
  }

  dirty_ = true;
  vk_render_pass_ = VK_NULL_HANDLE;
}

void VKRenderPass::cache_init()
{
  info_id_ = info_id_counter();
  multiview_layers_ = 1;

  /* Number of ShaderOutputs.*/
  subpass_[info_id_].colorAttachmentCount = 0;
  subpass_[info_id_].pColorAttachments = attachments_.references_[info_id_].data();
  subpass_[info_id_].pDepthStencilAttachment = nullptr;

  /* Number of non-null ImageViews. */
  vk_create_info_[info_id_].attachmentCount = 0;
  vk_create_info_[info_id_].pAttachments = attachments_.descriptions_[info_id_].data();
  vk_create_info_[info_id_].subpassCount = 1;
  vk_create_info_[info_id_].pSubpasses = &subpass_[info_id_];
  render_pass_enum_ = eRenderpassType::Any;
  subpass_multi_attachments.clear();
  for (auto &i : attachments_.idx_[info_id_]) {
    i = VK_ATTACHMENT_EMPTY;
  }
  imageless_ = false;
};

void VKRenderPass::dependency_static_set(bool use_depth)
{
  if (render_pass_enum_ == eRenderpassType::Any) {
    return;
  }

  BLI_assert(!ELEM(render_pass_enum_, eRenderpassType::Any));

  bool depth_only = (subpass_[info_id_].colorAttachmentCount == 0);

  switch (render_pass_enum_) {
    case eRenderpassType::Attachment:
      switch (depth_only) {
        case true:
          vk_create_info_[info_id_].dependencyCount = 1;
          vk_create_info_[info_id_].pDependencies =
              &vk_subpass::dependencies[static_cast<uint64_t>(
                  vk_subpass::DependencyType::BASIC_DEPTH)];
          break;
        case false:
          vk_create_info_[info_id_].pDependencies =
              &vk_subpass::dependencies[static_cast<uint64_t>(
                  vk_subpass::DependencyType::BASIC_COLOR)];
          switch (use_depth) {
            case true:
              vk_create_info_[info_id_].dependencyCount = 2;
              break;
            case false:
              vk_create_info_[info_id_].dependencyCount = 1;
              break;
          }
          break;
      }
      break;
    case eRenderpassType::Storage:
    case eRenderpassType::ShaderBinding:
      switch (depth_only) {
        case true:
          vk_create_info_[info_id_].dependencyCount = 3;
          vk_create_info_[info_id_].pDependencies =
              &vk_subpass::dependencies[static_cast<uint64_t>(
                  vk_subpass::DependencyType::SHADER_READ_DEPATH_BEGIN)];
          break;
        case false:
          vk_create_info_[info_id_].pDependencies =
              &vk_subpass::dependencies[static_cast<uint64_t>(
                  vk_subpass::DependencyType::SHADER_READ_BEGIN)];
          switch (use_depth) {
            case true:
              vk_create_info_[info_id_].dependencyCount = 6;
              break;
            case false:
              vk_create_info_[info_id_].dependencyCount = 3;
              break;
          }
          break;
      }
      break;
    case eRenderpassType::Mix: {
      BLI_assert(depth_only == false);
      vk_create_info_[info_id_].pDependencies = &vk_subpass::dependencies[static_cast<uint64_t>(
          vk_subpass::DependencyType::BASIC_COLOR)];
      vk_create_info_[info_id_].dependencyCount = 8;
      break;
    }
    default:
      BLI_assert_unreachable();
  }
}

VkSubpassDependency2 VKRenderPass::dependency_get(int srcpass,
                                                  int dstpass,
                                                  SubpassTransitionPattern transition_pattern,
                                                  VkImageLayout dst_layout)
{
  VKSubpassDependency subpass_dep;
  switch (transition_pattern) {
    case SubpassTransitionPattern::EXTERNAL_TO_COLOR:
      return subpass_dep.external_to_color(dstpass, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::EXTERNAL_TO_DEPTH:
      return subpass_dep.external_to_depth(dstpass,
                                           VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::EXTERNAL_SHADER_READ_WRITE_TO_COLOR:
      return subpass_dep.external_to_color(dstpass, dst_layout);
    case SubpassTransitionPattern::EXTERNAL_SHADER_READ_WRITE_TO_DEPTH:
      return subpass_dep.external_to_depth(dstpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::COLOR_TO_INPUT:
      return subpass_dep.color_to_input(srcpass, dstpass);
    case SubpassTransitionPattern::COLOR_TO_EXTERNAL_SHADER_READ_WRITE:
      return subpass_dep.color_to_external(srcpass, dst_layout);
    case SubpassTransitionPattern::COLOR_TO_EXTERNAL:
      return subpass_dep.color_to_external(srcpass, dst_layout);
    case SubpassTransitionPattern::INPUT_TO_EXTERNAL_COLOR:
      return subpass_dep.input_to_external(srcpass, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::DEPTH_TO_EXTERNAL_SHADER_READ_WRITE:
      return subpass_dep.depth_to_external(srcpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::INPUT_TO_EXTERNAL_SHADER_READ_WRITE:
      return subpass_dep.input_to_external(srcpass, dst_layout);
    default:
      BLI_assert_unreachable();
  }
  return subpass_dep;
}

static int scan_pre_write_pass(SubpassBits &subpass, int read_pass)
{
  for (int i = read_pass - 1; i >= 0; i--) {
    if (subpass.bits.write[i] > 0) {
      return i;
    }
  }
  BLI_assert_unreachable();
  return -1;
}

static bool prepass_loaded(VkSubpassDescription2 &subpass, int view_order, bool color)
{
  if (subpass.colorAttachmentCount > 0) {
    if (color) {
      for (int i = 0; i < subpass.colorAttachmentCount; i++) {
        auto ref = subpass.pColorAttachments[i];
        if (ref.attachment == view_order) {
          return true;
        }
      }
    }
    else {
      if (subpass.pDepthStencilAttachment) {
        if (subpass.pDepthStencilAttachment->attachment == view_order) {
          return true;
        }
      }
    }
  }
  return false;
}

void VKRenderPass::ensure_subpass_multiple(VKFrameBuffer &frame_buffer)
{
  /**
   * `uint16` represents the types of dependency.
   * i.g. srcSubpass= 15-10 bits, dstSubpass= 9-4 bits, transitionPattern= 3-0 bits.
   */
  vk_create_info_[info_id_].dependencyCount = 0;
  vk_create_info_[info_id_].pDependencies = nullptr;
  Vector<VkSubpassDependency2> dependencies;
  Set<uint16_t> cache_dependency;
  /* Dependencies must be a set, so we use bit flags to avoid duplicating dependencies. */
  auto check_dependency = [&cache_dependency](uint32_t src_subpass,
                                              uint32_t dst_subpass,
                                              SubpassTransitionPattern pattern) -> bool {
    uint16_t flag = (uint16_t)((src_subpass & 0b111111) << 10) +
                    (uint16_t)((dst_subpass & 0b111111) << 4) +
                    (uint16_t)(((uint8_t)pattern) & 0b1111);
    if (cache_dependency.contains(flag)) {
      return false;
    };
    cache_dependency.add(flag);
    return true;
  };
  int N = vk_create_info_[info_id_].attachmentCount;
  vk_create_info_[info_id_].subpassCount = 0;
  /* The main configuration must already be done. The resource actually exists. */
  BLI_assert(N > 0);
  /* Prepare the temporary descriptions required to generate VulkanObject. */
  VkSubpassDescription2 subpasses[GPU_TEX_MAX_SUBPASS];
  Vector<VkAttachmentReference2> write_references_all[GPU_TEX_MAX_SUBPASS];
  Vector<VkAttachmentReference2> input_references_all[GPU_TEX_MAX_SUBPASS];
  /* The last subpass number where depth is written */
  int dst_pass_depth_final = -1;

  VkImageLayout dst_depth_layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL_KHR;
  auto layout2transition = [](VkImageLayout layout) -> SubpassTransitionPattern {
    switch (layout) {
      case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
      case VK_IMAGE_LAYOUT_GENERAL:
        return SubpassTransitionPattern::EXTERNAL_SHADER_READ_WRITE_TO_COLOR;
        break;
      case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
        return SubpassTransitionPattern::EXTERNAL_TO_COLOR;
      default:
        break;
    }
    BLI_assert_unreachable();
    return SubpassTransitionPattern::EXTERNAL_TO_COLOR;
  };
  /* Start parsing from the first pass. */
  for (int pass = 0; pass < GPU_TEX_MAX_SUBPASS; pass++) {
    bool subpass_add = false;
    subpass_input_orders_[pass].clear();
    auto &desc = subpasses[pass] = vk_subpass::descriptions_default;
    Vector<VkAttachmentReference2> &references = write_references_all[pass];
    Vector<VkAttachmentReference2> &ireferences = input_references_all[pass];
    references.resize(N);
    ireferences.clear();
    int unused_color = 0;
    int unused_depth = 0;
    /**
     * Scan this pass's Blender attachments.
     * Since the Texture has multiple subpass Bit states,
     * we take the Texture from the Attachment and identify the dependencies used in this pass from
     * its bits.
     */
    for (int view_order = 0; view_order < N; view_order++) {
      int type = attachments_.type_get(view_order, info_id_);
      GPUTexture *attachment_tex = nullptr;
      int type_index = static_cast<int>(type) - static_cast<int>(GPU_FB_COLOR_ATTACHMENT0);
      /* final-layout gives the initial state. */
      const VkImageLayout final_layout =
          attachments_.descriptions_[info_id_][view_order].finalLayout;

      if (type_index < 0) {
        attachment_tex = frame_buffer.depth_tex();
      }
      else {
        attachment_tex = frame_buffer.color_tex(type_index);
      }
      BLI_assert(attachment_tex);
      Texture *tex = reinterpret_cast<Texture *>(attachment_tex);
      auto subpass_flag = tex->subpass_bits_get();
      /* Is readable-state required ? */
      if ((subpass_flag.bits.read_pass >> pass) & 0b1) {
        /* This state is never an initial transition. */
        BLI_assert(pass > 0);
        desc.inputAttachmentCount++;
        VkAttachmentReference2 ireference = vk_attachment::references_default;
        ireference.attachment = view_order;
        ireference.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        ireference.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        ireferences.append(ireference);
        subpass_input_orders_[pass].append(type_index);
        int src_pass = scan_pre_write_pass(subpass_flag, pass);
        if (check_dependency(src_pass, pass, SubpassTransitionPattern::COLOR_TO_INPUT)) {
          dependencies.append(dependency_get(
              src_pass, pass, SubpassTransitionPattern::COLOR_TO_INPUT, final_layout));
        }
        unused_color--;
      }
      uint8_t write_attachment = subpass_flag.bits.write[pass];

      /* Is this an Attachment ? */
      if (type_index < 0) {
        if (write_attachment != 1) {
          unused_depth++;
          continue;
        }
        /* It is possible to transition depth to input, but it is a TODO. */
        desc.pDepthStencilAttachment = attachments_.reference_get(type_index, info_id_);

        if (dst_pass_depth_final >= 0) {
          dst_pass_depth_final = pass;
          continue;
        }
        subpass_add = true;
        if ((dst_pass_depth_final != -2) &&
            final_layout == VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_STENCIL_READ_ONLY_OPTIMAL)
        {
          dst_pass_depth_final = -2;
          dst_depth_layout = final_layout;
          if (!(vk_create_info_[info_id_].pAttachments[view_order].loadOp ==
                VK_ATTACHMENT_LOAD_OP_LOAD))
          {
            dependencies.append(dependency_get(VK_SUBPASS_EXTERNAL,
                                               pass,
                                               SubpassTransitionPattern::EXTERNAL_TO_DEPTH,
                                               final_layout));
          }
          dependencies.append(dependency_get(VK_SUBPASS_EXTERNAL,
                                             pass,
                                             SubpassTransitionPattern::EXTERNAL_TO_DEPTH,
                                             final_layout));
        }
        else {
          /* Scan shader-order. */
          /* The references are derived from the references generated in the main
           * configuration. */
          dependencies.append(
              dependency_get(VK_SUBPASS_EXTERNAL,
                             pass,
                             SubpassTransitionPattern::EXTERNAL_SHADER_READ_WRITE_TO_DEPTH,
                             final_layout));
          dst_pass_depth_final = pass;
          dst_depth_layout = final_layout;
        }
      }
      else {
        if (write_attachment >> view_order & 0b1) {

          VkAttachmentReference2 copy_reference = *attachments_.reference_get(type_index,
                                                                              info_id_);
          desc.colorAttachmentCount++;
          references[view_order] = copy_reference;
          BLI_assert(references[view_order].attachment == view_order);
          bool loaded = false;
          for (int ppass = pass - 1; ppass >= 0; ppass--) {
            loaded |= prepass_loaded(subpasses[ppass], view_order, true);
          }
          if (!loaded) {
            if (!(vk_create_info_[info_id_].pAttachments[view_order].loadOp ==
                  VK_ATTACHMENT_LOAD_OP_LOAD))
            {
              if (check_dependency(
                      VK_SUBPASS_EXTERNAL, pass, SubpassTransitionPattern::EXTERNAL_TO_COLOR))
              {
                dependencies.append(dependency_get(VK_SUBPASS_EXTERNAL,
                                                   pass,
                                                   SubpassTransitionPattern::EXTERNAL_TO_COLOR,
                                                   final_layout));
              }
            }
            else {
              SubpassTransitionPattern t_pattern = layout2transition(final_layout);
              if (check_dependency(VK_SUBPASS_EXTERNAL, pass, t_pattern)) {
                dependencies.append(
                    dependency_get(VK_SUBPASS_EXTERNAL, pass, t_pattern, final_layout));
              }
            }
          }
        }
        else {
          desc.colorAttachmentCount++;
          references[view_order] = *attachments_.reference_get(type_index, info_id_);
          references[view_order].attachment = VK_ATTACHMENT_UNUSED;
          unused_color++;
        };
      }
    }
    if ((unused_color + unused_depth) < N) {
      desc.pColorAttachments = references.data();
      if (desc.inputAttachmentCount > 0) {
        desc.pInputAttachments = ireferences.data();
      }

      vk_create_info_[info_id_].subpassCount++;
      subpass_multi_attachments.append(subpasses[pass].colorAttachmentCount);
    }
    else {
      desc.colorAttachmentCount = 0;
    }
  }
  if (dst_pass_depth_final > -1) {
    dependencies.append(
        dependency_get(dst_pass_depth_final,
                       VK_SUBPASS_EXTERNAL,
                       SubpassTransitionPattern::DEPTH_TO_EXTERNAL_SHADER_READ_WRITE,
                       dst_depth_layout));
  }
  {
    const VkAttachmentDescription2 *atta = vk_create_info_[info_id_].pAttachments;

    Vector<bool> final_transition_valid;
    final_transition_valid.resize(N, false);
    for (int pass = GPU_TEX_MAX_SUBPASS - 1; pass >= 0; pass--) {
      auto subpass = subpasses[pass];
      if (subpass.colorAttachmentCount == 0) {
        continue;
      }
      int color_num = (subpass.pDepthStencilAttachment == nullptr) ? N : N - 1;
      BLI_assert(subpass.colorAttachmentCount == color_num);
      for (int i = 0; i < color_num; i++) {
        if (final_transition_valid[i]) {
          continue;
        }
        auto ref = subpass.pColorAttachments[i];
        if (ref.attachment != VK_ATTACHMENT_UNUSED) {
          BLI_assert(ref.attachment == i);
          auto desc = atta[ref.attachment];
          SubpassTransitionPattern transition;
          switch (desc.finalLayout) {
            case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
              transition = SubpassTransitionPattern::COLOR_TO_EXTERNAL;
              break;
            case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
            case VK_IMAGE_LAYOUT_GENERAL:
              transition = SubpassTransitionPattern::COLOR_TO_EXTERNAL_SHADER_READ_WRITE;
              break;
            default:
              break;
          }
          if (check_dependency(pass, VK_SUBPASS_EXTERNAL, transition)) {
            dependencies.append(
                dependency_get(pass, VK_SUBPASS_EXTERNAL, transition, desc.finalLayout));
          };
          final_transition_valid[i] = true;
          continue;
        }
        if (subpass.inputAttachmentCount <= i) {
          continue;
        }
        auto iref = subpass.pInputAttachments[i];
        if (iref.attachment != VK_ATTACHMENT_UNUSED) {

          auto desc = atta[iref.attachment];
          SubpassTransitionPattern transition = SubpassTransitionPattern::INPUT_TO_EXTERNAL_COLOR;
          if (transition != SubpassTransitionPattern::SUBPASS_TRANSITION_PATTERN_ALL) {
            if (check_dependency(pass, VK_SUBPASS_EXTERNAL, transition)) {
              dependencies.append(
                  dependency_get(pass, VK_SUBPASS_EXTERNAL, transition, desc.finalLayout));
            };

            final_transition_valid[i] = true;
            continue;
          }
        }
      }
    }
    VKSubpassDependency subpass_dep;
    for (int i = 1; i < vk_create_info_[info_id_].subpassCount - 1; i++) {
      dependencies.append(subpass_dep.depth_to_depth(i, i + 1));
      dependencies.append(subpass_dep.write_to_read(i, i + 1));
      dependencies.append(subpass_dep.write_to_read2(i, i + 1));
    }
  }
  vk_create_info_[info_id_].dependencyCount = dependencies.size();
  vk_create_info_[info_id_].pDependencies = dependencies.data();
  vk_create_info_[info_id_].pSubpasses = subpasses;
  dirty_ = true;
  create();
  for (int view_order = 0; view_order < N; view_order++) {
    int type = attachments_.type_get(view_order, info_id_);
    int type_index = static_cast<int>(type) - static_cast<int>(GPU_FB_COLOR_ATTACHMENT0);
    Texture *attachment_tex = nullptr;
    if (type_index < 0) {
      attachment_tex = reinterpret_cast<Texture *>(frame_buffer.depth_tex());
    }
    else {
      attachment_tex = reinterpret_cast<Texture *>(frame_buffer.color_tex(type_index));
    }
    attachment_tex->subpass_bits_clear();
  }
}

void VKRenderPass::imageless_pass_set()
{
  auto &info = vk_create_info_[info_id_];
  info.attachmentCount = 0;
  info.pAttachments = &vk_attachment::descriptions_default;
  info.correlatedViewMaskCount = 0;
  info.dependencyCount = 0;
  info.flags = 0;
  info.pCorrelatedViewMasks = nullptr;
  info.pDependencies = nullptr;
  info.pNext = nullptr;
  subpass_[info_id_].colorAttachmentCount = 0;
  subpass_[info_id_].pColorAttachments = nullptr;
  subpass_[info_id_].pDepthStencilAttachment = nullptr;
  info.pSubpasses = &subpass_[info_id_];
  info.subpassCount = 1;
  imageless_ = true;
}

void VKRenderPass::multiview_set()
{
  /* TODO::Multiview Config */
  if (subpass_[info_id_].viewMask != subpass_[info_id_counter()].viewMask) {
    subpass_[info_id_counter()].viewMask = subpass_[info_id_].viewMask;
    dirty_ = true;
  }
}
#ifdef VK_PIPELINE_REFACTOR
void VKRenderPass::pipeline_free(VKDevice &device)
{
  VK_ALLOCATION_CALLBACKS
  for (int i = 0; i < GPU_PRIM_NONE; i++) {
    for (int j = 0; j < pipeline_cache_nums[i]; j++) {
      vkDestroyPipeline(device.device_get(), vk_pipelines[i][j], vk_allocation_callbacks);
    }
  }
  memset(vk_pipelines, 0, sizeof(VkPipeline) * GPU_PRIM_NONE * VK_PIPELINE_CACHE_MAX);
  memset(pipeline_cache_by_prims, 0, sizeof(uint64_t) * GPU_PRIM_NONE * VK_PIPELINE_CACHE_MAX);
  memset(pipeline_cache_nums, 0, sizeof(uint32_t) * GPU_PRIM_NONE);
}

VkPipeline VKRenderPass::has_pipeline_cache(const GPUPrimType prim_type,
                                            const GPUState state,
                                            VkShaderModule v_module,
                                            VkShaderModule g_module,
                                            VkShaderModule f_module,
                                            const VkPipelineVertexInputStateCreateInfo &info,
                                            int type)
{
  BLI_assert(type == 3);
  std::string vao_info = "";
  vao_info += encode_struct(info.pVertexBindingDescriptions, info.vertexBindingDescriptionCount);
  vao_info += encode_struct(info.pVertexAttributeDescriptions,
                            info.vertexAttributeDescriptionCount);
  if (info.pNext != VK_NULL_HANDLE) {
    auto *dev_info = reinterpret_cast<VkPipelineVertexInputDivisorStateCreateInfoEXT *>(
        const_cast<void *>(info.pNext));
    vao_info += encode_struct(dev_info->pVertexBindingDivisors,
                              dev_info->vertexBindingDivisorCount);
  }
  for (int i = 0; i < pipeline_cache_nums[prim_type]; i++) {
    if ((pipeline_cache_by_prims[prim_type][i].state == state.data) &&
        (pipeline_cache_by_prims[prim_type][i].v_module == (uint64_t)v_module) &&
        (pipeline_cache_by_prims[prim_type][i].g_module == (uint64_t)g_module) &&
        (pipeline_cache_by_prims[prim_type][i].f_module == (uint64_t)f_module) &&
        (vao_info == pipeline_cache_by_prims[prim_type][i].vao_info))
      return vk_pipelines[prim_type][i];
  }
  return VK_NULL_HANDLE;
};

VkPipeline VKRenderPass::has_pipeline_cache(const GPUPrimType prim_type,
                                            const GPUState state,
                                            VkShaderModule v_module,
                                            VkShaderModule g_module,
                                            VkShaderModule f_module,
                                            int type)
{
  BLI_assert(type == 2);
  for (int i = 0; i < pipeline_cache_nums[prim_type]; i++) {
    if ((pipeline_cache_by_prims[prim_type][i].state == state.data) &&
        (pipeline_cache_by_prims[prim_type][i].v_module == (uint64_t)v_module) &&
        (pipeline_cache_by_prims[prim_type][i].g_module == (uint64_t)g_module) &&
        (pipeline_cache_by_prims[prim_type][i].f_module == (uint64_t)f_module))
      return vk_pipelines[prim_type][i];
  }
  return VK_NULL_HANDLE;
};

VkPipeline VKRenderPass::has_pipeline_cache(const GPUPrimType prim_type,
                                            const GPUState state,
                                            int type)
{
  BLI_assert(type == 1);
  for (int i = 0; i < pipeline_cache_nums[prim_type]; i++) {
    if (pipeline_cache_by_prims[prim_type][i].state == state.data) {
      return vk_pipelines[prim_type][i];
    }
  }
  return VK_NULL_HANDLE;
};

void VKRenderPass::set_pipeline(VkPipeline pipeline,
                                const GPUPrimType prim_type,
                                const GPUState state,
                                int type)
{
  BLI_assert(type == 1);
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].state = state.data;
  vk_pipelines[prim_type][pipeline_cache_nums[prim_type]] = pipeline;
  pipeline_cache_nums[prim_type]++;
  BLI_assert(pipeline_cache_nums[prim_type] < VK_PIPELINE_CACHE_MAX);
}

void VKRenderPass::set_pipeline(VkPipeline pipeline,
                                const GPUPrimType prim_type,
                                VkShaderModule v_module,
                                VkShaderModule g_module,
                                VkShaderModule f_module,
                                const GPUState state,
                                int type)
{
  BLI_assert(type == 2);
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].state = state.data;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].v_module = (uint64_t)v_module;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].g_module = (uint64_t)g_module;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].f_module = (uint64_t)f_module;
  vk_pipelines[prim_type][pipeline_cache_nums[prim_type]] = pipeline;
  pipeline_cache_nums[prim_type]++;
  BLI_assert(pipeline_cache_nums[prim_type] < VK_PIPELINE_CACHE_MAX);
}

void VKRenderPass::set_pipeline(VkPipeline pipeline,
                                const GPUPrimType prim_type,
                                VkShaderModule v_module,
                                VkShaderModule g_module,
                                VkShaderModule f_module,
                                const GPUState state,
                                const VkPipelineVertexInputStateCreateInfo &info,
                                int type)
{
  BLI_assert(type == 3);
  std::string vao_info = "";
  vao_info += encode_struct(info.pVertexBindingDescriptions, info.vertexBindingDescriptionCount);
  vao_info += encode_struct(info.pVertexAttributeDescriptions,
                            info.vertexAttributeDescriptionCount);
  if (info.pNext != VK_NULL_HANDLE) {
    auto *dev_info = reinterpret_cast<VkPipelineVertexInputDivisorStateCreateInfoEXT *>(
        const_cast<void *>(info.pNext));
    vao_info += encode_struct(dev_info->pVertexBindingDivisors,
                              dev_info->vertexBindingDivisorCount);
  }
  vao_info += '\0';
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].state = state.data;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].v_module = (uint64_t)v_module;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].g_module = (uint64_t)g_module;
  pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].f_module = (uint64_t)f_module;
  memcpy(pipeline_cache_by_prims[prim_type][pipeline_cache_nums[prim_type]].vao_info,
         vao_info.data(),
         vao_info.size());
  vk_pipelines[prim_type][pipeline_cache_nums[prim_type]] = pipeline;
  pipeline_cache_nums[prim_type]++;
  BLI_assert(pipeline_cache_nums[prim_type] < VK_PIPELINE_CACHE_MAX);
}
#endif
}  // namespace blender::gpu
