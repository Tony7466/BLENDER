/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_renderpass.hh"
#include "vk_backend.hh"
#include "vk_framebuffer.hh"
#include "vk_subpass.hh"
#include "vk_memory.hh"
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
  const VKDevice &device = VKBackend::get().device_get();
  vkCreateRenderPass2(
      device.device_get(), &vk_create_info_[info_id_], vk_allocation_callbacks, &vk_render_pass_);
  dirty_ = false;
}

void VKRenderPass::free()
{
  VK_ALLOCATION_CALLBACKS
  if (vk_render_pass_ == VK_NULL_HANDLE) {
    return;
  }
  VKDevice &device = VKBackend::get().device_get();
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
  dirty_ = false;
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
};

void VKRenderPass::dependency_static_set(bool use_depth)
{
  if (render_pass_enum_ == eRenderpassType::Any) {
    return;
  }

  BLI_assert(ELEM(render_pass_enum_, eRenderpassType::Attachment, eRenderpassType::ShaderBinding));

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
    default:
      BLI_assert_unreachable();
  }
}
VkSubpassDependency2 VKRenderPass::dependency_get(int srcpass,
                                                  int dstpass,
                                                  SubpassTransitionPattern transition_pattern)
{
  VKSubpassDependency subpass_dep;
  switch (transition_pattern) {
    case SubpassTransitionPattern::EXTERNAL_TO_COLOR:
      return subpass_dep.external_to_color(dstpass, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::EXTERNAL_TO_DEPTH:
      return subpass_dep.external_to_depth(dstpass,
                                           VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::EXTERNAL_SHADER_READ_TO_COLOR:
      return subpass_dep.external_to_color(dstpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::EXTERNAL_SHADER_WRITE_TO_COLOR:
      return subpass_dep.external_to_color(dstpass, VK_IMAGE_LAYOUT_GENERAL);
    case SubpassTransitionPattern::EXTERNAL_SHADER_READ_TO_DEPTH:
      return subpass_dep.external_to_depth(dstpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::COLOR_TO_INPUT:
      return subpass_dep.color_to_input(srcpass, dstpass);
    case SubpassTransitionPattern::COLOR_TO_EXTERNAL_SHADER_READ:
      return subpass_dep.color_to_external(srcpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::COLOR_TO_EXTERNAL_SHADER_WRITE:
      return subpass_dep.color_to_external(srcpass, VK_IMAGE_LAYOUT_GENERAL);
    case SubpassTransitionPattern::INPUT_TO_EXTERNAL_COLOR:
      return subpass_dep.input_to_external(srcpass, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
    case SubpassTransitionPattern::INPUT_TO_EXTERNAL_SHADER_READ:
      return subpass_dep.input_to_external(srcpass, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    case SubpassTransitionPattern::INPUT_TO_EXTERNAL_SHADER_WRITE:
      return subpass_dep.input_to_external(srcpass, VK_IMAGE_LAYOUT_GENERAL);
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
  SubpassTransitionPattern color_transition_next[GPU_TEX_MAX_SUBPASS];
  SubpassTransitionPattern color_read_transition_next[GPU_TEX_MAX_SUBPASS];

  /* Start parsing from the first pass. */
  for (int pass = 0; pass < GPU_TEX_MAX_SUBPASS; pass++) {
    color_transition_next[pass] = SubpassTransitionPattern::SUBPASS_TRANSITION_PATTERN_ALL;
    color_read_transition_next[pass] = SubpassTransitionPattern::SUBPASS_TRANSITION_PATTERN_ALL;
    bool subpass_add = false;
    subpass_input_orders_[pass].clear();
    auto &desc = subpasses[pass] = vk_subpass::descriptions_default;
    Vector<VkAttachmentReference2> &references = write_references_all[pass];
    Vector<VkAttachmentReference2> &ireferences = input_references_all[pass];
    references.clear();
    ireferences.clear();
    /**
     * Scan this pass's Blender attachments.
     * Since the Texture has multiple subpass Bit states,
     * we take the Texture from the Attachment and identify the dependencies used in this pass from
     * its bits.
     */
    for (int view_order = 0; view_order < N; view_order++) {
      int type = attachments_.type_get(view_order, info_id_);
      /* final-layout gives the initial state. */
      const VkImageLayout final_layout =
          attachments_.descriptions_[info_id_][view_order].finalLayout;
      BLI_assert(attachments_.descriptions_[info_id_][view_order].initialLayout == final_layout);
      GPUTexture *attachment_tex = nullptr;
      int type_index = static_cast<int>(type) - static_cast<int>(GPU_FB_COLOR_ATTACHMENT0);
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
          dependencies.append(
              dependency_get(src_pass, pass, SubpassTransitionPattern::COLOR_TO_INPUT));
        }
        color_read_transition_next[pass] =
            (final_layout == VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL) ?
                SubpassTransitionPattern::INPUT_TO_EXTERNAL_COLOR :
            (final_layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) ?
                SubpassTransitionPattern::INPUT_TO_EXTERNAL_SHADER_READ :
                SubpassTransitionPattern::INPUT_TO_EXTERNAL_SHADER_WRITE;
        /* It is never readable and write-attachment. */
        continue;
      }
      uint8_t write_attachment = subpass_flag.bits.write[pass];
      /* Is this an Attachment ? */
      if (write_attachment > 0) {
        /* Scan shader-order. */
        for (int shader_order = 0; shader_order < 8; shader_order++) {
          if ((write_attachment >> shader_order) & 0b1) {
            if (type_index < 0 && (dst_pass_depth_final != -2)) {
              if (dst_pass_depth_final >= 0) {
                dst_pass_depth_final = pass;
                break;
              }
              /* It is possible to transition depth to input, but it is a TODO. */
              desc.pDepthStencilAttachment = attachments_.reference_get(type_index, info_id_);
              subpass_add = true;
              if (final_layout == VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_STENCIL_READ_ONLY_OPTIMAL) {
                dependencies.append(
                    dependency_get(VK_SUBPASS_EXTERNAL,
                                   pass,
                                   SubpassTransitionPattern::EXTERNAL_SHADER_READ_TO_DEPTH));
                dst_pass_depth_final = -2;
              }
              else {
                dependencies.append(dependency_get(
                    VK_SUBPASS_EXTERNAL, pass, SubpassTransitionPattern::EXTERNAL_TO_DEPTH));
                dst_pass_depth_final = pass;
              }
              break;
            }
            else {
              /* The references are derived from the references generated in the main
               * configuration. */
              VkAttachmentReference2 copy_reference = *attachments_.reference_get(type_index,
                                                                                  info_id_);
              desc.colorAttachmentCount++;
              SubpassTransitionPattern t_pattern;
              switch (final_layout) {
                case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
                  t_pattern = SubpassTransitionPattern::EXTERNAL_TO_COLOR;
                  break;
                case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
                  t_pattern = SubpassTransitionPattern::EXTERNAL_SHADER_READ_TO_COLOR;
                  break;
                case VK_IMAGE_LAYOUT_GENERAL:
                  t_pattern = SubpassTransitionPattern::EXTERNAL_SHADER_WRITE_TO_COLOR;
                  break;
                default:
                  break;
              }
              if (check_dependency(VK_SUBPASS_EXTERNAL, pass, t_pattern)) {
                color_transition_next[pass] =
                    (t_pattern == SubpassTransitionPattern::EXTERNAL_TO_COLOR) ?
                        SubpassTransitionPattern::SUBPASS_TRANSITION_PATTERN_ALL :
                        static_cast<SubpassTransitionPattern>((~(uint8_t)t_pattern) & 0b1111);
                dependencies.append(dependency_get(VK_SUBPASS_EXTERNAL, pass, t_pattern));
              }
              if (references.size() < (shader_order + 1)) {
                references.resize(shader_order + 1);
              }
              references[shader_order] = copy_reference;
              BLI_assert(references[shader_order].attachment == view_order);
            }
            break;
          };
        }
      }
    }
    if (desc.colorAttachmentCount > 0) {
      desc.pColorAttachments = references.data();
      subpass_add = true;
    }
    if (desc.inputAttachmentCount > 0) {
      desc.pInputAttachments = ireferences.data();
      subpass_add = true;
    }
    if (subpass_add) {
      vk_create_info_[info_id_].subpassCount++;
      subpass_multi_attachments.append(subpasses[pass].colorAttachmentCount);
    }
  }
  if (dst_pass_depth_final > -1) {
    dependencies.append(dependency_get(dst_pass_depth_final,
                                       VK_SUBPASS_EXTERNAL,
                                       SubpassTransitionPattern::DEPTH_TO_EXTERNAL_SHADER_READ));
  }
  for (int pass = 0; pass < GPU_TEX_MAX_SUBPASS; pass++) {
    for (auto transition : {color_transition_next[pass], color_read_transition_next[pass]}) {
      if (transition == SubpassTransitionPattern::SUBPASS_TRANSITION_PATTERN_ALL) {
        continue;
      }
      if (check_dependency(pass, VK_SUBPASS_EXTERNAL, transition)) {
        dependencies.append(dependency_get(pass, VK_SUBPASS_EXTERNAL, transition));
      }
    }
  }
  vk_create_info_[info_id_].dependencyCount = dependencies.size();
  vk_create_info_[info_id_].pDependencies = dependencies.data();
  vk_create_info_[info_id_].pSubpasses = subpasses;

  create();
}
void VKRenderPass::multiview_set()
{
  /* TODO::Multiview Config */
  if (subpass_[info_id_].viewMask != subpass_[info_id_counter()].viewMask) {
    subpass_[info_id_counter()].viewMask = subpass_[info_id_].viewMask;
    dirty_ = true;
  }
}
}  // namespace blender::gpu
