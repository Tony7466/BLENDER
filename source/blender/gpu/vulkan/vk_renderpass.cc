/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_renderpass.hh"
#include "vk_backend.hh"
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
  if (vk_render_pass_ == VK_NULL_HANDLE) {
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

  if (vk_render_pass_ == VK_NULL_HANDLE) {
    return;
  }
  VKDevice &device = VKBackend::get().device_get();
  if (device.is_initialized()) {
    device.discard_render_pass(vk_render_pass_);
  }
  dirty_ = true;
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

  for (auto &i : attachments_.idx_[info_id_]) {
    i = -1;
  }
};

void VKRenderPass::dependency_set(bool use_depth)
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

void VKRenderPass::multiview_set()
{
  /* TODO::Multiview Config */
  if (subpass_[info_id_].viewMask != subpass_[info_id_counter()].viewMask) {
    subpass_[info_id_counter()].viewMask = subpass_[info_id_].viewMask;
    dirty_ = true;
  }
}
}  // namespace blender::gpu
