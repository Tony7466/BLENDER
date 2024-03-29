/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_render_graph_nodes.hh"

namespace blender::gpu {
class VKRenderGraph;

class VKRenderGraphCommandBuilder {
 private:
  Vector<NodeHandle> selected_nodes_;
  /**
   * Current state of each resource during command building. It will also keep track of
   * states/image layouts between submissions.
   */
  Vector<VKResourceBarrierState> resource_states_;

  /* Pool of VKBufferMemoryBarriers that can be reused when building barriers */
  Vector<VkBufferMemoryBarrier> vk_buffer_memory_barriers_;
  Vector<VkImageMemoryBarrier> vk_image_memory_barriers_;

  /** Template buffer memory barrier. */
  VkBufferMemoryBarrier vk_buffer_memory_barrier_;
  /** Template image memory barrier. */
  VkImageMemoryBarrier vk_image_memory_barrier_;

  VkPipeline active_compute_pipeline_ = VK_NULL_HANDLE;
  VkDescriptorSet active_compute_descriptor_set_ = VK_NULL_HANDLE;

  VkPipelineStageFlags src_stage_mask_ = VK_PIPELINE_STAGE_NONE;
  VkPipelineStageFlags dst_stage_mask_ = VK_PIPELINE_STAGE_NONE;

 public:
  VKRenderGraphCommandBuilder();

  /**
   * Reset the command builder.
   *
   * Needs to be called before each build_image/buffer.
   * Internal resources are reused to reduce memory operations.
   */
  void reset(VKRenderGraph &render_graph);

  /**
   * Build the commands to update the given vk_image to the last version
   */
  void build_image(VKRenderGraph &render_graph, VkImage vk_image);

  /**
   * Build the commands to update the given vk_buffer to the last version
   */
  void build_buffer(VKRenderGraph &render_graph, VkBuffer vk_buffer);

  /**
   * After the commands have been submitted the state needs to be updated.
   */
  void update_state_after_submission(VKRenderGraph &render_graph);

  /**
   * Remove a buffer or image resource from the command builder internals.
   *
   * Internally the command buffer keeps track of resource states. When a resource is deleted it
   * needs to be removed from the tracked state. New resources can reuse the same resource
   * handle. This cannot be detected at the moment the internals are reset, so this needs to be
   * done when the old resource is removed.
   */
  void remove_resource(ResourceHandle handle);

  /**
   * Ensure that the vk_image_layout is the given layout. If not it adds a transition to ensure
   * the given layout.
   *
   * NOTE: Should only be needed to ensure the swap chain images can be presented.
   * TODO: Migrate to a render graph node, so we can de-duplicate the code.
   */
  [[deprecated]] void ensure_image_layout(VKRenderGraph &render_graph,
                                          VkImage vk_image,
                                          VkImageLayout vk_image_layout);

 private:
  void build_node(VKRenderGraph &render_graph,
                  NodeHandle node_handle,
                  const VKRenderGraphNodes::Node &node);
  void build_node_clear_color_image(VKRenderGraph &render_graph,
                                    NodeHandle node_handle,
                                    const VKRenderGraphNodes::Node &node);
  void build_node_fill_buffer(VKRenderGraph &render_graph,
                              NodeHandle node_handle,
                              const VKRenderGraphNodes::Node &node);
  void build_node_copy_buffer(VKRenderGraph &render_graph,
                              NodeHandle node_handle,
                              const VKRenderGraphNodes::Node &node);
  void build_node_copy_buffer_to_image(VKRenderGraph &render_graph,
                                       NodeHandle node_handle,
                                       const VKRenderGraphNodes::Node &node);
  void build_node_copy_image(VKRenderGraph &render_graph,
                             NodeHandle node_handle,
                             const VKRenderGraphNodes::Node &node);
  void build_node_copy_image_to_buffer(VKRenderGraph &render_graph,
                                       NodeHandle node_handle,
                                       const VKRenderGraphNodes::Node &node);
  void build_node_blit_image(VKRenderGraph &render_graph,
                             NodeHandle node_handle,
                             const VKRenderGraphNodes::Node &node);
  void build_node_synchronization(VKRenderGraph &render_graph,
                                  NodeHandle node_handle,
                                  const VKRenderGraphNodes::Node &node);
  void build_node_dispatch(VKRenderGraph &render_graph,
                           NodeHandle node_handle,
                           const VKRenderGraphNodes::Node &node);

  void reset_barriers();
  void send_pipeline_barriers(VKRenderGraph &render_graph);

  void add_buffer_barriers(VKRenderGraph &render_graph,
                           NodeHandle node_handle,
                           VkPipelineStageFlags node_stages);
  void add_buffer_barrier(VkBuffer vk_buffer,
                          VkAccessFlags src_access_mask,
                          VkAccessFlags dst_access_mask);
  void add_buffer_read_barriers(VKRenderGraph &render_graph,
                                NodeHandle node_handle,
                                VkPipelineStageFlags node_stages);
  void add_buffer_write_barriers(VKRenderGraph &render_graph,
                                 NodeHandle node_handle,
                                 VkPipelineStageFlags node_stages);

  void add_image_barriers(VKRenderGraph &render_graph,
                          NodeHandle node_handle,
                          VkPipelineStageFlags node_stages);
  void add_image_barrier(VkImage vk_image,
                         VkAccessFlags src_access_mask,
                         VkAccessFlags dst_access_mask,
                         VkImageLayout old_image_layout,
                         VkImageLayout new_image_layout);
  void add_image_read_barriers(VKRenderGraph &render_graph,
                               NodeHandle node_handle,
                               VkPipelineStageFlags node_stages);
  void add_image_write_barriers(VKRenderGraph &render_graph,
                                NodeHandle node_handle,
                                VkPipelineStageFlags node_stages);
};

}  // namespace blender::gpu
