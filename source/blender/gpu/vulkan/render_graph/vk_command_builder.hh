/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_nodes.hh"

namespace blender::gpu::render_graph {
class VKRenderGraph;

class VKCommandBuilder {
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

  VKBoundPipelines active_pipelines;

  VkPipelineStageFlags src_stage_mask_ = VK_PIPELINE_STAGE_NONE;
  VkPipelineStageFlags dst_stage_mask_ = VK_PIPELINE_STAGE_NONE;

 public:
  VKCommandBuilder();

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

 private:
  void build_node(VKRenderGraph &render_graph, NodeHandle node_handle, const VKNodes::Node &node);

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

  template<typename NodeClass, typename NodeClassData>
  void build_node(VKRenderGraph &render_graph,
                  VKCommandBufferInterface &command_buffer,
                  NodeHandle node_handle,
                  const NodeClassData &node_data)
  {
    if constexpr (NodeClass::uses_buffer_resources || NodeClass::uses_image_resources) {
      reset_barriers();
      if constexpr (NodeClass::uses_image_resources) {
        add_image_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
      }
      if constexpr (NodeClass::uses_buffer_resources) {
        add_buffer_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
      }
      send_pipeline_barriers(render_graph);
    }
    NodeClass::build_commands(command_buffer, node_data);
  }

  template<typename NodeClass, typename NodeClassData>
  void build_node(VKRenderGraph &render_graph,
                  VKCommandBufferInterface &command_buffer,
                  NodeHandle node_handle,
                  const NodeClassData &node_data,
                  VKBoundPipelines &r_bound_pipelines)
  {
    if constexpr (NodeClass::uses_buffer_resources || NodeClass::uses_image_resources) {
      reset_barriers();
      if constexpr (NodeClass::uses_image_resources) {
        add_image_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
      }
      if constexpr (NodeClass::uses_buffer_resources) {
        add_buffer_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
      }
      send_pipeline_barriers(render_graph);
    }
    NodeClass::build_commands(command_buffer, node_data, r_bound_pipelines);
  }
};

}  // namespace blender::gpu::render_graph
