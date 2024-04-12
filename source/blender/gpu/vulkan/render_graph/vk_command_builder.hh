/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"
#include "vk_nodes.hh"
#include "vk_scheduler.hh"

namespace blender::gpu::render_graph {
class VKRenderGraph;

/**
 * Build the command buffer for sending to the device queue.
 *
 * Determine which nodes needs to be scheduled, Then for each node generate the needed pipeline
 * barriers and commands.
 */
class VKCommandBuilder {
 private:
  VKScheduler scheduler_;

  Vector<NodeHandle> selected_nodes_;

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
   * Needs to be called before each build_image/buffer. It ensures that swapchain images are reset
   * to the correct layout and that the pipelines are reset.
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
   * After the commands have been submitted the nodes that have been send to the GPU can be
   * removed.
   */
  void update_state_after_submission(VKRenderGraph &render_graph);

 private:
  /**
   * Build multiple nodes.
   *
   * Building happen in order provided by the `node_handles` parameter.
   */
  void build_nodes(VKRenderGraph &render_graph, Span<NodeHandle> node_handles);

  /**
   * Build the commands for the given node_handle and node.
   *
   * The dependencies of the node handle are checked and when needed a pipeline barrier will be
   * generated and added to the command buffer.
   *
   * Based on the node.type the correct node class will be used for adding commands to the command
   * buffer.
   */
  void build_node(VKRenderGraph &render_graph,
                  NodeHandle node_handle,
                  const VKNodeData &node_data);

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

  /**
   * Build the pipeline barrier that will be recorded before the given node handle.
   */
  template<typename NodeClass>
  void build_pipeline_barriers(VKRenderGraph &render_graph, NodeHandle node_handle)
  {
    reset_barriers();
    if constexpr (bool(NodeClass::resource_usages & VKResourceType::IMAGE)) {
      add_image_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
    }
    if constexpr (bool(NodeClass::resource_usages & VKResourceType::BUFFER)) {
      add_buffer_barriers(render_graph, node_handle, NodeClass::pipeline_stage);
    }
    send_pipeline_barriers(render_graph);
  }

  /**
   * Build the commands for a node and record them to the given command buffer. These will include
   * pipeline barrier commands.
   *
   * The `r_bound_pipelines` is used by dispatch and draw commands to be setup the
   * (compute/graphics draw) pipelines using as few as possible commands.
   * Data transfer nodes should not use this parameter.
   */
  template<typename NodeClass, typename NodeClassData>
  void build_node(VKRenderGraph &render_graph,
                  VKCommandBufferInterface &command_buffer,
                  NodeHandle node_handle,
                  const NodeClassData &node_data,
                  VKBoundPipelines &r_bound_pipelines)
  {
    NodeClass node_class;
    build_pipeline_barriers<NodeClass>(render_graph, node_handle);
    node_class.build_commands(command_buffer, node_data, r_bound_pipelines);
  }
};

}  // namespace blender::gpu::render_graph
