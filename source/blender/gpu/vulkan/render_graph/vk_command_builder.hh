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

  /* Pool of VKBufferMemoryBarriers that can be reused when building barriers */
  Vector<VkBufferMemoryBarrier> vk_buffer_memory_barriers_;
  Vector<VkImageMemoryBarrier> vk_image_memory_barriers_;

  /** Template buffer memory barrier. */
  VkBufferMemoryBarrier vk_buffer_memory_barrier_;
  /** Template image memory barrier. */
  VkImageMemoryBarrier vk_image_memory_barrier_;

  struct {
    /**
     * State of the bound pipelines during command building.
     */
    VKBoundPipelines active_pipelines;

    /**
     * When building memory barriers we need to track the src_stage_mask and dst_stage_mask and
     * pass them to
     * `https://docs.vulkan.org/spec/latest/chapters/synchronization.html#vkCmdPipelineBarrier`
     *
     * NOTE: Only valid between `reset_barriers` and `send_pipeline_barriers`.
     */
    VkPipelineStageFlags src_stage_mask = VK_PIPELINE_STAGE_NONE;
    VkPipelineStageFlags dst_stage_mask = VK_PIPELINE_STAGE_NONE;
  } state_;

 public:
  VKCommandBuilder();

  /**
   * Build the commands to update the given vk_image to the latest stamp. The commands are
   * recorded into the given `command_buffer`.
   *
   * `VKRenderGraph::submit_for_present` and its responsibility is to generate and record
   * all the commands needed to update the given vk_image to its latest stamp. The latest stamp is
   * determined by the last command has a write access to the `vk_image`.
   *
   * Pre-condition:
   * - `command_buffer` must not be in initial state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   *
   * Post-condition:
   * - `command_buffer` will be in executable state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   *
   * Result must be passed to the `remove_nodes` methods in `VKNodes` and `VKResourceDependencies`
   * to free their resources. This can be done after submission when the CPU is waiting.
   */
  [[nodiscard]] Span<NodeHandle> build_image(VKRenderGraph &render_graph,
                                             VKCommandBufferInterface &command_buffer,
                                             VkImage vk_image);

  /**
   * Build the commands to update the given vk_buffer to the latest stamp. The commands are
   * recorded into the given `command_buffer`.
   *
   * `VKRenderGraph::submit_buffer_for_read_back` and its responsibility is to generate and record
   * all the commands needed to update the given vk_buffer to its latest stamp. The latest stamp is
   * determined by the last command has a write access to the `vk_buffer`.
   *
   * Pre-condition:
   * - `command_buffer` must not be in initial state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   *
   * Post-condition:
   * - `command_buffer` will be in executable state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   *
   * Result must be passed to the `remove_nodes` methods in `VKNodes` and `VKResourceDependencies`
   * to free their resources. This can be done after submission when the CPU is waiting.
   */
  [[nodiscard]] Span<NodeHandle> build_buffer(VKRenderGraph &render_graph,
                                              VKCommandBufferInterface &command_buffer,
                                              VkBuffer vk_buffer);

 private:
  /**
   * Reset the command builder.
   *
   * Is called by `build_image/buffer` to ensure the internal state and swapchain images are reset.
   */
  void reset(VKRenderGraph &render_graph);

  /**
   * Build multiple nodes.
   *
   * Building happen in order provided by the `node_handles` parameter.
   *
   * Pre-condition:
   * - `command_buffer` must not be in initial state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   *
   * Post-condition:
   * - `command_buffer` will be in executable state according to
   *   https://docs.vulkan.org/spec/latest/chapters/cmdbuffers.html#commandbuffers-lifecycle
   */
  void build_nodes(VKRenderGraph &render_graph,
                   VKCommandBufferInterface &command_buffer,
                   Span<NodeHandle> node_handles);

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
                  VKCommandBufferInterface &command_buffer,
                  NodeHandle node_handle,
                  const VKNode &node);

  /**
   * Build the pipeline barriers that should be recorded before the given node handle.
   */
  void build_pipeline_barriers(VKRenderGraph &render_graph,
                               VKCommandBufferInterface &command_buffer,
                               NodeHandle node_handle,
                               VkPipelineStageFlags pipeline_stage);
  void reset_barriers();
  void send_pipeline_barriers(VKCommandBufferInterface &command_buffer);

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

}  // namespace blender::gpu::render_graph
