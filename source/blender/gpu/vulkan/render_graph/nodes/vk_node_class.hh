/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "render_graph/vk_command_buffer_wrapper.hh"
#include "render_graph/vk_resource_dependencies.hh"
#include "render_graph/vk_resources.hh"
#include "vk_common.hh"
#include "vk_types_pipeline.hh"

namespace blender::gpu::render_graph {

/**
 * Base class for node class.
 *
 * Nodes can be created using `NodeCreateInfo`. When a node is created the `VKNodeClass.node_type`
 * and `VKNodeClass.set_node_data` are used to fill a VKNodeData instance. The VKNodeData is stored
 * sequentially in the render graph. When the node is created the dependencies are extracted by
 * calling `VKNodeClass.build_resource_dependencies`.
 *
 * Eventually when a node should be added to a command buffer `VKNodeClass.build_commands` are
 * invoked.
 */
template<VKNodeType NodeType,
         typename NodeCreateInfo,
         typename NodeData,
         VkPipelineStageFlagBits PipelineStage,
         VKResourceType ResourceUsages>
class VKNodeClass : public NonCopyable {
 public:
  /**
   * Node type of this class.
   *
   * The node type used to link VKNodeData instance to a VKNodeClass.
   */
  static constexpr VKNodeType node_type = NodeType;

  /**
   * Which pipeline stage does this command belongs to. The pipeline stage is used when generating
   * pipeline barriers.
   */
  static constexpr VkPipelineStageFlags pipeline_stage = PipelineStage;

  /**
   * Which resource types are relevant. Some code can be skipped when a node can only depend on
   * resources of a single type.
   */
  static constexpr VKResourceType resource_usages = ResourceUsages;

  /**
   * Update the node data with the data inside create_info.
   *
   * Has been implemented as a template to ensure all node specific data
   * (`VK*Data`/`VK*CreateInfo`) types can be included in the same header file as the logic. The
   * actual node data (`VKNodeData` includes all header files.)
   *
   * This function must be implemented by all node classes. But due to cyclic inclusion of header
   * files it is implemented as a template function.
   */
  template<typename Node> static void set_node_data(Node &node, const NodeData &create_info);

  /**
   * Extract read/write resource dependencies from `create_info` and add them to `dependencies`.
   */
  virtual void build_resource_dependencies(VKResources &resources,
                                           VKResourceDependencies &dependencies,
                                           NodeHandle node_handle,
                                           const NodeCreateInfo &create_info) = 0;

  /**
   * Build the commands and add them to the command_buffer.
   *
   * The command buffer is passed as an interface as this is replaced by a logger when running test
   * cases. The test cases will validate the log to find out if the correct commands where added.
   */
  virtual void build_commands(VKCommandBufferInterface &command_buffer,
                              const NodeData &data,
                              VKBoundPipelines &r_bound_pipelines) = 0;
};
}  // namespace blender::gpu::render_graph
