/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "render_graph/vk_resource_access_info.hh"
#include "render_graph/vk_resource_dependencies.hh"
#include "vk_node_class.hh"

namespace blender::gpu::render_graph {
struct VKDispatchData {
  VKPipelineData pipeline_data;
  uint32_t group_count_x;
  uint32_t group_count_y;
  uint32_t group_count_z;
};

/**
 * All information to add a new dispatch node to the render graph.
 */
struct VKDispatchCreateInfo : NonCopyable {
  VKDispatchData dispatch_node;
  VKResourceAccessInfo resources;
};

class VKDispatchNode : public VKNodeClass<VKNodeType::DISPATCH,
                                          VKDispatchCreateInfo,
                                          VKDispatchData,
                                          VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                          VKResourceType::IMAGE | VKResourceType::BUFFER> {
 public:
  template<typename Node>
  static void set_node_data(Node &node, const VKDispatchCreateInfo &create_info)
  {
    node.dispatch = create_info.dispatch_node;
    localize_shader_data(node.dispatch.pipeline_data, create_info.dispatch_node.pipeline_data);
  }

  template<typename Node> static void free_data(Node &node)
  {
    vk_pipeline_free_data(node.dispatch.pipeline_data);
  }

  static void build_resource_dependencies(VKResources &resources,
                                          VKResourceDependencies &dependencies,
                                          NodeHandle node_handle,
                                          const VKDispatchCreateInfo &create_info)
  {
    resource_access_build_dependencies(
        resources, dependencies, node_handle, create_info.resources);
  }

  static void build_commands(VKCommandBufferInterface &command_buffer,
                             const VKDispatchData &data,
                             VKBoundPipelines &r_bound_pipelines)
  {
    // TODO: introduce helper function in pipeline types.
    const VKPipelineData &pipeline_data = data.pipeline_data;
    if (assign_if_different(r_bound_pipelines.compute.vk_pipeline, pipeline_data.vk_pipeline)) {
      command_buffer.bind_pipeline(VK_PIPELINE_BIND_POINT_COMPUTE,
                                   r_bound_pipelines.compute.vk_pipeline);
    }

    if (assign_if_different(r_bound_pipelines.compute.vk_descriptor_set,
                            pipeline_data.vk_descriptor_set))
    {
      command_buffer.bind_descriptor_sets(VK_PIPELINE_BIND_POINT_COMPUTE,
                                          pipeline_data.vk_pipeline_layout,
                                          0,
                                          1,
                                          &r_bound_pipelines.compute.vk_descriptor_set,
                                          0,
                                          nullptr);
    }

    if (pipeline_data.push_constants_size) {
      command_buffer.push_constants(pipeline_data.vk_pipeline_layout,
                                    VK_SHADER_STAGE_COMPUTE_BIT,
                                    0,
                                    pipeline_data.push_constants_size,
                                    pipeline_data.push_constants_data);
    }

    command_buffer.dispatch(data.group_count_x, data.group_count_y, data.group_count_z);
  }
};
}  // namespace blender::gpu::render_graph
