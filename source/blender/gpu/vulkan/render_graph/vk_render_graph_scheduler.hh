/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_nodes.hh"

// TODO: Decide if scheduler is a better name... but scheduling is part of the command builder as
// well....
namespace blender::gpu::render_graph {
class VKRenderGraph;

class VKRenderGraphScheduler {
 public:
  virtual ~VKRenderGraphScheduler() = default;
  virtual void select_nodes_for_image(const VKRenderGraph &render_graph,
                                      VkImage vk_image,
                                      Vector<NodeHandle> &r_selected_nodes) = 0;
  virtual void select_nodes_for_buffer(const VKRenderGraph &render_graph,
                                       VkBuffer vk_buffer,
                                       Vector<NodeHandle> &r_selected_nodes) = 0;
};

/**
 * Sequential execution.
 *
 * As best as possible will stick to sequence how nodes where added to the render graph.
 * There are cases where the nodes needs to be reordered for example when performing a
 * data transfer operation during rendering. This is not allowed by Vulkan.
 */
class Sequential : public VKRenderGraphScheduler {
 public:
  /**
   * Select all nodes to the last access (read or write) of the given image.
   */
  void select_nodes_for_image(const VKRenderGraph &render_graph,
                              VkImage vk_image,
                              Vector<NodeHandle> &r_selected_nodes) override;
  void select_nodes_for_buffer(const VKRenderGraph &render_graph,
                               VkBuffer vk_buffer,
                               Vector<NodeHandle> &r_selected_nodes) override;

 private:
  /**
   * Select all nodes in the graph.
   *
   * In the future we could only select a subset to build a resource, but still in sequence.
   */
  void select_all_nodes(const VKRenderGraph &render_graph, Vector<NodeHandle> &r_selected_nodes);
};

}  // namespace blender::gpu::render_graph
