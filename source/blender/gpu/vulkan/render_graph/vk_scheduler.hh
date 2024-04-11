/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * The scheduler is responsible to find and reorder the nodes in the render graph to update an
 * image or buffer to its latest content and state.
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_nodes.hh"

namespace blender::gpu::render_graph {
class VKRenderGraph;

/**
 * VKScheduler is responsible for selecting and reordering of nodes in the render graph. This
 * selection and order is used to convert the nodes to commands and submitting it to the GPU.
 *
 * This scheduler selects all nodes in the order they were added to the render graph.
 *
 * This is an initial implementation and should be enhanced for:
 * - Moving data transfer and compute before drawing, when they are scheduled between drawing nodes
 *   that use the same pipeline.
 * - Only select the nodes that are only needed for the given vk_image/vk_buffer. When performing
 *   read-backs of buffers should be done with as least as possible nodes as they can block
 *   drawing. It is better to do handle most nodes just before presenting the image. This would
 * lead to less CPU locks.
 */
class VKScheduler {
 public:
  /**
   * Determine which nodes of the render graph should be selected and in what order they should
   * be executed to update the given vk_image to its latest content and state.
   *
   * NOTE: Currently will select all nodes.
   */
  void select_nodes_for_image(const VKRenderGraph &render_graph,
                              VkImage vk_image,
                              Vector<NodeHandle> &r_selected_nodes);

  /**
   * Determine which nodes of the render graph should be selected and in what order they should
   * be executed to update the given vk_buffer to its latest content and state.
   *
   * NOTE: Currently will select all nodes.
   */
  void select_nodes_for_buffer(const VKRenderGraph &render_graph,
                               VkBuffer vk_buffer,
                               Vector<NodeHandle> &r_selected_nodes);

 private:
  /**
   * Select all nodes.
   */
  void select_all_nodes(const VKRenderGraph &render_graph, Vector<NodeHandle> &r_selected_nodes);
};

}  // namespace blender::gpu::render_graph
