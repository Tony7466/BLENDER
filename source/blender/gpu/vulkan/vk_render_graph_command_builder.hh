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
  Vector<VkImageLayout> image_layouts_;

 public:
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
   * Ensure that the vk_image_layout is the given layout. If not it adds a transition to ensure the
   * given layout.
   */
  void ensure_image_layout(VKRenderGraph &render_graph,
                           VkImage vk_image,
                           VkImageLayout vk_image_layout);

 private:
  void build_node(VKRenderGraph &render_graph, VKRenderGraphNodes::Node &node);
  void build_node_clear_color_image(VKRenderGraph &render_graph, VKRenderGraphNodes::Node &node);
  void build_node_fill_buffer(VKRenderGraph &render_graph, VKRenderGraphNodes::Node &node);
};

}  // namespace blender::gpu
