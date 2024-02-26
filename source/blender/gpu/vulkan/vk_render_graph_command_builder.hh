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
  void reset(VKRenderGraph &render_graph);
  /**
   * Build the commands to update the given vk_image to the last version
   */
  void build_image(VKRenderGraph &render_graph, VkImage vk_image);

  /**
   * Ensure that the vk_image_layout is the given layout. If not it adds a transition to ensure the
   * given layout.
   */
  void ensure_image_layout(VKRenderGraph &render_graph,
                           VkImage vk_image,
                           VkImageLayout vk_image_layout);
};

}  // namespace blender::gpu
