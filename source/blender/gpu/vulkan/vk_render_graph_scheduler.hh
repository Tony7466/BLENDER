/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_render_graph_nodes.hh"

// TODO: Decide if scheduler is a better name... but scheduling is part of the command builder as
// well....
namespace blender::gpu {
class VKRenderGraphScheduler {
 public:
  virtual void select_nodes_for_image(VkImage vk_image, Vector<NodeHandle> &r_selected_nodes) = 0;
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
  void select_nodes_for_image(VkImage vk_image, Vector<NodeHandle> &r_selected_nodes) override;
};

}  // namespace blender::gpu
