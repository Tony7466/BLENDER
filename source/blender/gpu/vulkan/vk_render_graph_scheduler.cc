/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_render_graph_scheduler.hh"

namespace blender::gpu {

void Sequential::select_nodes_for_image(VkImage vk_image, Vector<NodeHandle> &r_selected_nodes)
{
  // select all nodes from the start of the graph to the last write access
}

}  // namespace blender::gpu
