/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_types.hh"

#include "nodes/vk_blit_image_node.hh"
#include "nodes/vk_clear_color_image_node.hh"
#include "nodes/vk_copy_buffer_node.hh"
#include "nodes/vk_copy_buffer_to_image_node.hh"
#include "nodes/vk_copy_image_node.hh"
#include "nodes/vk_copy_image_to_buffer_node.hh"
#include "nodes/vk_dispatch_node.hh"
#include "nodes/vk_fill_buffer_node.hh"
#include "nodes/vk_synchronization_node.hh"

namespace blender::gpu::render_graph {

/**
 * Node stored inside a render graph.
 *
 * Node specific data in the render graph are stored in a vector to ensure that the data can be
 * prefetched and removing a level of indirection. A consequence is that we cannot use class based
 * nodes.
 */
struct VKNodeData {
  VKNodeType type;
  union {
    VKBlitImageData blit_image;
    VKClearColorImageData clear_color_image;
    VKCopyBufferData copy_buffer;
    VKCopyBufferToImageData copy_buffer_to_image;
    VKCopyImageData copy_image;
    VKCopyImageToBufferData copy_image_to_buffer;
    VKDispatchData dispatch;
    VKFillBufferData fill_buffer;
    VKSynchronizationData synchronization;
  };
};

}  // namespace blender::gpu::render_graph
