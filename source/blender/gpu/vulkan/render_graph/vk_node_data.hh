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
 */
struct VKNodeData {
  VKNodeType type;
  union {
    VKBlitImageNode::Data blit_image;
    VKClearColorImageNode::Data clear_color_image;
    VKCopyBufferNode::Data copy_buffer;
    VKCopyBufferToImageNode::Data copy_buffer_to_image;
    VKCopyImageNode::Data copy_image;
    VKCopyImageToBufferNode::Data copy_image_to_buffer;
    VKDispatchNode::Data dispatch;
    VKFillBufferNode::Data fill_buffer;
    VKSynchronizationNode::Data synchronization;
  };
};

}  // namespace blender::gpu::render_graph
