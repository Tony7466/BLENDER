/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Nodes inside the render graph are connected via links to the resources they use. These links are
 * determined when adding a node to the render graph.
 *
 * The inputs of the node link to the resources that the node reads from. The outputs of the node
 * link to the resources that the node modifies.
 *
 * All links inside the graph are stored inside `VKResourceDependencies`.
 */

#pragma once

#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_resource_state_tracker.hh"

namespace blender::gpu::render_graph {

struct VKRenderGraphLink {
  /**
   * Which resource is being accessed.
   */
  ResourceWithStamp resource;

  /**
   * How is the resource being accessed.
   *
   * When generating pipeline barriers of a resource, the nodes access flags are evaluated to
   * create src/dst access masks.
   */
  VkAccessFlags vk_access_flags;

  /**
   * When resource is an image, which layout should the image be using.
   *
   * When generating the commands this attribute is compared with the actual image layout of the
   * the image. Additional pipeline barriers will be added to transit to the layout stored here.
   */
  VkImageLayout vk_image_layout;
};

/**
 * All input and output links of a node in the render graph.
 */
struct VKRenderGraphNodeLinks {
  /** All links to resources that a node reads from. */
  Vector<VKRenderGraphLink> inputs;
  /** All links to resources that a node writes to. */
  Vector<VKRenderGraphLink> outputs;

  /**
   * Add an input link.
   *
   * An input link describes a resource that the node reads from.
   *
   * Access flags should be added to steer the pipeline barrier generation. During pipeline barrier
   * generation previous and future usages of the resource are checked to construct the correct
   * pipeline barrier.
   *
   * When the resource is an image resource the needed image layout should be passed
   * (`vk_image_layout`) to ensure correct image layout transition. For buffer resources this can
   * be set to `VK_IMAGE_LAYOUT_UNDEFINED`.
   */
  void add_input(ResourceWithStamp resource_handle,
                 VkAccessFlags vk_access_flags,
                 VkImageLayout vk_image_layout)
  {
    VKRenderGraphLink link = {};
    link.resource = resource_handle;
    link.vk_access_flags = vk_access_flags;
    link.vk_image_layout = vk_image_layout;
    inputs.append(link);
  }

  /**
   * Add an output link.
   *
   * An output link describes a resource that the node modifies/writes to.
   *
   * Access flags should be added to steer the pipeline barrier generation. During pipeline barrier
   * generation previous and future usages of the resource are checked to construct the correct
   * pipeline barrier.
   *
   * When the resource is an image resource the needed image layout should be passed
   * (`vk_image_layout`) to ensure correct image layout transition. For buffer resources this can
   * be set to `VK_IMAGE_LAYOUT_UNDEFINED`.
   */
  void add_output(ResourceWithStamp resource_handle,
                  VkAccessFlags vk_access_flags,
                  VkImageLayout vk_image_layout)
  {
    VKRenderGraphLink link = {};
    link.resource = resource_handle;
    link.vk_access_flags = vk_access_flags;
    link.vk_image_layout = vk_image_layout;
    outputs.append(link);
  }
};

}  // namespace blender::gpu::render_graph
