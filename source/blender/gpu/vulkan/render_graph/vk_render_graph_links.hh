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
/**
 * Index of a node inside the render graph.
 * Defined here to solve a include dependency cycle.
 */
using NodeHandle = uint64_t;

/**
 * Class containing all links inside the render graph.
 */
class VKRenderGraphLinks : NonCopyable, NonMovable {
 public:
  struct Link {
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

  struct NodeLinks {
    /** All links to resources that a node reads from. */
    Vector<Link> inputs;
    /** All links to resources that a node writes to. */
    Vector<Link> outputs;
  };

 private:
  Vector<NodeLinks> links_per_node_;

 public:
  /**
   * Add an input link for the given node handle.
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
  void add_input(NodeHandle handle,
                 ResourceWithStamp resource_handle,
                 VkAccessFlags vk_access_flags,
                 VkImageLayout vk_image_layout);

  /**
   * Add an output link for the given node handle.
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
  void add_output(NodeHandle handle,
                  ResourceWithStamp resource_handle,
                  VkAccessFlags vk_access_flags,
                  VkImageLayout vk_image_layout);

  /**
   * Get all outputs connected to the given node_handle.
   *
   * Output links contain resources that the node modified/writes to.
   */
  Span<Link> get_outputs(NodeHandle node_handle)
  {
    return links_per_node_[node_handle].outputs;
  }

  /**
   * Get all inputs connected to the given node_handle.
   *
   * Input links contain resources that the node reads from.
   */
  Span<Link> get_inputs(NodeHandle node_handle)
  {
    return links_per_node_[node_handle].inputs;
  }

  /**
   * Remove all links for the given node_handles.
   */
  void remove_links(Span<NodeHandle> node_handles);

 private:
  void ensure_capacity(NodeHandle node_handle);
};

}  // namespace blender::gpu::render_graph
