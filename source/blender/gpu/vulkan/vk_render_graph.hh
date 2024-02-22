/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 *
 * Render graph is a render solution that is able to track resource usages in a single submission
 *
 * The graph contains nodes that refers to resources it depends on, or alters.
 *
 * Resources needs to be tracked as usage can alter the content of the resource. For example an
 * image can be optimized for data transfer, or optimized for sampling which can use a different
 * pixel layout on the device.
 *
 */

#pragma once

#include <mutex>
#include <optional>

#include "BLI_vector.hh"

#include "vk_common.hh"

namespace blender::gpu {

using ResourceHandle = uint64_t;
using NodeHandle = uint64_t;

class VKRenderGraph {
 public:
  /**
   * A node contains a draw/dispatch or transfer command. These commands depends on resources and
   * may also alter resources.
   */
  struct Node {
    Vector<ResourceHandle> read_resources;
    Vector<ResourceHandle> write_resources;
    // TODO: Bindings. shader, push constants
  };

  /**
   * A render resource can be a buffer or an image that needs to be tracked during rendering.
   *
   * Resources needs to be tracked as usage can alter the content of the resource. For example an
   * image can be optimized for data transfer, or optimized for sampling which can use a different
   * pixel layout on the device.
   */
  struct Resource {
    VkBuffer vk_buffer;
    VkImage vk_image;
    VkSampler vk_sampler;
    /**
     * Node that has altered the buffer/image to this state. Any usage of this resource is
     * dependant on the modifier.
     * This is an optional attribute as the last modifier could be rendered in the previous
     * frame and we are not tracking resource dependencies between frames.
     */
    std::optional<NodeHandle> modifier;
  };

 private:
  /**
   * Mutex locks adding new commands to a render graph that is being submitted.
   */
  std::mutex mutex_;
  Vector<Node> nodes_;
  Vector<Resource> resources_;
  // TODO: add resource lookup to latest version of the resource.

 public:
  void add_node(Node &node);
  ResourceHandle find_or_add_buffer(VkBuffer vk_buffer);
  ResourceHandle find_or_add_image(VkImage vk_image, VkSampler vk_sampler);
  void submit();
  void finish();
};

}  // namespace blender::gpu
