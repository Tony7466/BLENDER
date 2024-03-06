/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_map.hh"
#include "BLI_vector.hh"

#include "vk_common.hh"
#include "vk_render_graph_list.hh"

namespace blender::gpu {

class VKRenderGraphCommandBuilder;

using ResourceHandle = uint64_t;
using ResourceVersion = uint64_t;

struct VersionedResource {
  ResourceHandle handle;
  ResourceVersion version;
};

enum class ResourceOwner {
  /**
   * Resource is owned by the application.
   *
   * The resource can be destroyed by the application
   * when it isn't used anymore.
   */
  APPLICATION,

  /**
   * Resource is owned by the swapchain.
   *
   * These resources cannot be destroyed and might be recreated by the swap chain at all times.
   */
  SWAP_CHAIN,
};

class VKRenderGraphResources {
  /**
   * A render resource can be a buffer or an image that needs to be tracked during rendering.
   *
   * Resources needs to be tracked as usage can alter the content of the resource. For example an
   * image can be optimized for data transfer, or optimized for sampling which can use a different
   * pixel layout on the device.
   */
  struct Resource {
    VkBuffer vk_buffer = VK_NULL_HANDLE;
    VkImage vk_image = VK_NULL_HANDLE;

    /**
     * Last image layout that has been submitted to the queue.
     */
    VkImageLayout vk_image_layout = VK_IMAGE_LAYOUT_UNDEFINED;

    /**
     * Current version of the resource in the graph.
     */
    ResourceVersion version = 0;

    /** Who owns the resource. */
    ResourceOwner owner = ResourceOwner::APPLICATION;
  };

  VKRenderGraphList<ResourceHandle, Resource> resources_;
  Map<VkImage, ResourceHandle> image_resources_;
  Map<VkBuffer, ResourceHandle> buffer_resources_;

 public:
  /**
   * Register a buffer resource to the render graph.
   */
  void add_buffer(VkBuffer vk_buffer);

  /**
   * Register an image resource to the render graph.
   */
  void add_image(VkImage vk_image, VkImageLayout vk_image_layout, ResourceOwner owner);

  /**
   * Remove an registered image from the resource list.
   */
  void remove_image(VkImage vk_image);

  /**
   * Remove an registered buffer from the resource list.
   */
  void remove_buffer(VkBuffer vk_buffer);

  /**
   * Return the current version of the resource, and increase the version.
   */
  VersionedResource get_image_and_increase_version(VkImage vk_image);

  /**
   * Return the current version of the resource, and increase the version.
   */
  VersionedResource get_buffer_and_increase_version(VkBuffer vk_buffer);
  /**
   * Return the current version of the resource.
   */
  VersionedResource get_buffer(VkBuffer vk_buffer) const;
  /**
   * Return the current version of the resource.ß
   */
  VersionedResource get_image(VkImage vk_image) const;
  ResourceHandle get_image_handle(VkImage vk_image) const;
  ResourceHandle get_buffer_handle(VkBuffer vk_buffer) const;

  friend class VKRenderGraphCommandBuilder;

 private:
  /**
   * Get the current version of the resource.
   */
  static VersionedResource get_version(ResourceHandle handle, const Resource &resource);
  /**
   * Get the current version of the resource and increase the version.
   */
  static VersionedResource get_and_increase_version(ResourceHandle handle, Resource &resource);
};

}  // namespace blender::gpu
