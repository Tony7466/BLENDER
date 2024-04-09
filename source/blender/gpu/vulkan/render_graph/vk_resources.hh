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
#include "vk_types.hh"

namespace blender::gpu::render_graph {

class VKCommandBuilder;

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
   * These resources cannot be destroyed and could be recreated externally.
   *
   * Actual layout can be changed externally and therefore will be reset when building command
   * buffers.
   */
  SWAP_CHAIN,
};

/**
 * Class to track resources.
 *
 * Resources are tracked on device level. Their are two kind of resources, namely buffers and
 * images. Each resource can have multiple versions; every time a resource is changed (written to)
 * a new version is tracked.
 */
class VKResources {
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

    /**
     * State tracking to ensure correct pipeline barriers can be created.
     */
    VKResourceBarrierState barrier_state;

    /**
     * Reset the image layout to its original state.
     *
     * The layout of swap chain images are externally managed. When they are used again we need to
     * ensure the correct state.
     *
     * NOTE: Also needed when working with external memory (Cycles, OpenXR, multi device).
     */
    void reset_image_layout()
    {
      barrier_state.image_layout = vk_image_layout;
    }
  };

  VKRenderGraphList<ResourceHandle, Resource> resources_;
  Map<VkImage, ResourceHandle> image_resources_;
  Map<VkBuffer, ResourceHandle> buffer_resources_;

  std::mutex mutex_;

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
   * Return the current version of the resource.
   */
  VersionedResource get_image(VkImage vk_image) const;

  /**
   * Return the resource handle of the given VkImage.
   */
  ResourceHandle get_image_handle(VkImage vk_image) const;

  /**
   * Return the resource handle of the given VkBuffer.
   */
  ResourceHandle get_buffer_handle(VkBuffer vk_buffer) const;

  /**
   * Reset the swap chain image layouts to its original layout.
   *
   * The layout of swap chain images are externally managed. When they are reused we need to
   * ensure the correct state.
   *
   * NOTE: This is also needed when working with external memory (Cycles, OpenXR, multi device
   * rendering).
   */
  void reset_image_layouts();

  /**
   * Get reference to the device mutex.
   *
   * The mutex is stored in resources due to:
   * - It protects resources and their state.
   * - Allowing test cases to do testing without setting up a device instance which requires ghost.
   * - Device instance isn't accessible in test cases.
   */
  std::mutex &mutex_get()
  {
    return mutex_;
  }

 private:
  /**
   * Get the current version of the resource.
   */
  static VersionedResource get_version(ResourceHandle handle, const Resource &resource);
  /**
   * Get the current version of the resource and increase the version.
   */
  static VersionedResource get_and_increase_version(ResourceHandle handle, Resource &resource);

  /* When a command buffer is reset the resources are re-synced. During the syncing the command
   * builder attributes are resized to reduce reallocations. */
  friend class VKCommandBuilder;
};

}  // namespace blender::gpu::render_graph
