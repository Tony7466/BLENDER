/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_resource_access_info.hh"
#include "vk_resource_dependencies.hh"
#include "vk_resources.hh"

namespace blender::gpu::render_graph {

void resource_access_build_dependencies(VKResources &resources,
                                        VKResourceDependencies &dependencies,
                                        NodeHandle node_handle,
                                        const VKResourceAccessInfo &access_info)
{
  // TODO: Add a debug time validation that resources are unique (or merged).
  for (const VKBufferAccess &buffer_access : access_info.buffers) {
    VkAccessFlags read_access = buffer_access.vk_access_flags & VK_ACCESS_READ_MASK;
    if (read_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources.get_buffer(buffer_access.vk_buffer);
      dependencies.add_read_resource(
          node_handle, versioned_resource, read_access, VK_IMAGE_LAYOUT_UNDEFINED);
    }

    VkAccessFlags write_access = buffer_access.vk_access_flags & VK_ACCESS_WRITE_MASK;
    if (write_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources.get_buffer_and_increase_version(
          buffer_access.vk_buffer);
      dependencies.add_write_resource(
          node_handle, versioned_resource, write_access, VK_IMAGE_LAYOUT_UNDEFINED);
    }
  }

  for (const VKImageAccess &image_access : access_info.images) {
    VkAccessFlags read_access = image_access.vk_access_flags & VK_ACCESS_READ_MASK;
    if (read_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources.get_image(image_access.vk_image);
      dependencies.add_read_resource(
          node_handle, versioned_resource, read_access, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }

    VkAccessFlags write_access = image_access.vk_access_flags & VK_ACCESS_WRITE_MASK;
    if (write_access != VK_ACCESS_NONE) {
      VersionedResource versioned_resource = resources.get_image_and_increase_version(
          image_access.vk_image);
      /* Extract the correct layout to use from the access flags. */
      dependencies.add_write_resource(
          node_handle, versioned_resource, write_access, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    }
  }
}

}  // namespace blender::gpu::render_graph
