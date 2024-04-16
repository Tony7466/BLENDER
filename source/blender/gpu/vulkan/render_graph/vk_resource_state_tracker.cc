/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "BLI_index_range.hh"

#include "vk_resource_state_tracker.hh"

namespace blender::gpu::render_graph {

/* -------------------------------------------------------------------- */
/** \name Adding resources
 * \{ */

void VKResourceStateTracker::add_image(VkImage vk_image,
                                       VkImageLayout vk_image_layout,
                                       ResourceOwner owner)
{
  BLI_assert_msg(!image_resources_.contains(vk_image),
                 "Image resource is added twice to the render graph.");
  std::scoped_lock lock(mutex);
  ResourceHandle handle = resources_.allocate();
  Resource &resource = resources_.get(handle);
  image_resources_.add_new(vk_image, handle);

  resource.owner = owner;
  resource.vk_buffer = VK_NULL_HANDLE;
  resource.vk_image = vk_image;
  resource.vk_image_layout = vk_image_layout;
  resource.version = 0;
}

void VKResourceStateTracker::add_buffer(VkBuffer vk_buffer)
{
  BLI_assert_msg(!buffer_resources_.contains(vk_buffer),
                 "Buffer resource is added twice to the render graph.");
  std::scoped_lock lock(mutex);
  ResourceHandle handle = resources_.allocate();
  Resource &resource = resources_.get(handle);
  buffer_resources_.add_new(vk_buffer, handle);

  resource.owner = ResourceOwner::APPLICATION;
  resource.vk_buffer = vk_buffer;
  resource.vk_image = VK_NULL_HANDLE;
  resource.vk_image_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  resource.version = 0;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Remove resources
 * \{ */

void VKResourceStateTracker::remove_buffer(VkBuffer vk_buffer)
{
  std::scoped_lock lock(mutex);
  ResourceHandle handle = buffer_resources_.pop(vk_buffer);
  resources_.free(handle);
}

void VKResourceStateTracker::remove_image(VkImage vk_image)
{
  std::scoped_lock lock(mutex);
  ResourceHandle handle = image_resources_.pop(vk_image);
  resources_.free(handle);
}

/** \} */

ResourceHandle VKResourceStateTracker::get_image_handle(VkImage vk_image) const
{
  return image_resources_.lookup(vk_image);
}
ResourceHandle VKResourceStateTracker::get_buffer_handle(VkBuffer vk_buffer) const
{
  return buffer_resources_.lookup(vk_buffer);
}

ResourceWithStamp VKResourceStateTracker::get_version(ResourceHandle handle,
                                                      const Resource &resource)
{
  ResourceWithStamp result;
  result.handle = handle;
  result.stamp = resource.version;
  return result;
}

ResourceWithStamp VKResourceStateTracker::get_and_increase_version(ResourceHandle handle,
                                                                   Resource &resource)
{
  ResourceWithStamp result = get_version(handle, resource);
  resource.version += 1;
  return result;
}

ResourceWithStamp VKResourceStateTracker::get_image_and_increase_version(VkImage vk_image)
{
  ResourceHandle handle = get_image_handle(vk_image);
  Resource &resource = resources_.get(handle);
  return get_and_increase_version(handle, resource);
}

ResourceWithStamp VKResourceStateTracker::get_buffer_and_increase_version(VkBuffer vk_buffer)
{
  ResourceHandle handle = get_buffer_handle(vk_buffer);
  Resource &resource = resources_.get(handle);
  return get_and_increase_version(handle, resource);
}

ResourceWithStamp VKResourceStateTracker::get_buffer(VkBuffer vk_buffer) const
{
  ResourceHandle handle = get_buffer_handle(vk_buffer);
  const Resource &resource = resources_.get(handle);
  return get_version(handle, resource);
}

ResourceWithStamp VKResourceStateTracker::get_image(VkImage vk_image) const
{
  ResourceHandle handle = get_image_handle(vk_image);
  const Resource &resource = resources_.get(handle);
  return get_version(handle, resource);
}

void VKResourceStateTracker::reset_image_layouts()
{
  for (ResourceHandle image_handle : image_resources_.values()) {
    VKResourceStateTracker::Resource &resource = resources_.get(image_handle);
    if (resource.owner == ResourceOwner::SWAP_CHAIN) {
      resource.reset_image_layout();
    }
  }
}

}  // namespace blender::gpu::render_graph
