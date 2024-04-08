/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_utility_mixins.hh"

#include "vk_common.hh"
#include "vk_types.hh"

namespace blender::gpu::render_graph {
class VKResources;
class VKResourceDependencies;

struct VKImageAccess {
  VkImage vk_image;
  VkAccessFlags vk_access_flags;
};

struct VKBufferAccess {
  VkBuffer vk_buffer;
  VkAccessFlags vk_access_flags;
};

struct VKResourceAccessInfo : NonCopyable {
  Vector<VKBufferAccess> buffers;
  Vector<VKImageAccess> images;

  void clear()
  {
    buffers.clear();
    images.clear();
  }
};

void resource_access_build_dependencies(VKResources &resources,
                                        VKResourceDependencies &dependencies,
                                        NodeHandle node_handle,
                                        const VKResourceAccessInfo &access_info);

}  // namespace blender::gpu::render_graph
