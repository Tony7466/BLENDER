/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "BLI_map.hh"
#include "BLI_stack.hh"
#include "BLI_utility_mixins.hh"
#include "BLI_vector.hh"

#include "gpu_context_private.hh"

#include "vk_common.hh"

namespace blender::gpu {
class VKDevice;
class VKBuffer;

class VKBufferPool : public NonCopyable {
  struct Pool {
    Vector<VKBuffer *> values;
    Stack<VKBuffer *> free_items;

    VKBuffer &acquire();
    void release(VKBuffer &buffer);
    void free_data(VKDevice &device);
  };

  Map<VkDeviceSize, Pool> pools_;
  /** Released resources that are still in use. They cannot be reused at this moment. */
  Stack<VKBuffer *> released_;

 public:
  VKBuffer &acquire(VkDeviceSize size);
  void release(VKBuffer &buffer);
  void reset();
  void free_data(VKDevice &device);
};
}  // namespace blender::gpu
