/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "vk_common.hh"

namespace blender::gpu {

class VKBuffer;

/**
 * Helper struct to enable buffers to be bound with an offset.
 */
struct VKBufferWithOffset {
  VKBuffer &buffer;
  VkDeviceSize offset;
};

}  // namespace blender::gpu
