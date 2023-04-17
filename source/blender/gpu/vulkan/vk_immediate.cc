/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 *
 * Mimics old style OpenGL immediate mode drawing.
 */

#include "vk_immediate.hh"

#include "gpu_vertex_format_private.h"

namespace blender::gpu {

VKImmediate::VKImmediate() {}
VKImmediate::~VKImmediate() {}

uchar *VKImmediate::begin()
{
  VKContext &context = *VKContext::get();
  const size_t bytes_needed = vertex_buffer_size(&vertex_format, vertex_len);
  if (!buffer_.is_allocated()) {
    buffer_.create(context,
                   DEFAULT_INTERNAL_BUFFER_SIZE,
                   GPU_USAGE_DYNAMIC,
                   static_cast<VkBufferUsageFlagBits>(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT));
  }

  /* TODO: Resize buffer when more is needed. Currently assert as we haven't implemented it yet. */
  BLI_assert(bytes_needed < DEFAULT_INTERNAL_BUFFER_SIZE);

  uchar *data = static_cast<uchar *>(buffer_.mapped_memory_get());
  return data;
}

void VKImmediate::end()
{
  BLI_assert_msg(prim_type != GPU_PRIM_NONE, "Illegal state: not between an immBegin/End pair.");
  if (vertex_len == 0) {
    return;
  }

  VKContext &context = *VKContext::get();
  BLI_assert(context.shader == unwrap(shader));
  context.state_manager->apply_state();
  vertex_attributes_.update_bindings(*this);
  context.bind_graphics_pipeline(prim_type, vertex_attributes_);
  vertex_attributes_.bind(context);

  context.command_buffer_get().draw(0, vertex_len, 0, 1);
}

/** \} */

}  // namespace blender::gpu
