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

static VkDeviceSize new_buffer_size(size_t sub_buffer_size)
{
  return max_ii(sub_buffer_size, DEFAULT_INTERNAL_BUFFER_SIZE);
}

uchar *VKImmediate::begin()
{
  VKContext &context = *VKContext::get();
  const size_t bytes_needed = vertex_buffer_size(&vertex_format, vertex_len);
  const bool new_buffer_needed = !buffer_.is_allocated() || buffer_bytes_free() < bytes_needed;
  if (new_buffer_needed) {
    /* TODO: get buffer from context.reusable_buffers. */
    BLI_assert(!buffer_.is_allocated());
    buffer_.create(context,
                   new_buffer_size(bytes_needed),
                   GPU_USAGE_DYNAMIC,
                   static_cast<VkBufferUsageFlagBits>(VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT));
    buffer_offset_ = 0;
    current_subbuffer_len_ = 0;
  }

  if (bytes_needed <= buffer_bytes_free()) {
    current_subbuffer_len_ = bytes_needed;
  }
  else {
    /* TODO allocate new buffer. */
  }

  uchar *data = static_cast<uchar *>(buffer_.mapped_memory_get());
  return data + subbuffer_offset_get();
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
  buffer_offset_ += current_subbuffer_len_;
  current_subbuffer_len_ = 0;
}

VkDeviceSize VKImmediate::subbuffer_offset_get()
{
  return buffer_offset_;
}

VkDeviceSize VKImmediate::buffer_bytes_free()
{
  return buffer_.size_in_bytes() - subbuffer_offset_get();
}

/** \} */

}  // namespace blender::gpu
