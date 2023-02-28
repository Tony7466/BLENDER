/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */
#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_vertex_buffer.hh"

#include "vk_storage_buffer.hh"

#include "BLI_span.hh"

namespace blender::gpu {

void VKStorageBuffer::update(const void *data)
{
  VKContext &context = *VKContext::get();
  if (!buffer_.is_allocated()) {
    allocate(context);
  }
  buffer_.update(context, data);
}

void VKStorageBuffer::allocate(VKContext &context)
{
  buffer_.create(context,
                 size_in_bytes_,
                 usage_,
                 static_cast<VkBufferUsageFlagBits>(VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT));
}

void VKStorageBuffer::bind(int slot)
{
  VKContext &context = *VKContext::get();
  if (!buffer_.is_allocated()) {
    allocate(context);
  }
  VKShader *shader = static_cast<VKShader *>(context.shader);
  const VKShaderInterface &shader_interface = shader->interface_get();
  const VKDescriptorSet::Location location = shader_interface.descriptor_set_location(
      shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER, slot);
  shader->pipeline_get().descriptor_set_get().bind(*this, location);
}

void VKStorageBuffer::unbind()
{
}

static bool is_uniform(Span<uint32_t> data)
{
  BLI_assert(!data.is_empty());
  uint32_t expected_value = data[0];
  for (uint32_t value : data) {
    if (value != expected_value) {
      return false;
    }
  }
  return true;
}

static bool can_use_fill_command(eGPUTextureFormat internal_format,
                                 eGPUDataFormat /*data_format*/,
                                 void *data)
{
  int element_size = to_bytesize(internal_format);
  int num_components = element_size / 4;
  if (is_uniform(Span<uint32_t>(static_cast<uint32_t *>(data), num_components))) {
    return true;
  }
  return false;
}

void VKStorageBuffer::clear(eGPUTextureFormat internal_format,
                            eGPUDataFormat data_format,
                            void *data)
{
  VKContext &context = *VKContext::get();
  if (!buffer_.is_allocated()) {
    allocate(context);
  }

  VKCommandBuffer &command_buffer = context.command_buffer_get();
  if (can_use_fill_command(internal_format, data_format, data)) {
    /* When the data format is 4 bytes we can use vkCmdFillBuffer.
     * Common case when using GPU_storagebuf_clear_to_zero. */
    command_buffer.fill(buffer_, *static_cast<uint32_t *>(data));
    return;
  }

  /* TODO: Use a compute shader to clear the buffer. */
  BLI_assert_unreachable();
}

void VKStorageBuffer::copy_sub(VertBuf * /*src*/,
                               uint /*dst_offset*/,
                               uint /*src_offset*/,
                               uint /*copy_size*/)
{
}

void VKStorageBuffer::read(void *data)
{
  VKContext &context = *VKContext::get();
  if (!buffer_.is_allocated()) {
    allocate(context);
  }

  VKCommandBuffer &command_buffer = context.command_buffer_get();
  command_buffer.submit();

  void *mapped_memory;
  if (buffer_.map(context, &mapped_memory)) {
    memcpy(data, mapped_memory, size_in_bytes_);
    buffer_.unmap(context);
  }
}

}  // namespace blender::gpu
