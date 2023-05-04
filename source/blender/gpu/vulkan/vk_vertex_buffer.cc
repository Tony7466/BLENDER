/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#include "MEM_guardedalloc.h"

#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_vertex_buffer.hh"

namespace blender::gpu {

VKVertexBuffer::~VKVertexBuffer()
{
  release_data();
}

void VKVertexBuffer::bind_as_ssbo(uint binding)
{
  if (!buffer_.is_allocated()) {
    allocate();
  }

  VKContext &context = *VKContext::get();
  VKShader *shader = static_cast<VKShader *>(context.shader);
  const VKShaderInterface &shader_interface = shader->interface_get();
  const VKDescriptorSet::Location location = shader_interface.descriptor_set_location(
      shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER, binding);
  shader->pipeline_get().descriptor_set_get().bind_as_ssbo(*this, location);
}

void VKVertexBuffer::bind_as_texture(uint /*binding*/) {}

void VKVertexBuffer::wrap_handle(uint64_t /*handle*/) {}

void VKVertexBuffer::update_sub(uint /*start*/, uint /*len*/, const void * /*data*/) {}

void VKVertexBuffer::read(void *data) const
{
  VKContext &context = *VKContext::get();
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  command_buffer.submit();
  buffer_.read(data);
}

void VKVertexBuffer::acquire_data()
{
  if (usage_ == GPU_USAGE_DEVICE_ONLY) {
    return;
  }

  /* Discard previous data if any. */
  /* TODO: Use mapped memory. */
  MEM_SAFE_FREE(data);
  data = (uchar *)MEM_mallocN(sizeof(uchar) * this->size_alloc_get(), __func__);
}

void VKVertexBuffer::resize_data() {}

void VKVertexBuffer::release_data()
{
  MEM_SAFE_FREE(data);
}

static bool needs_conversion(const GPUVertAttr &vertex_attribute)
{
  return (vertex_attribute.fetch_mode == GPU_FETCH_INT_TO_FLOAT &&
          ELEM(vertex_attribute.comp_type, GPU_COMP_I32, GPU_COMP_U32));
}
/**
 * Check if the VertBuf format needs conversion as one or more attributes aren't natively
 * supported.
 *
 * For example GPU_COMP_I32 With GPU_FETCH_INT_TO_FLOAT isn't natively supported by Vulkan. We
 * should perform the conversion on the host.
 */
static bool needs_conversion(const GPUVertFormat &vertex_format)
{
  for (int attr_index : IndexRange(vertex_format.attr_len)) {
    const GPUVertAttr &vert_attr = vertex_format.attrs[attr_index];
    if (needs_conversion(vert_attr)) {
      return true;
    }
  }
  return false;
}

static bool inplace_conversion_doable(const GPUUsageType &usage)
{
  /* based on the flags the conversion can be done inplace.*/
  return usage == GPU_USAGE_STATIC;
}

void *VKVertexBuffer::convert() const
{
  void *out_data = data;
  if (!inplace_conversion_doable(usage_)) {
    out_data = MEM_dupallocN(out_data);
  }
  BLI_assert(format.deinterleaved);

  for (int attr_index : IndexRange(format.attr_len)) {
    const GPUVertAttr &vert_attr = format.attrs[attr_index];
    if (!needs_conversion(vert_attr)) {
      continue;
    }
    void *row_data = out_data + vert_attr.offset;
    for (int vert_index = 0; vert_index < vertex_len; vert_index++) {
      if (vert_attr.comp_type == GPU_COMP_I32) {
        for (int component : IndexRange(vert_attr.comp_len)) {
          int32_t *component_in = static_cast<int32_t *>(row_data) + component;
          float *component_out = static_cast<float *>(row_data) + component;
          *component_out = float(*component_in);
        }
      }
      else if (vert_attr.comp_type == GPU_COMP_U32) {
        for (int component : IndexRange(vert_attr.comp_len)) {
          uint32_t *component_in = static_cast<uint32_t *>(row_data) + component;
          float *component_out = static_cast<float *>(row_data) + component;
          *component_out = float(*component_in);
        }
      }
      row_data += format.stride;
    }
  }
  return out_data;
}

void VKVertexBuffer::upload_data()
{
  if (!buffer_.is_allocated()) {
    allocate();
  }

  if (flag &= GPU_VERTBUF_DATA_DIRTY) {
    void *converted_data = data;
    if (needs_conversion(format)) {
      printf("Slow conversion\n");
      converted_data = convert();
    }
    buffer_.update(converted_data);
    if (converted_data != data) {
      MEM_SAFE_FREE(converted_data);
    }
    if (usage_ == GPU_USAGE_STATIC) {
      MEM_SAFE_FREE(data);
    }

    flag &= ~GPU_VERTBUF_DATA_DIRTY;
    flag |= GPU_VERTBUF_DATA_UPLOADED;
  }
}

void VKVertexBuffer::duplicate_data(VertBuf * /*dst*/) {}

void VKVertexBuffer::allocate()
{
  buffer_.create(size_used_get(),
                 usage_,
                 static_cast<VkBufferUsageFlagBits>(VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT));
  debug::object_label(buffer_.vk_handle(), "VertexBuffer");
}

}  // namespace blender::gpu
