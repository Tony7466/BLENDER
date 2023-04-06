/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

#include "vk_vertex_attribute_object.hh"

#include "vk_batch.hh"
#include "vk_context.hh"
#include "vk_shader.hh"
#include "vk_shader_interface.hh"
#include "vk_vertex_buffer.hh"

#include "BLI_array.hh"

namespace blender::gpu {
VKVertexAttributeObject::VKVertexAttributeObject()
{
  clear();
}

void VKVertexAttributeObject::clear()
{
  is_valid = false;
  info.pNext = NULL;
  bindings.clear();
  attributes.clear();
  vbos.clear();
}

VKVertexAttributeObject &VKVertexAttributeObject::operator=(const VKVertexAttributeObject &other)
{
  if (this == &other) {
    return *this;
  }

  /* TODO: resize is an exact resize. Might introduce unneeded re-allocations. */
  /* TODO: This might copy data, that isn't needed. */

  is_valid = other.is_valid;
  info = other.info;
  bindings.resize(other.bindings.size());
  int i = 0;
  for (const VkVertexInputBindingDescription &e : other.bindings) {
    bindings[i++] = e;
  }
  i = 0;
  attributes.resize(other.attributes.size());
  for (const VkVertexInputAttributeDescription e : other.attributes) {
    attributes[i++] = e;
  }
  i = 0;
  vbos.resize(other.vbos.size());
  for (VKVertexBuffer *e : other.vbos) {
    vbos[i++] = e;
  }
  return *this;
}

void VKVertexAttributeObject::bind(VKContext &context)
{
  Array<bool> visited_bindings(bindings.size());
  visited_bindings.fill(false);

  for (VkVertexInputAttributeDescription attribute : attributes) {
    if (visited_bindings[attribute.binding]) {
      continue;
    }
    visited_bindings[attribute.binding] = true;
    BLI_assert(vbos[attribute.binding]);
    VKVertexBuffer &vbo = *vbos[attribute.binding];
    vbo.upload();
    context.command_buffer_get().bind(attribute.binding, vbo, 0);
  }
}

void VKVertexAttributeObject::update_bindings(const VKContext &context, VKBatch &batch)
{
  const VKShaderInterface &interface = unwrap(context.shader)->interface_get();

  /* Reverse order so first VBO'S have more prevalence (in term of attribute override). */
  AttributeMask attribute_mask = interface.enabled_attr_mask_;

  for (int v = GPU_BATCH_VBO_MAX_LEN - 1; v > -1; v--) {
    VKVertexBuffer *vbo = batch.vertex_buffer_get(v);
    if (vbo) {
      attribute_mask &= ~update_bindings(*vbo, interface, false);
    }
  }

  for (int v = GPU_BATCH_INST_VBO_MAX_LEN - 1; v > -1; v--) {
    VKVertexBuffer *vbo = batch.instance_buffer_get(v);
    if (vbo) {
      attribute_mask &= ~update_bindings(*vbo, interface, true);
    }
  }
}

AttributeMask VKVertexAttributeObject::update_bindings(VKVertexBuffer &vertex_buffer,
                                                       const VKShaderInterface &interface,
                                                       const bool use_instancing)
{

  uint enabled_attributes = 0;
  const GPUVertFormat &format = vertex_buffer.format;

  if (format.attr_len <= 0) {
    return enabled_attributes;
  }

  uint32_t offset = 0;
  uint32_t stride = format.stride;
  const uint32_t binding = bindings.size();

  for (uint32_t attribute_index = 0; attribute_index < format.attr_len; attribute_index++) {
    const GPUVertAttr &attribute = format.attrs[attribute_index];
    if (format.deinterleaved) {
      offset += ((attribute_index == 0) ? 0 : format.attrs[attribute_index - 1].size) *
                vertex_buffer.vertex_len;
      stride = attribute.size;
    }
    else {
      offset = attribute.offset;
    }

    VkVertexInputBindingDescription vk_binding_descriptor = {};
    vk_binding_descriptor.binding = binding;
    vk_binding_descriptor.stride = stride;
    vk_binding_descriptor.inputRate = use_instancing ? VK_VERTEX_INPUT_RATE_INSTANCE :
                                                       VK_VERTEX_INPUT_RATE_VERTEX;
    bindings.append(vk_binding_descriptor);
    vbos.append(&vertex_buffer);

    for (uint32_t name_index = 0; name_index <= attribute.name_len; name_index++) {
      const char *name = GPU_vertformat_attr_name_get(&format, &attribute, name_index);
      const ShaderInput *shader_input = interface.attr_get(name);
      if (shader_input == nullptr || shader_input->location == -1) {
        continue;
      }

      VkVertexInputAttributeDescription attribute_description = {};
      enabled_attributes |= (1 << shader_input->location);
      attribute_description.binding = binding;
      attribute_description.location = shader_input->location;
      attribute_description.format = to_vk_format(
          static_cast<GPUVertCompType>(attribute.comp_type), attribute.size);
      attributes.append(attribute_description);
    }

    // TODO: resolving conflicting location numbers.
  }

  return enabled_attributes;
}

}  // namespace blender::gpu
