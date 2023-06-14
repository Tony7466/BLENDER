/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#include "vk_state_manager.hh"
#include "vk_context.hh"
#include "vk_index_buffer.hh"
#include "vk_pipeline.hh"
#include "vk_shader.hh"
#include "vk_storage_buffer.hh"
#include "vk_texture.hh"
#include "vk_vertex_buffer.hh"

#include "GPU_capabilities.h"

namespace blender::gpu {

VKStateManager::VKStateManager()
{
  sampler_.create();
  constexpr int max_bindings = 16;
  texture_bindings_ = Array<TextureBinding>(max_bindings);
  texture_bindings_.fill(TextureBinding());
}

void VKStateManager::apply_state()
{
  VKContext &context = *VKContext::get();
  if (context.shader) {
    VKShader &shader = unwrap(*context.shader);
    VKPipeline &pipeline = shader.pipeline_get();
    pipeline.state_manager_get().set_state(state, mutable_state);
  }
}

void VKStateManager::apply_bindings()
{
  VKContext &context = *VKContext::get();
  if (context.shader) {

    for (int binding : IndexRange(texture_bindings_.size())) {
      if (texture_bindings_[binding].texture) {
        texture_bindings_[binding].texture->bind(binding, sampler_);
      }
      else if (texture_bindings_[binding].vertex_buffer) {
        texture_bindings_[binding].vertex_buffer->bind(
            binding, shader::ShaderCreateInfo::Resource::BindType::SAMPLER);
      }
    }

    image_bindings_.apply_bindings();
    uniform_buffer_bindings_.apply_bindings();
    storage_buffer_bindings_.apply_bindings();
  }
}

void VKStateManager::force_state()
{
  VKContext &context = *VKContext::get();
  BLI_assert(context.shader);
  VKShader &shader = unwrap(*context.shader);
  VKPipeline &pipeline = shader.pipeline_get();
  pipeline.state_manager_get().force_state(state, mutable_state);
}

void VKStateManager::issue_barrier(eGPUBarrier /*barrier_bits*/)
{
  VKContext &context = *VKContext::get();
  VKCommandBuffer &command_buffer = context.command_buffer_get();
  /* TODO: Pipeline barriers should be added. We might be able to extract it from
   * the actual pipeline, later on, but for now we submit the work as barrier. */
  command_buffer.submit();
}

void VKStateManager::texture_bind(Texture *tex, GPUSamplerState /*sampler*/, int unit)
{
  VKTexture *texture = unwrap(tex);
  texture_bindings_[unit].texture = texture;
  texture_bindings_[unit].vertex_buffer = nullptr;
}

void VKStateManager::texture_unbind(Texture *tex)
{
  VKTexture *texture = unwrap(tex);
  for (TextureBinding &binding : texture_bindings_) {
    if (binding.texture == texture) {
      binding.texture = nullptr;
    }
  }
}

void VKStateManager::texture_unbind_all()
{
  for (TextureBinding &binding : texture_bindings_) {
    binding.texture = nullptr;
  }
}

void VKStateManager::image_bind(Texture *tex, int binding)
{
  VKTexture *texture = unwrap(tex);
  image_bindings_.bind(binding, *texture);
}

void VKStateManager::image_unbind(Texture *tex)
{
  VKTexture *texture = unwrap(tex);
  image_bindings_.unbind(*texture);
}

void VKStateManager::image_unbind_all()
{
  image_bindings_.unbind_all();
}

void VKStateManager::uniform_buffer_bind(VKUniformBuffer *uniform_buffer, int slot)
{
  uniform_buffer_bindings_.bind(slot, *uniform_buffer);
}

void VKStateManager::uniform_buffer_unbind(VKUniformBuffer *uniform_buffer)
{
  uniform_buffer_bindings_.unbind(*uniform_buffer);
}

void VKStateManager::unbind_from_all_namespaces(VKBindableResource &resource)
{
  uniform_buffer_bindings_.unbind(resource);
  storage_buffer_bindings_.unbind(resource);
  image_bindings_.unbind(resource);
}

void VKStateManager::texel_buffer_bind(VKVertexBuffer *vertex_buffer, int slot)
{
  texture_bindings_[slot].vertex_buffer = vertex_buffer;
  texture_bindings_[slot].texture = nullptr;
}

void VKStateManager::texel_buffer_unbind(VKVertexBuffer *vertex_buffer)
{
  for (TextureBinding &binding : texture_bindings_) {
    if (binding.vertex_buffer == vertex_buffer) {
      binding.vertex_buffer = nullptr;
      binding.texture = nullptr;
    }
  }
}

void VKStateManager::storage_buffer_bind(VKBindableResource &resource, int slot)
{
  storage_buffer_bindings_.bind(slot, resource);
}

void VKStateManager::storage_buffer_unbind(VKBindableResource &resource)
{
  storage_buffer_bindings_.unbind(resource);
}

void VKStateManager::texture_unpack_row_length_set(uint len)
{
  texture_unpack_row_length_ = len;
}

uint VKStateManager::texture_unpack_row_length_get() const
{
  return texture_unpack_row_length_;
}

}  // namespace blender::gpu
