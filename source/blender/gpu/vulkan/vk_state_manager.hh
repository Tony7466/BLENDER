/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_state_private.hh"

#include "BLI_array.hh"

#include "render_graph/vk_resource_access_info.hh"
#include "vk_bindable_resource.hh"

namespace blender::gpu {
class VKTexture;
class VKUniformBuffer;
class VKVertexBuffer;
class VKStorageBuffer;
class VKIndexBuffer;
class VKContext;
class VKDescriptorSetTracker;

class VKStateManager : public StateManager {
  friend class VKDescriptorSetTracker;

  class BindSpaceUniformBuffers {
   public:
    Vector<VKUniformBuffer *> bound_resources;

    void bind(VKUniformBuffer *uniform_buffer, int binding)
    {
      if (bound_resources.size() <= binding) {
        bound_resources.resize(binding + 1);
      }
      bound_resources[binding] = uniform_buffer;
    }

    VKUniformBuffer *get(int binding)
    {
      return bound_resources[binding];
    }

    void unbind(void *uniform_buffer)
    {
      for (int index : IndexRange(bound_resources.size())) {
        if (bound_resources[index] == uniform_buffer) {
          bound_resources[index] = nullptr;
        }
      }
    }

    void unbind_all()
    {
      bound_resources.clear();
    }
  };

  uint texture_unpack_row_length_ = 0;

  VKBindSpace<shader::ShaderCreateInfo::Resource::BindType::SAMPLER> textures_;
  VKBindSpace<shader::ShaderCreateInfo::Resource::BindType::IMAGE, BIND_SPACE_IMAGE_OFFSET>
      images_;
  BindSpaceUniformBuffers uniform_buffers_;
  VKBindSpace<shader::ShaderCreateInfo::Resource::BindType::STORAGE_BUFFER> storage_buffers_;

 public:
  bool is_dirty = false;

  void apply_state() override;
  void force_state() override;

  void issue_barrier(eGPUBarrier barrier_bits) override;

  /** Apply resources to the bindings of the active shader. */
  void apply_bindings(VKContext &context,
                      render_graph::VKResourceAccessInfo &resource_access_info);

  void texture_bind(Texture *tex, GPUSamplerState sampler, int unit) override;
  void texture_unbind(Texture *tex) override;
  void texture_unbind_all() override;

  void image_bind(Texture *tex, int unit) override;
  void image_unbind(Texture *tex) override;
  void image_unbind_all() override;

  void uniform_buffer_bind(VKUniformBuffer *uniform_buffer, int slot);
  void uniform_buffer_unbind(VKUniformBuffer *uniform_buffer);
  void uniform_buffer_unbind_all();

  void texel_buffer_bind(VKVertexBuffer &vertex_buffer, int slot);
  void texel_buffer_unbind(VKVertexBuffer &vertex_buffer);

  void storage_buffer_bind(VKBindableResource &resource, int slot);
  void storage_buffer_unbind(VKBindableResource &resource);
  void storage_buffer_unbind_all();

  void unbind_from_all_namespaces(VKBindableResource &bindable_resource);

  void texture_unpack_row_length_set(uint len) override;

  /**
   * Row length for unpacking host data when uploading texture data.
   *
   * When set to zero (0) host data can be assumed to be stored sequential.
   */
  uint texture_unpack_row_length_get() const;
};
}  // namespace blender::gpu
