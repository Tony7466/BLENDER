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

/** Bind space for a single resource type. */
template<typename StorageType> class BindSpace {
 public:
  Vector<StorageType *> bound_resources;

  void bind(StorageType *resource, int binding)
  {
    if (bound_resources.size() <= binding) {
      bound_resources.resize(binding + 1);
    }
    bound_resources[binding] = resource;
  }

  StorageType *get(int binding)
  {
    return bound_resources[binding];
  }

  void unbind(void *resource)
  {
    for (int index : IndexRange(bound_resources.size())) {
      if (bound_resources[index] == resource) {
        bound_resources[index] = nullptr;
      }
    }
  }

  void unbind_all()
  {
    bound_resources.clear();
  }
};

/**
 * Bind space for resource classes that are offsetted.
 */
template<typename StorageType, int Offset> class BindSpaceOffset {
 public:
  Vector<StorageType *> bound_resources;

  void bind(StorageType *resource, int binding)
  {
    if (binding >= Offset) {
      binding -= Offset;
    }
    if (bound_resources.size() <= binding) {
      bound_resources.resize(binding + 1);
    }
    bound_resources[binding] = resource;
  }

  StorageType *get(int binding)
  {
    if (binding >= Offset) {
      binding -= Offset;
    }
    return bound_resources[binding];
  }

  void unbind(void *resource)
  {
    for (int index : IndexRange(bound_resources.size())) {
      if (bound_resources[index] == resource) {
        bound_resources[index] = nullptr;
      }
    }
  }

  void unbind_all()
  {
    bound_resources.clear();
  }
};

/** Bind space where different type of resources are used. */
class BindSpaceTyped {
 public:
  enum class Type {
    Unused,
    UniformBuffer,
    VertexBuffer,
    IndexBuffer,
    StorageBuffer,
  };
  struct Elem {
    Type resource_type;
    void *resource;
  };
  Vector<Elem> bound_resources;

  void bind(Type resource_type, void *resource, int binding)
  {
    if (bound_resources.size() <= binding) {
      bound_resources.resize(binding + 1);
    }
    bound_resources[binding].resource_type = resource_type;
    bound_resources[binding].resource = resource;
  }

  const Elem &get(int binding) const
  {
    return bound_resources[binding];
  }

  void unbind(void *resource)
  {
    for (int index : IndexRange(bound_resources.size())) {
      if (bound_resources[index].resource == resource) {
        bound_resources[index].resource = nullptr;
        bound_resources[index].resource_type = Type::Unused;
      }
    }
  }

  void unbind_all()
  {
    bound_resources.clear();
  }
};

class BindSpaceTypedSampled {
 public:
  enum class Type {
    Unused,
    VertexBuffer,
    Texture,
  };
  struct Elem {
    Type resource_type;
    void *resource;
    GPUSamplerState sampler;
  };
  Vector<Elem> bound_resources;

  void bind(Type resource_type, void *resource, GPUSamplerState sampler, int binding)
  {
    if (bound_resources.size() <= binding) {
      bound_resources.resize(binding + 1);
    }
    bound_resources[binding].resource_type = resource_type;
    bound_resources[binding].resource = resource;
    bound_resources[binding].sampler = sampler;
  }

  const Elem &get(int binding) const
  {
    return bound_resources[binding];
  }

  void unbind(void *resource)
  {
    for (int index : IndexRange(bound_resources.size())) {
      if (bound_resources[index].resource == resource) {
        bound_resources[index].resource = nullptr;
        bound_resources[index].resource_type = Type::Unused;
        bound_resources[index].sampler = GPUSamplerState::default_sampler();
      }
    }
  }

  void unbind_all()
  {
    bound_resources.clear();
  }
};

class VKStateManager : public StateManager {
  friend class VKDescriptorSetTracker;

  uint texture_unpack_row_length_ = 0;

  BindSpaceTypedSampled textures_;
  BindSpaceOffset<VKTexture, BIND_SPACE_IMAGE_OFFSET> images_;
  BindSpace<VKUniformBuffer> uniform_buffers_;
  BindSpaceTyped storage_buffers_;

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

  void storage_buffer_bind(BindSpaceTyped::Type resource_type, void *resource, int binding);
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
