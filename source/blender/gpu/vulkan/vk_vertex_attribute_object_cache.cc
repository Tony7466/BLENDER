/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

#include "vk_vertex_attribute_object_cache.hh"

#include "vk_shader_interface.hh"

#include "BLI_span.hh"

namespace blender::gpu {

VKVaoCache::VKVaoCache()
{
  init();
}

VKVaoCache::~VKVaoCache()
{
  this->clear();
}

void VKVaoCache::init()
{
  context_ = VKContext::get();
  interface_ = nullptr;
  is_dynamic_vao_count = false;
  for (int i = 0; i < VK_GPU_VAO_STATIC_LEN; i++) {
    static_vaos.interfaces[i] = nullptr;
    static_vaos.vao_ids[i].is_valid = false;
  }
  base_instance_ = 0;
  vao_id_.is_valid = false;
}

void VKVaoCache::insert(const VKShaderInterface *interface, VKVertexAttributeObject &vao)
{

  BLI_assert(vao.is_valid);

  /* Now insert the cache. */
  if (!is_dynamic_vao_count) {
    int i; /* find first unused slot */
    for (i = 0; i < VK_GPU_VAO_STATIC_LEN; i++) {
      if (static_vaos.vao_ids[i].is_valid == false) {
        break;
      }
    }

    if (i < VK_GPU_VAO_STATIC_LEN) {
      static_vaos.interfaces[i] = interface;
      static_vaos.vao_ids[i] = vao;
    }
    else {
      /* Erase previous entries, they will be added back if drawn again. */
      for (int i = 0; i < VK_GPU_VAO_STATIC_LEN; i++) {
        if (static_vaos.interfaces[i] != nullptr) {
          static_vaos.vao_ids[i].clear();
          /*  context_->vao_free(static_vaos.vao_ids[i]); */
        }
      }
      /* Not enough place switch to dynamic. */
      is_dynamic_vao_count = true;
      /* Init dynamic arrays and let the branch below set the values. */
      dynamic_vaos.count = GPU_BATCH_VAO_DYN_ALLOC_COUNT;
      dynamic_vaos.interfaces = (const VKShaderInterface **)MEM_callocN(
          dynamic_vaos.count * sizeof(VKShaderInterface *), "dyn vaos interfaces");
      dynamic_vaos.vao_ids = (VKVertexAttributeObject *)MEM_callocN(
          dynamic_vaos.count * sizeof(VKVertexAttributeObject), "dyn vaos ids");
      for (int j = 0; j < dynamic_vaos.count; j++) {
        dynamic_vaos.interfaces[j] = nullptr;
      }
    }
  }

  if (is_dynamic_vao_count) {
    int i; /* find first unused slot */
    for (i = 0; i < dynamic_vaos.count; i++) {
      if (dynamic_vaos.vao_ids[i].is_valid == false) {
        break;
      }
    }

    if (i == dynamic_vaos.count) {
      /* Not enough place, realloc the array. */
      i = dynamic_vaos.count;
      dynamic_vaos.count += GPU_BATCH_VAO_DYN_ALLOC_COUNT;
      dynamic_vaos.interfaces = (const VKShaderInterface **)MEM_recallocN(
          (void *)dynamic_vaos.interfaces, sizeof(VKShaderInterface *) * dynamic_vaos.count);
      dynamic_vaos.vao_ids = (VKVertexAttributeObject *)MEM_recallocN(
          dynamic_vaos.vao_ids, sizeof(VKVertexAttributeObject) * dynamic_vaos.count);
      for (int j = 0; j < GPU_BATCH_VAO_DYN_ALLOC_COUNT; j++) {
        dynamic_vaos.interfaces[dynamic_vaos.count - j - 1] = nullptr;
      }
    }

    dynamic_vaos.interfaces[i] = interface;
    dynamic_vaos.vao_ids[i] = vao;
  }
}

void VKVaoCache::remove(const VKShaderInterface *interface)
{
  const int count = (is_dynamic_vao_count) ? dynamic_vaos.count : VK_GPU_VAO_STATIC_LEN;
  VKVertexAttributeObject *vaos = (is_dynamic_vao_count) ? dynamic_vaos.vao_ids :
                                                           static_vaos.vao_ids;

  const VKShaderInterface **interfaces = (is_dynamic_vao_count) ? dynamic_vaos.interfaces :
                                                                  static_vaos.interfaces;
  for (int i = 0; i < count; i++) {
    if (interfaces[i] == interface) {
      vaos[i].clear();
      interfaces[i] = nullptr;
      break; /* cannot have duplicates */
    }
  }
}

void VKVaoCache::clear()
{

  const int count = (is_dynamic_vao_count) ? dynamic_vaos.count : VK_GPU_VAO_STATIC_LEN;
  VKVertexAttributeObject *vaos = (is_dynamic_vao_count) ? dynamic_vaos.vao_ids :
                                                           static_vaos.vao_ids;
  /* Early out, nothing to free. */
  if (context_ == nullptr) {
    return;
  }

  /* TODO(fclem): Slow way. Could avoid multiple mutex lock here */
  for (int i = 0; i < count; i++) {
    vaos[i].clear();
    // context_->vao_free(vaos[i]);
  }

  if (is_dynamic_vao_count) {
    MEM_freeN((void *)dynamic_vaos.interfaces);
    MEM_freeN((void *)dynamic_vaos.vao_ids);
  }

  /* Reinit. */
  this->init();
}

static VKVertexAttributeObject &lookup_interface(int64_t element_len,
                                                 const VKShaderInterface **interfaces,
                                                 VKVertexAttributeObject *vaos,
                                                 const VKShaderInterface *interface)
{
  for (int i = 0; i < element_len; i++) {
    if (interfaces[i] == interface) {
      return vaos[i];
    }
  }

  static VKVertexAttributeObject vao = {};
  vao.is_valid = false;
  return vao;
}

VKVertexAttributeObject &VKVaoCache::lookup(const VKShaderInterface *interface)
{
  if (is_dynamic_vao_count) {
    return lookup_interface(
        dynamic_vaos.count, dynamic_vaos.interfaces, dynamic_vaos.vao_ids, interface);
  }
  else {
    return lookup_interface(
        VK_GPU_VAO_STATIC_LEN, static_vaos.interfaces, static_vaos.vao_ids, interface);
  }
}

VKVertexAttributeObject &VKVaoCache::vao_get(GPUBatch * /*batch*/)
{
  Shader *shader = VKContext::get()->shader;
  VKShaderInterface *interface = static_cast<VKShaderInterface *>(shader->interface);
  if (interface_ != interface) {
    interface_ = interface;
  };

  vao_id_ = this->lookup(interface_);

  if (!vao_id_.is_valid) {
    vao_id_.is_valid = true;
    this->insert(interface_, vao_id_);
  }

  vao_id_.clear();
  vao_id_.is_valid = true;

  return vao_id_;
}

}  // namespace blender::gpu
