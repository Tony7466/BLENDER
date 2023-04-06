/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_batch_private.hh"

#include "vk_context.hh"
#include "vk_vertex_attribute_object.hh"

namespace blender::gpu {
const uint32_t VK_GPU_VAO_STATIC_LEN = 3;

/**
 * VAO management: remembers all geometry state (vertex attribute bindings & element buffer)
 * for each shader interface. Start with a static number of VAO's and fallback to dynamic count
 * if necessary. Once a batch goes dynamic it does not go back.
 */
class VKVaoCache {

 private:
  /** Context for which the vao_cache_ was generated. */
  VKContext *context_ = nullptr;

  /* TODO: why are these needed? */
  /** Last interface this batch was drawn with. */
  VKShaderInterface *interface_ = nullptr;
  /** Cached VAO for the last interface. */
  VKVertexAttributeObject vao_id_ = {};

  int base_instance_ = 0;

  bool is_dynamic_vao_count = false;
  union {
    /** Static handle count */
    struct {
      const VKShaderInterface *interfaces[VK_GPU_VAO_STATIC_LEN];
      VKVertexAttributeObject vao_ids[VK_GPU_VAO_STATIC_LEN];
    } static_vaos;
    /** Dynamic handle count */
    struct {
      uint32_t count;
      const VKShaderInterface **interfaces;
      VKVertexAttributeObject *vao_ids;
    } dynamic_vaos;
  };

 public:
  bool is_dirty = false;
  VKVaoCache();
  ~VKVaoCache();

  VKVertexAttributeObject &vao_get(GPUBatch *batch);

  /**
   * Return nullptr on cache miss (invalid VAO).
   */
  VKVertexAttributeObject &lookup(const VKShaderInterface *interface);
  /**
   * Create a new VAO object and store it in the cache.
   */
  void insert(const VKShaderInterface *interface, VKVertexAttributeObject &vao_id);
  void remove(const VKShaderInterface *interface);
  void clear();

 private:
  void init();
};

}  // namespace blender::gpu