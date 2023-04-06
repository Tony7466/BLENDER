/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_batch_private.hh"

#include "vk_vertex_attribute_object_cache.hh"

namespace blender::gpu {

class VKBatch : public Batch {
 private:
  VKVaoCache vao_cache_;

 public:
  void draw(int v_first, int v_count, int i_first, int i_count) override;
  void draw_indirect(GPUStorageBuf *indirect_buf, intptr_t offset) override;
  void multi_draw_indirect(GPUStorageBuf *indirect_buf,
                           int count,
                           intptr_t offset,
                           intptr_t stride) override;

  VKVertexBuffer *vertex_buffer_get(int index);
  VKVertexBuffer *instance_buffer_get(int index);
  VKIndexBuffer *index_buffer_get();
};

}  // namespace blender::gpu
