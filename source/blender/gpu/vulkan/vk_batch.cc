/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#include "vk_batch.hh"

#include "vk_context.hh"
#include "vk_index_buffer.hh"
#include "vk_vertex_buffer.hh"

namespace blender::gpu {

void VKBatch::draw(int v_first, int v_count, int i_first, int i_count)
{
  if (flag & GPU_BATCH_DIRTY) {
    vao_cache_.clear();
    flag &= ~GPU_BATCH_DIRTY;
  }

  /* Finalize graphics pipeline */
  VKContext &context = *VKContext::get();
  context.state_manager->apply_state();
  VKVertexAttributeObject &vao = vao_cache_.vao_get(this);
  vao.update_bindings(context, *this);
  context.bind_graphics_pipeline(prim_type, vao);

  /* Bind geometry resources. */
  vao.bind(context);
  VKIndexBuffer *index_buffer = index_buffer_get();
  const bool draw_indexed = index_buffer != nullptr;
  if (draw_indexed) {
    index_buffer->upload_data();
    index_buffer->bind(context);
    context.command_buffer_get().draw(
        index_buffer->index_len_get(), i_count, index_buffer->index_start_get(), v_first, i_first);
  }
  else {
    context.command_buffer_get().draw(v_first, v_count, i_first, i_count);
  }

  context.command_buffer_get().submit();
}

void VKBatch::draw_indirect(GPUStorageBuf * /*indirect_buf*/, intptr_t /*offset*/) {}

void VKBatch::multi_draw_indirect(GPUStorageBuf * /*indirect_buf*/,
                                  int /*count*/,
                                  intptr_t /*offset*/,
                                  intptr_t /*stride*/)
{
}

VKVertexBuffer *VKBatch::vertex_buffer_get(int index)
{
  return unwrap(verts_(index));
}

VKVertexBuffer *VKBatch::instance_buffer_get(int index)
{
  return unwrap(inst_(index));
}

VKIndexBuffer *VKBatch::index_buffer_get()
{
  return unwrap(unwrap(elem));
}

}  // namespace blender::gpu
