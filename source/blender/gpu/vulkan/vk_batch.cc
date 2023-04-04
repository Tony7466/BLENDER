/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation */

/** \file
 * \ingroup gpu
 */

#include "vk_batch.hh"

#include "vk_context.hh"

namespace blender::gpu {

void VKBatch::draw(int v_first, int v_count, int i_first, int i_count)
{
  // Based on the context construct the pipeline.
  VKContext &context = *VKContext::get();
  context.bind_graphics_pipeline();
  context.command_buffer_get().draw(v_first, v_count, i_first, i_count);
}

void VKBatch::draw_indirect(GPUStorageBuf * /*indirect_buf*/, intptr_t /*offset*/) {}

void VKBatch::multi_draw_indirect(GPUStorageBuf * /*indirect_buf*/,
                                  int /*count*/,
                                  intptr_t /*offset*/,
                                  intptr_t /*stride*/)
{
}

}  // namespace blender::gpu
