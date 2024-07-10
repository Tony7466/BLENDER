/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 */

/**
 * NPR TODOs:
 * - Fix hacky handling of NPR node shader weight (ntree_shader_weight_tree_invert).
 *   NPR socket type.
 * - Store eSpaceNode_ShaderFrom in the tree itself.
 */

#include "eevee_npr.hh"
#include "eevee_instance.hh"

#include "NPR/npr_defines.hh"

#include "DNA_material_types.h"
#include "DRW_render.hh"
#include "NOD_shader.h"
#include "gpu_shader_create_info.hh"

#pragma once

namespace blender::eevee {

void NPRModule::init() {}

void NPRModule::begin_sync()
{
  indices_.clear();

  surface_ps_.init();
  surface_ps_.state_set(DRW_STATE_WRITE_COLOR);
  surface_ps_.bind_texture(INDEX_TX_SLOT, &inst_.render_buffers.npr_index_tx);
  surface_ps_.bind_texture(DEPTH_TX_SLOT, &inst_.render_buffers.depth_tx);
  surface_ps_.bind_texture(NORMAL_TX_SLOT, &inst_.gbuffer.normal_tx);
  surface_ps_.bind_texture(COMBINED_TX_SLOT, &inst_.render_buffers.combined_tx);
}

static void codegen_callback(void * /*thunk*/, GPUMaterial * /*mat*/, GPUCodegenOutput *codegen)
{
  using namespace blender::gpu::shader;
  ShaderCreateInfo &info = *reinterpret_cast<ShaderCreateInfo *>(codegen->create_info);
  info.additional_info("npr_surface");
}

int NPRModule::sync_material(::Material *material)
{
  if (!material || !material->nodetree) {
    return 0;
  }

  bNodeTree *ntree = npr_tree_get(material);
  if (!ntree) {
    return 0;
  }

  if (indices_.contains(ntree)) {
    return indices_.lookup(ntree);
  }

  /* TODO(NPR) */
  bool deferred_compilation = false;
  GPUMaterial *mat = DRW_shader_from_material(
      material, ntree, GPU_MAT_NPR, 0, false, deferred_compilation, codegen_callback, nullptr);

  if (!mat || GPU_material_status(mat) != GPU_MAT_SUCCESS) {
    return 0;
  }

  int index = indices_.size() + 1;
  PassSimple::Sub &sub_ps = surface_ps_.sub(ntree->id.name);
  sub_ps.material_set(*inst_.manager, mat);
  sub_ps.push_constant("npr_index", index);
  sub_ps.draw_procedural(GPU_PRIM_TRIS, 1, 3);
  indices_.add(ntree, index);
  return index;
}

void NPRModule::end_sync() {}

void NPRModule::render(View &view)
{
  surface_tx_.acquire(inst_.render_buffers.extent_get(),
                      inst_.render_buffers.color_format,
                      GPU_TEXTURE_USAGE_ATTACHMENT);
  /* TODO(NPR): Optimize-out this copy. */
  GPU_texture_copy(surface_tx_, inst_.render_buffers.combined_tx);

  surface_fb_.ensure(GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(surface_tx_));
  surface_fb_.bind();

  inst_.manager->submit(surface_ps_, view);

  /* TODO(NPR): Optimize-out this copy. */
  GPU_texture_copy(inst_.render_buffers.combined_tx, surface_tx_);

  surface_tx_.release();
}

}  // namespace blender::eevee
