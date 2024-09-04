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

#include "DNA_material_types.h"
#include "DRW_render.hh"
#include "NOD_shader.h"
#include "gpu_shader_create_info.hh"

#pragma once

#define NPR_SCREEN_SPACE 0

namespace blender::eevee {

void NPRModule::init() {}

void NPRModule::begin_sync()
{
  indices_.clear();

#if NPR_SCREEN_SPACE
  surface_ps_.init();
  surface_ps_.state_set(DRW_STATE_WRITE_COLOR);
  surface_ps_.bind_texture(INDEX_NPR_TX_SLOT, &inst_.render_buffers.npr_index_tx);
  surface_ps_.bind_texture(DEPTH_NPR_TX_SLOT, &inst_.render_buffers.depth_tx);
#  if 0
  surface_ps_.bind_resources(inst_.gbuffer);
#  else
  /* Bind manually to pre-defined slots. */
  surface_ps_.bind_texture(GBUF_NORMAL_NPR_TX_SLOT, &inst_.gbuffer.normal_tx);
  surface_ps_.bind_texture(GBUF_HEADER_NPR_TX_SLOT, &inst_.gbuffer.header_tx);
  surface_ps_.bind_texture(GBUF_CLOSURE_NPR_TX_SLOT, &inst_.gbuffer.closure_tx);
#  endif
  for (int i : IndexRange(3)) {
    surface_ps_.bind_texture(DIRECT_RADIANCE_NPR_TX_SLOT_1 + i, &direct_radiance_txs_[i]);
    surface_ps_.bind_texture(INDIRECT_RADIANCE_NPR_TX_SLOT_1 + i, &indirect_radiance_txs_[i]);
  };
  surface_ps_.bind_resources(inst_.uniform_data);
#endif
}

static void codegen_callback(void * /*thunk*/, GPUMaterial * /*mat*/, GPUCodegenOutput *codegen)
{
  using namespace blender::gpu::shader;
  ShaderCreateInfo &info = *reinterpret_cast<ShaderCreateInfo *>(codegen->create_info);
  info.additional_info("npr_surface");

  std::stringstream frag_gen;
  frag_gen << (!codegen->material_functions.empty() ? codegen->material_functions : "\n");
  frag_gen << "vec4 nodetree_npr(){\n";
  frag_gen << (!codegen->npr.empty() ? codegen->npr : "return g_combined_color;\n");
  frag_gen << "}\n\n";

  info.fragment_source_generated = frag_gen.str();
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

#if NPR_SCREEN_SPACE
  /* TODO(NPR) */
  bool deferred_compilation = false;
  GPUMaterial *mat = DRW_shader_from_material(
      material, ntree, GPU_MAT_NPR, 0, false, deferred_compilation, codegen_callback, nullptr);

  if (!mat || GPU_material_status(mat) != GPU_MAT_SUCCESS) {
    return 0;
  }
#endif

  int index = indices_.size() + 1;
#if NPR_SCREEN_SPACE
  PassSimple::Sub &sub_ps = surface_ps_.sub(ntree->id.name);
  sub_ps.material_set(*inst_.manager, mat);
  sub_ps.push_constant("npr_index", index);
  sub_ps.push_constant("use_split_radiance", &use_split_radiance_);
  sub_ps.draw_procedural(GPU_PRIM_TRIS, 1, 3);
#endif
  indices_.add(ntree, index);
  return index;
}

void NPRModule::end_sync() {}

void NPRModule::render(View &view,
                       TextureFromPool direct_radiance_txs[3],
                       RayTraceResultTexture indirect_radiance_txs[3])
{
#if NPR_SCREEN_SPACE
  use_split_radiance_ = indirect_radiance_txs != nullptr;
  for (int i : IndexRange(3)) {
    direct_radiance_txs_[i] = direct_radiance_txs[i];
    if (indirect_radiance_txs) {
      indirect_radiance_txs_[i] = indirect_radiance_txs[i];
    }
  }

  surface_fb_.ensure(GPU_ATTACHMENT_NONE,
                     GPU_ATTACHMENT_TEXTURE(inst_.render_buffers.combined_tx));
  surface_fb_.bind();

  inst_.manager->submit(surface_ps_, view);

  for (int i : IndexRange(3)) {
    direct_radiance_txs_[i] = nullptr;
    indirect_radiance_txs_[i] = nullptr;
  }
#else
  UNUSED_VARS(view, direct_radiance_txs, indirect_radiance_txs);
#endif
}

}  // namespace blender::eevee
