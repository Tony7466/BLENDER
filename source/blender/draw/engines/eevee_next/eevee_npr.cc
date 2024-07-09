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

#include "DNA_material_types.h"
#include "DRW_render.hh"
#include "NOD_shader.h"
#include "gpu_shader_create_info.hh"

#pragma once

namespace blender::eevee {

void NPRModule::init()
{
  passes_.clear();
  indices_.clear();

  /* Append an empty pass for materials without a NPR pass. */
  passes_.append({});
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

  int index = passes_.size();
  passes_.append({ntree, mat});
  indices_.add(ntree, index);
  return index;
}

}  // namespace blender::eevee
