/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 */

#pragma once

#include "BLI_map.hh"
#include "BLI_vector.hh"

struct bNodeTree;
struct GPUMaterial;
struct Material;

namespace blender::eevee {

class Instance;

class NPRModule {
 private:
  Instance &inst_;

  struct PassInfo {
    bNodeTree *ntree;
    GPUMaterial *gpu_mat;
  };
  Vector<PassInfo> passes_;
  Map<bNodeTree *, int> indices_;

 public:
  NPRModule(Instance &inst) : inst_(inst){};

  void init();
  int sync_material(::Material *material);
};

}  // namespace blender::eevee
