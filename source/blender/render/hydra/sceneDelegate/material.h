/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include "pxr/base/tf/hashmap.h"

#include "DNA_material_types.h"

#include "id.h"

namespace blender::render::hydra {

class MaterialData;
using MaterialDataMap = pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<MaterialData>, pxr::SdfPath::Hash>;

class MaterialData: IdData {
 public:
  static std::unique_ptr<MaterialData> init(pxr::HdSceneDelegate *scene_delegate, Material *material);
  static pxr::SdfPath prim_id(pxr::HdSceneDelegate *scene_delegate, Material *material);

  MaterialData(pxr::HdSceneDelegate *scene_delegate, Material *material);

  pxr::VtValue get_data(pxr::TfToken const &key) override;
  void insert_prim() override;
  void remove_prim() override;
  void mark_prim_dirty(DirtyBits dirty_bits) override;

  pxr::VtValue material_resource();
  void export_mtlx();

private:
  pxr::SdfAssetPath mtlx_path;
};

} // namespace blender::render::hydra
