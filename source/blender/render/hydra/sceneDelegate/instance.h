/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_duplilist.h"

#include "id.h"

namespace blender::render::hydra {

class InstanceData: public IdData {
public:
  InstanceData(BlenderSceneDelegate *scene_delegate, DupliObject *dupli);

  int random_id();

  pxr::VtValue get_data(pxr::TfToken const &key) override;
  void insert_prim() override;
  void remove_prim() override;
  void mark_prim_dirty(DirtyBits dirty_bits) override;

public:
  DupliObject *dupli;
};

using InstanceDataMap = pxr::TfHashMap<pxr::SdfPath, std::unique_ptr<InstanceData>, pxr::SdfPath::Hash>;

} // namespace blender::render::hydra
