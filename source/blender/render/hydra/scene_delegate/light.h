/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "pxr/base/tf/hashmap.h"
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include "object.h"

namespace blender::render::hydra {

class LightData : public ObjectData {
 public:
  LightData(BlenderSceneDelegate *scene_delegate, Object *object);

  pxr::VtValue get_data(pxr::TfToken const &key) override;
  void insert_prim() override;
  void remove_prim() override;
  void mark_prim_dirty(DirtyBits dirty_bits) override;

 private:
  std::map<pxr::TfToken, pxr::VtValue> data;
  pxr::TfToken prim_type();
};

}  // namespace blender::render::hydra
