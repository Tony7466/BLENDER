/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <map>

#include "pxr/base/tf/staticTokens.h"
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>

#include "DNA_view3d_types.h"
#include "DNA_world_types.h"

#include "id.h"

namespace blender::render::hydra {

class WorldData : public IdData {
 public:
  WorldData(BlenderSceneDelegate *scene_delegate, pxr::SdfPath const &prim_id);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;
  void update(World *world);

  pxr::VtValue get_data(pxr::TfToken const &key) const override;

  pxr::GfMatrix4d transform;

 private:
  void write_transform();

  std::map<pxr::TfToken, pxr::VtValue> data_;
};

}  // namespace blender::render::hydra
