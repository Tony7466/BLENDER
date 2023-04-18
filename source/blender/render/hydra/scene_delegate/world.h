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
  WorldData(BlenderSceneDelegate *scene_delegate, World *world, bContext *context);

  static std::unique_ptr<WorldData> create(BlenderSceneDelegate *scene_delegate,
                                           World *world,
                                           bContext *context);
  static pxr::SdfPath prim_id(BlenderSceneDelegate *scene_delegate);

  void init() override;
  void insert() override;
  void remove() override;
  void update() override;
  void update(World *world);

  pxr::GfMatrix4d transform();
  pxr::VtValue get_data(pxr::TfToken const &key) const override;

 private:
  std::map<pxr::TfToken, pxr::VtValue> data_;
  bContext *context_;
};

}  // namespace blender::render::hydra
