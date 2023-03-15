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
  static std::unique_ptr<WorldData> init(BlenderSceneDelegate *scene_delegate,
                                         World *world,
                                         bContext *context);
  static pxr::SdfPath prim_id(BlenderSceneDelegate *scene_delegate);

  WorldData(BlenderSceneDelegate *scene_delegate, World *world, bContext *context);

  pxr::GfMatrix4d transform();

  pxr::VtValue get_data(pxr::TfToken const &key) override;
  void insert_prim() override;
  void remove_prim() override;
  void mark_prim_dirty(DirtyBits dirty_bits) override;

 private:
  std::map<pxr::TfToken, pxr::VtValue> data;
};

}  // namespace blender::render::hydra
