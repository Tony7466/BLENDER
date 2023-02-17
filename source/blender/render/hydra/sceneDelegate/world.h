/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <map>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/usd/sdf/assetPath.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/base/vt/value.h>
#include "pxr/base/tf/staticTokens.h"

#include "DNA_view3d_types.h"
#include "DNA_world_types.h"

namespace blender::render::hydra {

class WorldData {
public:
  WorldData();
  WorldData(World *world, bContext *b_context);

  pxr::TfToken prim_type();
  pxr::GfMatrix4d transform(std::string const &renderer_name);

  pxr::VtValue &get_data(pxr::TfToken const &key);
  template<class T>
  const T &get_data(pxr::TfToken const &key);
  bool has_data(pxr::TfToken const &key);
  bool is_visible();

  bContext *b_context;
  World *world;
  
 private:
  std::map<pxr::TfToken, pxr::VtValue> data;
};

template<class T>
const T &WorldData::get_data(pxr::TfToken const &key)
{
  return get_data(key).Get<T>();
}

} // namespace blender::render::hydra
