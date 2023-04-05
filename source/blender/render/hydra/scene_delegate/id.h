/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "DNA_ID.h"

namespace blender::render::hydra {

class BlenderSceneDelegate;

class IdData {
 public:
  IdData(BlenderSceneDelegate *scene_delegate, ID *id);
  virtual ~IdData() = default;

  virtual void init() = 0;
  virtual void insert() = 0;
  virtual void remove() = 0;
  virtual void update() = 0;

  virtual pxr::VtValue get_data(pxr::TfToken const &key) const;
  template<class T> const T get_data(pxr::TfToken const &key) const;

 protected:
  BlenderSceneDelegate *scene_delegate;
  ID *id;
  pxr::SdfPath p_id;
};

template<class T> const T IdData::get_data(pxr::TfToken const &key) const
{
  return get_data(key).Get<T>();
}

}  // namespace blender::render::hydra
