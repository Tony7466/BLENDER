/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/vt/value.h>
#include <pxr/base/tf/token.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "DNA_ID.h"

namespace blender::render::hydra {

class IdData {
 public:
  IdData(pxr::HdSceneDelegate *scene_delegate, ID *id);
  virtual ~IdData() = default;

  std::string name();
  virtual pxr::VtValue get_data(pxr::TfToken const &key);
  template<class T> const T get_data(pxr::TfToken const &key);

  enum class DirtyBits {
    DirtyTransform = 1,
    DirtyVisibility,
    DirtyMaterial,
    AllDirty
  };

  virtual void insert_prim() = 0;
  virtual void remove_prim() = 0;
  virtual void mark_prim_dirty(DirtyBits dirty_bits) = 0;

 protected:
  pxr::HdSceneDelegate *scene_delegate;
  ID *id;
};

template<class T> const T IdData::get_data(pxr::TfToken const &key)
{
  return get_data(key).Get<T>();
}

} // namespace blender::render::hydra
