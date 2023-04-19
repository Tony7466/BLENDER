/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>
#include <pxr/imaging/hd/sceneDelegate.h>

#include "DNA_ID.h"

namespace blender::render::hydra {

class BlenderSceneDelegate;
class InstancerData;

class IdData {
  friend InstancerData;

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
  BlenderSceneDelegate *scene_delegate_;
  ID *id_;
  pxr::SdfPath p_id_;
};

template<class T> const T IdData::get_data(pxr::TfToken const &key) const
{
  return get_data(key).Get<T>();
}

#define ID_LOG(level, msg, ...) \
  CLOG_INFO(LOG_BSD, level, "%s (%s): " msg, p_id_.GetText(), id_->name, __VA_ARGS__);

}  // namespace blender::render::hydra
