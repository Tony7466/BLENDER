/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>

#include "BLI_hash.hh"
#include "DNA_ID.h"

template<> struct blender::DefaultHash<pxr::SdfPath> {
  uint64_t operator()(const pxr::SdfPath &value) const
  {
    return pxr::SdfPath::Hash()(value);
  }
};

template<> struct blender::DefaultHash<pxr::TfToken> {
  uint64_t operator()(const pxr::TfToken &value) const
  {
    return pxr::TfHash()(value);
  }
};

namespace blender::render::hydra {

class BlenderSceneDelegate;

class IdData {
 public:
  IdData(BlenderSceneDelegate *scene_delegate, ID *id, pxr::SdfPath const &prim_id);
  virtual ~IdData() = default;

  virtual void init() = 0;
  virtual void insert() = 0;
  virtual void remove() = 0;
  virtual void update() = 0;

  virtual pxr::VtValue get_data(pxr::TfToken const &key) const;
  template<class T> const T get_data(pxr::TfToken const &key) const;

  ID *id;
  pxr::SdfPath prim_id;

 protected:
  BlenderSceneDelegate *scene_delegate_;
};

template<class T> const T IdData::get_data(pxr::TfToken const &key) const
{
  return get_data(key).Get<T>();
}

#define ID_LOG(level, msg, ...) \
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, \
            level, \
            "%s (%s): " msg, \
            prim_id.GetText(), \
            id ? id->name : "", \
            ##__VA_ARGS__);

}  // namespace blender::render::hydra
