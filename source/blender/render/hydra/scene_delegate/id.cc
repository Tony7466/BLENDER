/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_lib_id.h"

#include "id.h"

namespace blender::render::hydra {

IdData::IdData(BlenderSceneDelegate *scene_delegate, ID *id, pxr::SdfPath const &prim_id)
    : id(id), prim_id(prim_id), scene_delegate_(scene_delegate)
{
}

pxr::VtValue IdData::get_data(pxr::TfToken const & /*key*/) const
{
  return pxr::VtValue();
}

}  // namespace blender::render::hydra
