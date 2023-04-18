/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_lib_id.h"

#include "blender_scene_delegate.h"
#include "id.h"

namespace blender::render::hydra {

IdData::IdData(BlenderSceneDelegate *scene_delegate, ID *id)
    : scene_delegate_(scene_delegate), id_(id)
{
}

pxr::VtValue IdData::get_data(pxr::TfToken const &key) const
{
  return pxr::VtValue();
}

}  // namespace blender::render::hydra
