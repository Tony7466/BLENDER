/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_lib_id.h"

#include "blenderSceneDelegate.h"
#include "id.h"

using namespace pxr;

namespace blender::render::hydra {

IdData::IdData(BlenderSceneDelegate *scene_delegate, ID *id)
  : scene_delegate(scene_delegate)
  , id(id)
{
}

std::string IdData::name()
{
  char str[MAX_ID_FULL_NAME];
  BKE_id_full_name_get(str, id, 0);
  return str;
}

VtValue IdData::get_data(TfToken const &key)
{
  return VtValue();
}

} // namespace blender::render::hydra
