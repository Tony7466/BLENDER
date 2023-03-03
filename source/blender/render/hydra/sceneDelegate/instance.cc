/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "blenderSceneDelegate.h"
#include "instance.h"

using namespace pxr;

namespace blender::render::hydra {

InstanceData::InstanceData(BlenderSceneDelegate *scene_delegate, DupliObject *dupli)
  : IdData(scene_delegate, (ID *)dupli->ob_data)
  , dupli(dupli)
{
}

int InstanceData::random_id()
{
  return dupli->random_id;
}

VtValue InstanceData::get_data(TfToken const &key)
{
  return VtValue();
}

void InstanceData::insert_prim()
{
}

void InstanceData::remove_prim()
{
}

void InstanceData::mark_prim_dirty(DirtyBits dirty_bits)
{
}

} // namespace blender::render::hydra
