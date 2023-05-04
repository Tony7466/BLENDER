/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_object.h"

#include "blender_scene_delegate.h"
#include "light.h"
#include "mesh.h"
#include "object.h"

namespace blender::render::hydra {

ObjectData::ObjectData(BlenderSceneDelegate *scene_delegate,
                       Object *object,
                       pxr::SdfPath const &prim_id)
    : IdData(scene_delegate, (ID *)object, prim_id), transform(pxr::GfMatrix4d(1.0))
{
}

std::unique_ptr<ObjectData> ObjectData::create(BlenderSceneDelegate *scene_delegate,
                                               Object *object,
                                               pxr::SdfPath const &prim_id)
{
  std::unique_ptr<ObjectData> data;

  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
      data = std::make_unique<MeshData>(scene_delegate, object, prim_id);
      break;

    case OB_LAMP:
      data = std::make_unique<LightData>(scene_delegate, object, prim_id);
      break;

    default:
      break;
  }
  return data;
}

bool ObjectData::is_supported(Object *object)
{
  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
    case OB_LAMP:
      return true;

    default:
      break;
  }
  return false;
}

bool ObjectData::update_visibility()
{
  if (!scene_delegate_->view3d) {
    return false;
  }

  bool prev_visible = visible;
  visible = BKE_object_is_visible_in_viewport(scene_delegate_->view3d, (Object *)id);
  bool ret = visible != prev_visible;
  if (ret) {
    ID_LOG(2, "");
  }
  return ret;
}

void ObjectData::update_parent()
{
  Object *object = (Object *)id;
  if (parent_ != object->parent) {
    ID_LOG(2, "");
    parent_ = object->parent;

    /* Looking for corresponded instancer and update it as parent */
    for (Object *ob = parent_; ob != nullptr; ob = ob->parent) {
      InstancerData *i_data = scene_delegate_->instancer_data(
          scene_delegate_->instancer_prim_id(ob));
      if (i_data) {
        i_data->update_as_parent();
        break;
      }
    }
  }
}

void ObjectData::write_transform()
{
  transform = gf_matrix_from_transform(((Object *)id)->object_to_world);
}

pxr::GfMatrix4d gf_matrix_from_transform(float m[4][4])
{
  pxr::GfMatrix4d ret;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      ret[i][j] = m[i][j];
    }
  }
  return ret;
}

}  // namespace blender::render::hydra
