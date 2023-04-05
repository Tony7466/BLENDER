/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_object.h"

#include "../utils.h"
#include "blender_scene_delegate.h"
#include "light.h"
#include "mesh.h"
#include "object.h"

namespace blender::render::hydra {

bool ObjectData::supported(Object *object)
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

std::unique_ptr<ObjectData> ObjectData::create(BlenderSceneDelegate *scene_delegate,
                                               Object *object)
{
  std::unique_ptr<ObjectData> data;

  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
      data = std::make_unique<MeshData>(scene_delegate, object);
      break;

    case OB_LAMP:
      data = std::make_unique<LightData>(scene_delegate, object);
      break;

    default:
      break;
  }
  if (data) {
    data->init();
    data->insert();
  }
  return data;
}

pxr::SdfPath ObjectData::prim_id(BlenderSceneDelegate *scene_delegate, Object *object)
{
  /* Making id of object in form like O_<pointer in 16 hex digits format>. Example:
   * O_000002073e369608 */
  char str[32];
  snprintf(str, 32, "O_%016llx", (uint64_t)object);
  return scene_delegate->GetDelegateID().AppendElementString(str);
}

ObjectData::ObjectData(BlenderSceneDelegate *scene_delegate, Object *object)
    : IdData(scene_delegate, (ID *)object), visible(true)
{
  p_id = prim_id(scene_delegate, object);
}

pxr::GfMatrix4d ObjectData::transform()
{
  return gf_matrix_from_transform(((Object *)id)->object_to_world);
}

bool ObjectData::update_visibility(View3D *view3d)
{
  if (!view3d) {
    return false;
  }

  bool prev_visible = visible;
  visible = BKE_object_is_visible_in_viewport(view3d, (Object *)id);
  return visible != prev_visible;
}

}  // namespace blender::render::hydra
