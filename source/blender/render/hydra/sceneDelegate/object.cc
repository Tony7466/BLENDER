/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "BKE_object.h"

#include "blenderSceneDelegate.h"
#include "object.h"
#include "mesh.h"
#include "light.h"
#include "../utils.h"

using namespace pxr;

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

std::unique_ptr<ObjectData> ObjectData::init(BlenderSceneDelegate *scene_delegate, Object *object)
{
  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
      return std::make_unique<MeshData>(scene_delegate, object);

    case OB_LAMP:
      return std::make_unique<LightData>(scene_delegate, object);

    default:
      break;
  }
  return nullptr;
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
  : IdData(scene_delegate, (ID *)object)
  , visible(true)
{
}

int ObjectData::type()
{
  return ((Object *)id)->type;
}

GfMatrix4d ObjectData::transform()
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
