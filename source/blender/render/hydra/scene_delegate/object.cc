/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "DEG_depsgraph_query.h"

#include "blender_scene_delegate.h"
#include "curves.h"
#include "light.h"
#include "mesh.h"
#include "object.h"
#include "volume.h"

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
    case OB_CURVES:
      data = std::make_unique<CurvesData>(scene_delegate, object, prim_id);
      break;
    case OB_LAMP:
      data = std::make_unique<LightData>(scene_delegate, object, prim_id);
      break;
    case OB_VOLUME:
      data = std::make_unique<VolumeData>(scene_delegate, object, prim_id);
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
    case OB_CURVES:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
    case OB_LAMP:
    case OB_VOLUME:
      return true;

    default:
      break;
  }
  return false;
}

bool ObjectData::is_visible(BlenderSceneDelegate *scene_delegate, Object *object, int mode)
{
  eEvaluationMode deg_mode = DEG_get_mode(scene_delegate->depsgraph);
  bool ret = BKE_object_visibility(object, deg_mode) & mode;
  if (deg_mode == DAG_EVAL_VIEWPORT) {
    ret &= BKE_object_is_visible_in_viewport(scene_delegate->view3d, object);
  }
  /* Note: visibility for final render we are taking from depsgraph */
  return ret;
}

bool ObjectData::update_visibility()
{
  bool prev_visible = visible;
  visible = is_visible(scene_delegate_, (Object *)id);
  return visible != prev_visible;
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
