/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>

#include "../utils.h"
#include "blender_scene_delegate.h"
#include "instancer.h"

namespace blender::render::hydra {

bool InstancerData::supported(Object *object)
{
  switch (object->type) {
    case OB_MESH:
    case OB_SURF:
    case OB_FONT:
    case OB_CURVES_LEGACY:
    case OB_MBALL:
      return true;

    default:
      break;
  }
  return false;
}

std::unique_ptr<InstancerData> InstancerData::create(BlenderSceneDelegate *scene_delegate,
                                                     Object *object)
{
  auto data = std::make_unique<InstancerData>(scene_delegate, object);
  data->init();
  data->insert();
  return data;
}

pxr::SdfPath InstancerData::prim_id(BlenderSceneDelegate *scene_delegate, Object *object)
{
  /* Making id of instancer in form like I_<pointer in 16 hex digits format>. Example:
   * I_000002073e369608 */
  char str[32];
  snprintf(str, 32, "I_%016llx", (uint64_t)object);
  return scene_delegate->GetDelegateID().AppendElementString(str);
}

InstancerData::InstancerData(BlenderSceneDelegate *scene_delegate, Object *object)
    : MeshData(scene_delegate, object), parent_obj(object)
{
  id = nullptr;
  p_id = prim_id(scene_delegate, object);
  instancer_id = p_id.AppendElementString("Instancer");
  CLOG_INFO(LOG_BSD, 2, "%s, instancer_id=%s", ((ID *)parent_obj)->name, instancer_id.GetText());
}

void InstancerData::init()
{
  CLOG_INFO(LOG_BSD, 2, "%s", ((ID *)parent_obj)->name);

  set_instances();
  MeshData::init();
}

pxr::GfMatrix4d InstancerData::transform()
{
  return pxr::GfMatrix4d(1.0);
}

bool InstancerData::update_visibility(View3D *view3d)
{
  if (!view3d) {
    return false;
  }

  bool prev_visible = visible;
  visible = BKE_object_is_visible_in_viewport(view3d, parent_obj);
  bool ret = visible != prev_visible;
  if (ret) {
    scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
        p_id, pxr::HdChangeTracker::DirtyVisibility);
  }
  return ret;
}

pxr::VtValue InstancerData::get_data(pxr::TfToken const &key) const
{
  CLOG_INFO(LOG_BSD, 3, "%s [%s]", id->name, key.GetText());

  if (key == pxr::HdInstancerTokens->instanceTransform) {
    return pxr::VtValue(transforms);
  }
  return MeshData::get_data(key);
}

pxr::HdPrimvarDescriptorVector InstancerData::instancer_primvar_descriptors(
    pxr::HdInterpolation interpolation)
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationInstance) {
    primvars.emplace_back(
        pxr::HdInstancerTokens->instanceTransform, interpolation, pxr::HdPrimvarRoleTokens->none);
  }
  return primvars;
}

pxr::VtIntArray InstancerData::instance_indices()
{
  pxr::VtIntArray ret(transforms.size());
  for (size_t i = 0; i < ret.size(); ++i) {
    ret[i] = i;
  }
  return ret;
}

bool InstancerData::is_base(Object *object) const
{
  return (ID *)object == id;
}

bool InstancerData::set_instances()
{
  ID *prev_id = id;
  id = nullptr;
  transforms.clear();
  ListBase *lb = object_duplilist(scene_delegate->depsgraph, scene_delegate->scene, parent_obj);
  LISTBASE_FOREACH (DupliObject *, dupli, lb) {
    if (!id) {
      /* TODO: We create instances only for object in first dupli.
         Instances should be created for all objects */
      id = (ID *)dupli->ob;
    }
    if (id != (ID *)dupli->ob) {
      continue;
    }
    transforms.push_back(gf_matrix_from_transform(dupli->mat));
    CLOG_INFO(
        LOG_BSD, 2, "Instance %s (%s) %d", id->name, ((ID *)dupli->ob)->name, dupli->random_id);
  }
  free_object_duplilist(lb);

  return id != prev_id;
}

void InstancerData::insert()
{
  CLOG_INFO(LOG_BSD, 2, "%s", ((ID *)parent_obj)->name);
  MeshData::insert();

  if (face_vertex_counts.empty()) {
    return;
  }
  scene_delegate->GetRenderIndex().InsertInstancer(scene_delegate, instancer_id);
}

void InstancerData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", ((ID *)parent_obj)->name);

  if (!scene_delegate->GetRenderIndex().HasInstancer(instancer_id)) {
    return;
  }
  scene_delegate->GetRenderIndex().RemoveInstancer(instancer_id);

  MeshData::remove();
}

void InstancerData::update()
{
  CLOG_INFO(LOG_BSD, 2, "%s", ((ID *)parent_obj)->name);

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  unsigned int recalc = ((ID *)parent_obj)->recalc;

  Object *object = (Object *)id;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    scene_delegate->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
        p_id, pxr::HdChangeTracker::AllDirty);
    return;
  }

  if ((recalc & ID_RECALC_GEOMETRY) || (((ID *)parent_obj->data)->recalc & ID_RECALC_GEOMETRY)) {
    init();
    bits |= pxr::HdChangeTracker::AllDirty;
  }
  else if (recalc & ID_RECALC_TRANSFORM || id->recalc & ID_RECALC_TRANSFORM) {
    set_instances();
    bits |= pxr::HdChangeTracker::DirtyTransform;
  }
  if (bits != pxr::HdChangeTracker::Clean) {
    scene_delegate->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(instancer_id, bits);
  }
}

}  // namespace blender::render::hydra
