/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>

#include "blender_scene_delegate.h"
#include "instancer.h"

namespace blender::render::hydra {

InstancerData::InstancerData(BlenderSceneDelegate *scene_delegate,
                             Object *object,
                             pxr::SdfPath const &prim_id)
    : ObjectData(scene_delegate, object, prim_id)
{
}

std::unique_ptr<InstancerData> InstancerData::create(BlenderSceneDelegate *scene_delegate,
                                                     Object *object,
                                                     pxr::SdfPath const &prim_id)
{
  auto data = std::make_unique<InstancerData>(scene_delegate, object, prim_id);
  data->init();
  data->insert();
  return data;
}

bool InstancerData::is_supported(Object *object)
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

void InstancerData::init()
{
  ID_LOG(2, "");
  set_instances();
}

void InstancerData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertInstancer(scene_delegate_, prim_id);
  for (auto &it : instances_) {
    it.second.obj_data->insert();
  }
}

void InstancerData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 2, "%s", prim_id.GetText());
  for (auto &it : instances_) {
    it.second.obj_data->remove();
  }
  scene_delegate_->GetRenderIndex().RemoveInstancer(prim_id);
}

void InstancerData::update()
{
  ID_LOG(2, "");

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;

  Object *object = (Object *)id;
  if ((id->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    set_instances();
    bits |= pxr::HdChangeTracker::AllDirty;
  }
  else if (id->recalc & ID_RECALC_TRANSFORM) {
    set_instances();
    bits |= pxr::HdChangeTracker::DirtyTransform;
  }
  if (bits != pxr::HdChangeTracker::Clean) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(prim_id, bits);
  }
}

pxr::VtValue InstancerData::get_data(pxr::TfToken const &key) const
{
  ID_LOG(3, "%s", key.GetText());
  pxr::VtValue ret;
  if (key == pxr::HdInstancerTokens->instanceTransform) {
    ret = transforms_;
  }
  return ret;
}

bool InstancerData::update_visibility()
{
  bool ret = ObjectData::update_visibility();
  if (ret) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
        prim_id, pxr::HdChangeTracker::DirtyVisibility);
    for (auto &it : instances_) {
      it.second.obj_data->visible = visible;
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
          it.second.obj_data->prim_id, pxr::HdChangeTracker::DirtyVisibility);
    }
  }
  return ret;
}

pxr::HdPrimvarDescriptorVector InstancerData::primvar_descriptors(
    pxr::HdInterpolation interpolation) const
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationInstance) {
    primvars.emplace_back(
        pxr::HdInstancerTokens->instanceTransform, interpolation, pxr::HdPrimvarRoleTokens->none);
  }
  return primvars;
}

pxr::VtIntArray InstancerData::indices(pxr::SdfPath const &id) const
{
  return instances_.find(id)->second.indices;
}

ObjectData *InstancerData::object_data(pxr::SdfPath const &id) const
{
  return instances_.find(id)->second.obj_data.get();
}

pxr::SdfPathVector InstancerData::prototypes() const
{
  pxr::SdfPathVector paths;
  for (auto &it : instances_) {
    paths.push_back(it.first);
  }
  return paths;
}

void InstancerData::check_update(Object *object)
{
  pxr::SdfPath path = object_prim_id(object);
  auto it = instances_.find(path);
  if (it == instances_.end()) {
    return;
  }
  ObjectData *obj_data = it->second.obj_data.get();
  obj_data->update();

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;
  if (object->id.recalc & ID_RECALC_TRANSFORM) {
    set_instances();
    bits |= pxr::HdChangeTracker::DirtyTransform;
  }
  if (bits != pxr::HdChangeTracker::Clean) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(prim_id, bits);
  }
}

void InstancerData::check_remove(std::set<std::string> &available_objects)
{
  bool ret = false;
  for (auto it = instances_.begin(); it != instances_.end(); ++it) {
    if (available_objects.find(it->first.GetName()) != available_objects.end()) {
      continue;
    }
    it->second.obj_data->remove();
    instances_.erase(it);
    it = instances_.begin();
    ret = true;
  }
  if (ret) {
    set_instances();
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
        prim_id, pxr::HdChangeTracker::AllDirty);
  }
}

void InstancerData::available_materials(std::set<pxr::SdfPath> &paths) const
{
  for (auto &it : instances_) {
    pxr::SdfPath mat_id = ((MeshData *)it.second.obj_data.get())->material_id();
    if (!mat_id.IsEmpty()) {
      paths.insert(mat_id);
    }
  }
}

pxr::SdfPath InstancerData::object_prim_id(Object *object) const
{
  /* Making id of object in form like <prefix>_<pointer in 16 hex digits format> */
  char str[32];
  snprintf(str, 32, "O_%016llx", (uint64_t)object);
  return prim_id.AppendElementString(str);
}

void InstancerData::set_instances()
{
  transforms_.clear();
  for (auto &it : instances_) {
    it.second.indices.clear();
  }
  int index = 0;
  Instance *inst;
  pxr::SdfPath p_id;

  ListBase *lb = object_duplilist(
      scene_delegate_->depsgraph, scene_delegate_->scene, (Object *)id);
  LISTBASE_FOREACH (DupliObject *, dupli, lb) {
    p_id = object_prim_id(dupli->ob);
    auto it = instances_.find(p_id);
    if (it == instances_.end()) {
      inst = &instances_[p_id];
      if (!is_supported(dupli->ob)) {
        continue;
      }
      inst->obj_data = ObjectData::create(scene_delegate_, dupli->ob, p_id);

      /* Instance's transform should be identity */
      inst->obj_data->transform = pxr::GfMatrix4d(1.0);
    }
    else {
      inst = &it->second;
    }
    transforms_.push_back(gf_matrix_from_transform(dupli->mat));
    inst->indices.push_back(index);
    ID_LOG(2, "%s %d", inst->obj_data->id->name, index);
    ++index;
  }
  free_object_duplilist(lb);

  /* Remove intances without indices */
  for (auto it = instances_.begin(); it != instances_.end(); ++it) {
    if (!it->second.indices.empty()) {
      continue;
    }
    it->second.obj_data->remove();
    instances_.erase(it);
    it = instances_.begin();
  }
}

}  // namespace blender::render::hydra
