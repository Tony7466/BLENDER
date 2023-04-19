/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>

#include "../utils.h"
#include "blender_scene_delegate.h"
#include "instancer.h"

namespace blender::render::hydra {

InstancerData::InstancerData(BlenderSceneDelegate *scene_delegate, Object *object)
    : ObjectData(scene_delegate, object)
{
  p_id_ = prim_id(scene_delegate, object);
  ID_LOG(2, "");
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

void InstancerData::init()
{
  ID_LOG(2, "");
  set_instances();
}

void InstancerData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertInstancer(scene_delegate_, p_id_);
  for (auto &it : instances_) {
    it.second.obj_data->insert();
  }
}

void InstancerData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", p_id_.GetText());
  for (auto &it : instances_) {
    it.second.obj_data->remove();
  }
  scene_delegate_->GetRenderIndex().RemoveInstancer(p_id_);
}

void InstancerData::update()
{
  ID_LOG(2, "");

  pxr::HdDirtyBits bits = pxr::HdChangeTracker::Clean;

  Object *object = (Object *)id_;
  if ((id_->recalc & ID_RECALC_GEOMETRY) || (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY)) {
    set_instances();
    bits |= pxr::HdChangeTracker::AllDirty;
  }
  else if (id_->recalc & ID_RECALC_TRANSFORM) {
    set_instances();
    bits |= pxr::HdChangeTracker::DirtyTransform;
  }
  if (bits != pxr::HdChangeTracker::Clean) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(p_id_, bits);
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

pxr::GfMatrix4d InstancerData::transform()
{
  return pxr::GfMatrix4d(1.0);
}

bool InstancerData::update_visibility(View3D *view3d)
{
  if (!view3d) {
    return false;
  }

  bool ret = ObjectData::update_visibility(view3d);
  if (ret) {
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(
        p_id_, pxr::HdChangeTracker::DirtyVisibility);
    for (auto &it : instances_) {
      it.second.obj_data->visible = visible;
      scene_delegate_->GetRenderIndex().GetChangeTracker().MarkRprimDirty(
          it.second.obj_data->p_id_, pxr::HdChangeTracker::DirtyVisibility);
    }
  }
  return ret;
}

pxr::HdPrimvarDescriptorVector InstancerData::primvar_descriptors(pxr::HdInterpolation interpolation)
{
  pxr::HdPrimvarDescriptorVector primvars;
  if (interpolation == pxr::HdInterpolationInstance) {
    primvars.emplace_back(
        pxr::HdInstancerTokens->instanceTransform, interpolation, pxr::HdPrimvarRoleTokens->none);
  }
  return primvars;
}

pxr::VtIntArray InstancerData::indices(pxr::SdfPath const &id)
{
  return instances_[id].indices;
}

ObjectData *InstancerData::object_data(pxr::SdfPath const &id)
{
  return instances_[id].obj_data.get();
}

pxr::SdfPathVector InstancerData::prototypes()
{
  pxr::SdfPathVector paths;
  for (auto &it : instances_) {
    paths.push_back(it.first);
  }
  return paths;
}

void InstancerData::check_update(Object *object)
{
  pxr::SdfPath path = ObjectData::prim_id(scene_delegate_, object);
  path = p_id_.AppendElementString(path.GetName());
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
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(p_id_, bits);
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
    scene_delegate_->GetRenderIndex().GetChangeTracker().MarkInstancerDirty(p_id_,
                                                                           pxr::HdChangeTracker::AllDirty);
  }
}

void InstancerData::available_materials(std::set<pxr::SdfPath> &paths)
{
  for (auto &it : instances_) {
    pxr::SdfPath mat_id = ((MeshData *)it.second.obj_data.get())->material_id();
    if (!mat_id.IsEmpty()) {
      paths.insert(mat_id);
    }
  }
}

void InstancerData::set_instances()
{
  transforms_.clear();
  for (auto &it : instances_) {
    it.second.indices.clear();
  }
  int index = 0;
  Instance *inst;
  pxr::SdfPath path;

  ListBase *lb = object_duplilist(scene_delegate_->depsgraph_, scene_delegate_->scene_, (Object *)id_);
  LISTBASE_FOREACH (DupliObject *, dupli, lb) {
    path = ObjectData::prim_id(scene_delegate_, dupli->ob);
    path = p_id_.AppendElementString(path.GetName());
    auto it = instances_.find(path);
    if (it == instances_.end()) {
      inst = &instances_[path];
      if (!is_supported(dupli->ob)) {
        continue;
      }
      inst->obj_data = std::make_unique<InstanceMeshData>(scene_delegate_, dupli->ob, path);
      inst->obj_data->init();
      inst->obj_data->insert();
    }
    else {
      inst = &it->second;
    }
    transforms_.push_back(gf_matrix_from_transform(dupli->mat));
    inst->indices.push_back(index);
    ID_LOG(2, "Instance %s %d", inst->obj_data->id_->name, index);
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

InstanceMeshData::InstanceMeshData(BlenderSceneDelegate *scene_delegate,
                                   Object *object,
                                   pxr::SdfPath const &p_id)
    : MeshData(scene_delegate, object)
{
  this->p_id_ = p_id;
}

pxr::GfMatrix4d InstanceMeshData::transform()
{
  return pxr::GfMatrix4d(1.0);
}

}  // namespace blender::render::hydra
