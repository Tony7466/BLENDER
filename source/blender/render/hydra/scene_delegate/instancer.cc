/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/base/gf/vec2f.h>
#include <pxr/imaging/hd/light.h>

#include "DEG_depsgraph_query.h"

#include "blender_scene_delegate.h"
#include "instancer.h"

namespace blender::render::hydra {

InstancerData::InstancerData(BlenderSceneDelegate *scene_delegate, pxr::SdfPath const &prim_id)
    : IdData(scene_delegate, nullptr, prim_id)
{
}

bool InstancerData::is_instance_supported(Object *object)
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

void InstancerData::init() {}

void InstancerData::insert() {}

void InstancerData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  for (auto &m_inst : mesh_instances_.values()) {
    m_inst.data->remove();
  }
  if (!mesh_instances_.is_empty()) {
    scene_delegate_->GetRenderIndex().RemoveInstancer(prim_id);
  }
  mesh_instances_.clear();

  for (auto &l_inst : light_instances_.values()) {
    l_inst.transforms.clear();
    update_light_instance(l_inst);
  }
  light_instances_.clear();
}

void InstancerData::update() {}

pxr::VtValue InstancerData::get_data(pxr::TfToken const &key) const
{
  ID_LOG(3, "%s", key.GetText());
  if (key == pxr::HdInstancerTokens->instanceTransform) {
    return pxr::VtValue(mesh_transforms_);
  }
  return pxr::VtValue();
}

pxr::GfMatrix4d InstancerData::get_transform(pxr::SdfPath const &id) const
{
  LightInstance *l_inst = light_instance(id);
  if (l_inst) {
    return l_inst->transforms[light_prim_id_index(id)];
  }

  /* Mesh instance transform must be identity */
  return pxr::GfMatrix4d(1.0);
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
  return mesh_instance(id)->indices;
}

ObjectData *InstancerData::object_data(pxr::SdfPath const &id) const
{
  MeshInstance *m_inst = mesh_instance(id);
  if (m_inst) {
    return m_inst->data.get();
  }
  LightInstance *l_inst = light_instance(id);
  if (l_inst) {
    return l_inst->data.get();
  }
  return nullptr;
}

pxr::SdfPathVector InstancerData::prototypes() const
{
  pxr::SdfPathVector paths;
  for (auto &m_inst : mesh_instances_.values()) {
    for (auto &p : m_inst.data->submesh_paths()) {
      paths.push_back(p);
    }
  }
  return paths;
}

void InstancerData::available_materials(Set<pxr::SdfPath> &paths) const
{
  for (auto &m_inst : mesh_instances_.values()) {
    ((MeshData *)m_inst.data.get())->available_materials(paths);
  }
}

void InstancerData::update_double_sided(MaterialData *mat_data)
{
  for (auto &m_inst : mesh_instances_.values()) {
    m_inst.data->update_double_sided(mat_data);
  }
}

void InstancerData::pre_update()
{
  mesh_transforms_.clear();
  for (auto &m_inst : mesh_instances_.values()) {
    m_inst.indices.clear();
  }
  for (auto &l_inst : light_instances_.values()) {
    l_inst.transforms.clear();
  }
}

void InstancerData::update_instance(Object *parent_ob, DupliObject *dupli)
{
  if (!ObjectData::is_visible(scene_delegate_, parent_ob, OB_VISIBLE_INSTANCES)) {
    return;
  }
  Object *ob = dupli->ob;
  if (!is_instance_supported(ob)) {
    return;
  }
  if (!scene_delegate_->shading_settings.use_scene_lights && ob->type == OB_LAMP) {
    return;
  }

  pxr::SdfPath p_id = object_prim_id(ob);
  if (ob->type == OB_LAMP) {
    LightInstance *inst = light_instance(p_id);
    if (!inst) {
      inst = &light_instances_.lookup_or_add_default(p_id);
      inst->data = std::make_unique<LightData>(scene_delegate_, ob, p_id);
      inst->data->init();
    }
    ID_LOG(2, "Light %s %d", inst->data->id->name, (int)inst->transforms.size());
    inst->transforms.push_back(gf_matrix_from_transform(dupli->mat));
  }
  else {
    MeshInstance *inst = mesh_instance(p_id);
    if (!inst) {
      inst = &mesh_instances_.lookup_or_add_default(p_id);
      inst->data = std::make_unique<MeshData>(scene_delegate_, ob, p_id);
      inst->data->init();
      inst->data->insert();
    }
    else {
      inst->data->update();
    }
    ID_LOG(2, "Mesh %s %d", inst->data->id->name, (int)mesh_transforms_.size());
    inst->indices.push_back(mesh_transforms_.size());
    mesh_transforms_.push_back(gf_matrix_from_transform(dupli->mat));
  }
}

void InstancerData::post_update()
{
  /* Remove mesh intances without indices */
  mesh_instances_.remove_if([&](auto item) {
    bool res = item.value.indices.empty();
    if (res) {
      item.value.data->remove();
    }
    return res;
  });

  /* Update light intances and remove instances without transforms */
  for (auto &l_inst : light_instances_.values()) {
    update_light_instance(l_inst);
  }
  light_instances_.remove_if([&](auto item) { return item.value.transforms.empty(); });

  /* Insert/remove/update instancer in RenderIndex */
  pxr::HdRenderIndex &index = scene_delegate_->GetRenderIndex();
  if (mesh_instances_.is_empty()) {
    /* Important: removing instancer when light_instances_ are empty too */
    if (index.HasInstancer(prim_id) && light_instances_.is_empty()) {
      index.RemoveInstancer(prim_id);
      ID_LOG(1, "Remove instancer");
    }
  }
  else {
    if (index.HasInstancer(prim_id)) {
      index.GetChangeTracker().MarkInstancerDirty(prim_id, pxr::HdChangeTracker::AllDirty);
      ID_LOG(1, "Update instancer");
    }
    else {
      index.InsertInstancer(scene_delegate_, prim_id);
      ID_LOG(1, "Insert instancer");
    }
  }
}

pxr::SdfPath InstancerData::object_prim_id(Object *object) const
{
  /* Making id of object in form like <prefix>_<pointer in 16 hex digits format> */
  char name[32];
  snprintf(name, sizeof(name), "O_%p", object);
  return prim_id.AppendElementString(name);
}

pxr::SdfPath InstancerData::light_prim_id(LightInstance const &inst, int index) const
{
  char name[16];
  snprintf(name, sizeof(name), "L_%08x", index);
  return inst.data->prim_id.AppendElementString(name);
}

int InstancerData::light_prim_id_index(pxr::SdfPath const &id) const
{
  int index;
  sscanf(id.GetName().c_str(), "L_%x", &index);
  return index;
}

void InstancerData::update_light_instance(LightInstance &inst)
{
  auto &render_index = scene_delegate_->GetRenderIndex();
  LightData &l_data = *inst.data;

  int i;
  pxr::SdfPath p;

  /* Remove old light instances */
  while (inst.count > inst.transforms.size()) {
    --inst.count;
    p = light_prim_id(inst, inst.count);
    render_index.RemoveSprim(l_data.prim_type_, p);
    ID_LOG(2, "Remove %s", p.GetText());
  }

  /* Update current light instances */
  if (inst.data->prim_type((Light *)((Object *)l_data.id)->data) != l_data.prim_type_) {
    /* Recreate instances when prim_type was changed */
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.RemoveSprim(l_data.prim_type_, p);
      ID_LOG(2, "Remove %s", p.GetText());
    }
    inst.data->init();
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.InsertSprim(l_data.prim_type_, scene_delegate_, p);
      ID_LOG(2, "Insert %s (%s)", p.GetText(), l_data.id->name);
    }
  }
  else {
    /* Update light instances*/
    pxr::HdDirtyBits bits = pxr::HdLight::DirtyTransform;
    Object *obj = (Object *)inst.data->id;
    if (obj->id.recalc & ID_RECALC_GEOMETRY || ((ID *)obj->data)->recalc & ID_RECALC_GEOMETRY) {
      l_data.init();
      bits = pxr::HdLight::AllDirty;
    }
    for (i = 0; i < inst.count; ++i) {
      p = light_prim_id(inst, i);
      render_index.GetChangeTracker().MarkSprimDirty(p, bits);
      ID_LOG(2, "Update %s (%s)", p.GetText(), l_data.id->name);
    }
  }

  /* Add new light instances */
  while (inst.count < inst.transforms.size()) {
    p = light_prim_id(inst, inst.count);
    render_index.InsertSprim(l_data.prim_type_, scene_delegate_, p);
    ID_LOG(2, "Insert %s (%s)", p.GetText(), l_data.id->name);
    ++inst.count;
  }
}

InstancerData::MeshInstance *InstancerData::mesh_instance(pxr::SdfPath const &id) const
{
  auto m_inst = mesh_instances_.lookup_ptr(id.GetPathElementCount() == 4 ? id.GetParentPath() :
                                                                           id);
  if (!m_inst) {
    return nullptr;
  }
  return const_cast<MeshInstance *>(m_inst);
}

InstancerData::LightInstance *InstancerData::light_instance(pxr::SdfPath const &id) const
{
  auto l_inst = light_instances_.lookup_ptr(id.GetPathElementCount() == 4 ? id.GetParentPath() :
                                                                            id);
  if (!l_inst) {
    return nullptr;
  }
  return const_cast<LightInstance *>(l_inst);
}

}  // namespace blender::render::hydra
