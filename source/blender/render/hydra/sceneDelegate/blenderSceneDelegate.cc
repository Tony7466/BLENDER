/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <pxr/imaging/hd/light.h>
#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/usd/usdLux/tokens.h>
#include <pxr/imaging/hdSt/tokens.h>

#include "glog/logging.h"

#include "blenderSceneDelegate.h"
#include "object.h"

using namespace std;

namespace blender::render::hydra {

BlenderSceneDelegate::BlenderSceneDelegate(HdRenderIndex* parentIndex, SdfPath const& delegateID)
  : HdSceneDelegate(parentIndex, delegateID),
    b_depsgraph(nullptr),
    b_context(nullptr),
    view3d(nullptr),
    is_populated(false)
{
}

void BlenderSceneDelegate::set_material(MeshData &mesh_data)
{
  Material *material = mesh_data.material();
  if (!material) {
    mesh_data.material_id = SdfPath::EmptyPath();
    return;
  }
  SdfPath id = MaterialData::prim_id(this, material);
  MaterialData *mat_data = material_data(id);
  if (!mat_data) {
    materials[id] = MaterialData::init(this, material);
    mat_data = material_data(id);
    mat_data->export_mtlx();
    mat_data->insert_prim();
  }
  mesh_data.material_id = id;
}

void BlenderSceneDelegate::update_material(Material *material)
{
  MaterialData *mat_data = material_data(MaterialData::prim_id(this, material));
  if (mat_data) {
    mat_data->export_mtlx();
    mat_data->mark_prim_dirty(IdData::DirtyBits::AllDirty);
  }
}

void BlenderSceneDelegate::update_world()
{
  World *world = (World *)b_depsgraph->scene().world().ptr.data;
  if (!world_data) {
    if (world) {
      world_data = WorldData::init(this, world, (bContext *)b_context->ptr.data);
      world_data->insert_prim();
    }
  }
  else {
    if (world) {
      world_data = WorldData::init(this, world, (bContext *)b_context->ptr.data);
      world_data->mark_prim_dirty(IdData::DirtyBits::AllDirty);
    }
    else {
      world_data->remove_prim();
      world_data = nullptr;
    }
  }
}

bool BlenderSceneDelegate::GetVisible(SdfPath const &id)
{
  if (id == WorldData::prim_id(this)) {
    return true;
  }

  return object_data(id)->visible;
}

void BlenderSceneDelegate::update_collection()
{
  /* add new objects */
  std::set<SdfPath> available_objects;
  for (auto &inst : b_depsgraph->object_instances) {
    if (inst.is_instance()) {
      continue;
    }
    Object *object = (Object *)inst.object().ptr.data;
    if (!supported_object(object)) {
      continue;
    }
    available_objects.insert(ObjectData::prim_id(this, object));

    if (!is_populated) {
      add_update_object(object, true, true, true);
    }
  }

  if (!is_populated) {
    return;
  }

  /* remove unused objects */
  for (auto it = objects.begin(); it != objects.end(); ++it) {
    if (available_objects.find(it->first) != available_objects.end()) {
      continue;
    }
    it->second->remove_prim();
    objects.erase(it);
    it = objects.begin();
  }

  /* remove unused materials */
  std::set<SdfPath> available_materials;
  for (auto &obj : objects) {
    MeshData *m_data = dynamic_cast<MeshData *>(obj.second.get());
    if (m_data && !m_data->material_id.IsEmpty()) {
      available_materials.insert(m_data->material_id);
    }
  }
  for (auto it = materials.begin(); it != materials.end(); ++it) {
    if (available_materials.find(it->first) != available_materials.end()) {
      continue;
    }
    it->second->remove_prim();
    materials.erase(it);
    it = materials.begin();
  }
}

void BlenderSceneDelegate::add_update_object(Object *object, bool geometry, bool transform, bool shading)
{
  SdfPath id = ObjectData::prim_id(this, object);
  ObjectData *obj_data = object_data(id);
  if (!obj_data) {
    objects[id] = ObjectData::init(this, object);
    obj_data = object_data(id);
    obj_data->update_visibility(view3d);
    obj_data->insert_prim();
    MeshData *m_data = dynamic_cast<MeshData *>(obj_data);
    if (m_data) {
      set_material(*m_data);
    }
    return;
  }

  if (geometry) {
    objects[id] = ObjectData::init(this, object);
    obj_data = object_data(id);
    obj_data->update_visibility(view3d);
    MeshData *m_data = dynamic_cast<MeshData *>(obj_data);
    if (m_data) {
      set_material(*m_data);
    }
    obj_data->mark_prim_dirty(IdData::DirtyBits::AllDirty);
    return;
  }

  if (transform) {
    obj_data->mark_prim_dirty(IdData::DirtyBits::DirtyTransform);
  }

  if (shading) {
    obj_data->mark_prim_dirty(IdData::DirtyBits::DirtyMaterial);
  }
}

ObjectData *BlenderSceneDelegate::object_data(SdfPath const &id)
{
  auto it = objects.find(id);
  if (it == objects.end()) {
    return nullptr;
  }
  return it->second.get();
}

MeshData *BlenderSceneDelegate::mesh_data(SdfPath const &id)
{
  return static_cast<MeshData *>(object_data(id));
}

LightData *BlenderSceneDelegate::light_data(SdfPath const &id)
{
  return static_cast<LightData *>(object_data(id));
}

MaterialData *BlenderSceneDelegate::material_data(SdfPath const &id)
{
  auto it = materials.find(id);
  if (it == materials.end()) {
    return nullptr;
  }
  return it->second.get();
}

bool BlenderSceneDelegate::supported_object(Object *object)
{
  return object->type == OB_MESH ||
         object->type == OB_LAMP ||
         object->type == OB_SURF ||
         object->type == OB_FONT ||
         object->type == OB_CURVES ||
         object->type == OB_CURVES_LEGACY ||
         object->type == OB_MBALL;
}

void BlenderSceneDelegate::Populate(BL::Depsgraph &b_deps, BL::Context &b_cont)
{
  LOG(INFO) << "Populate " << is_populated;

  b_depsgraph = &b_deps;
  b_context = &b_cont;
  view3d = (View3D *)b_context->space_data().ptr.data;

  if (!is_populated) {
    /* Export initial objects */
    update_collection();
    update_world();

    is_populated = true;
    return;
  }

  /* Working with updates */
  bool do_update_collection = false;
  bool do_update_visibility = false;
  bool do_update_world = false;

  for (auto &update : b_depsgraph->updates) {
    BL::ID id = update.id();
    LOG(INFO) << "Update: " << id.name_full() << " ["
              << update.is_updated_transform()
              << update.is_updated_geometry()
              << update.is_updated_shading() << "]";

    if (id.is_a(&RNA_Object)) {
      Object *object = (Object *)id.ptr.data;
      if (!supported_object(object)) {
        continue;
      }
      add_update_object(object,
                        update.is_updated_geometry(),
                        update.is_updated_transform(),
                        update.is_updated_shading());
      continue;
    }

    if (id.is_a(&RNA_Material)) {
      if (update.is_updated_shading()) {
        Material *material = (Material *)id.ptr.data;
        update_material(material);
      }
      continue;
    }
      
    if (id.is_a(&RNA_Collection)) {
      if (update.is_updated_transform() && update.is_updated_geometry()) {
        do_update_collection = true;
      }
      continue;
    }

    if (id.is_a(&RNA_Scene)) {
      if (!update.is_updated_geometry() && !update.is_updated_transform() && !update.is_updated_shading()) {
        do_update_visibility = true;

        Scene *scene = (Scene *)id.ptr.data;
        if ((scene->world && !world_data) || (!scene->world && world_data)) {
          do_update_world = true;
        }
      }
      continue;
    }

    if (id.is_a(&RNA_World)) {
      if (update.is_updated_shading()) {
        do_update_world = true;
      }
      continue;
    }
  }

  if (do_update_collection) {
    update_collection();
  }
  if (do_update_visibility) {
    update_visibility();
  }
  if (do_update_world) {
    update_world();
  }

}

void BlenderSceneDelegate::update_visibility()
{
  /* Check and update visibility */
  for (auto &obj : objects) {
    if (obj.second->update_visibility(view3d)) {
      obj.second->mark_prim_dirty(IdData::DirtyBits::DirtyVisibility);
    };
  }

  /* Export of new visible objects which were not exported before */
  for (auto &inst : b_depsgraph->object_instances) {
    if (inst.is_instance()) {
      continue;
    }
    Object *object = (Object *)inst.object().ptr.data;
    if (!supported_object(object)) {
      continue;
    }

    if (!object_data(ObjectData::prim_id(this, object))) {
      add_update_object(object, true, true, true);
    }
  }
}

HdMeshTopology BlenderSceneDelegate::GetMeshTopology(SdfPath const& id)
{
  MeshData *m_data = mesh_data(id);
  return m_data->mesh_topology();
}

VtValue BlenderSceneDelegate::Get(SdfPath const& id, TfToken const& key)
{
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    return obj_data->get_data(key);
  }

  MaterialData *mat_data = material_data(id);
  if (mat_data) {
    return mat_data->get_data(key);
  }
  return VtValue();
}

HdPrimvarDescriptorVector BlenderSceneDelegate::GetPrimvarDescriptors(SdfPath const& id, HdInterpolation interpolation)
{
  return mesh_data(id)->primvar_descriptors(interpolation);
}

SdfPath BlenderSceneDelegate::GetMaterialId(SdfPath const & rprimId)
{
  return mesh_data(rprimId)->material_id;
}

VtValue BlenderSceneDelegate::GetMaterialResource(SdfPath const& id)
{
  MaterialData *mat_data = material_data(id);
  if (mat_data) {
    return mat_data->material_resource();
  }
  return VtValue();
}

GfMatrix4d BlenderSceneDelegate::GetTransform(SdfPath const& id)
{
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    return obj_data->transform();
  }
  if (id == WorldData::prim_id(this)) {
    return world_data->transform();
  }
  return GfMatrix4d();
}

VtValue BlenderSceneDelegate::GetLightParamValue(SdfPath const& id, TfToken const& key)
{
  LightData *l_data = light_data(id);
  if (l_data) {
    return l_data->get_data(key);
  }
  if (id == WorldData::prim_id(this)) {
    return world_data->get_data(key);
  }
  return VtValue();
}

} // namespace blender::render::hydra
