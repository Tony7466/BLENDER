/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <bitset>

#include "DEG_depsgraph_query.h"
#include "DNA_scene_types.h"

#include "..\utils.h"
#include "blender_scene_delegate.h"

namespace blender::render::hydra {

CLG_LOGREF_DECLARE_GLOBAL(LOG_BSD, "rhd.bsd");

BlenderSceneDelegate::BlenderSceneDelegate(pxr::HdRenderIndex *parent_index,
                                           pxr::SdfPath const &delegate_id,
                                           BlenderSceneDelegate::EngineType engine_type)
    : HdSceneDelegate(parent_index, delegate_id),
      engine_type(engine_type),
      depsgraph(nullptr),
      context(nullptr),
      view3d(nullptr)
{
}

void BlenderSceneDelegate::update_world()
{
  World *world = scene->world;
  if (!world_data) {
    if (world) {
      world_data = WorldData::create(this, world, context);
    }
  }
  else {
    if (world) {
      world_data->update(world);
    }
    else {
      world_data->remove();
      world_data = nullptr;
    }
  }
}

bool BlenderSceneDelegate::GetVisible(pxr::SdfPath const &id)
{
  if (id == WorldData::prim_id(this)) {
    return true;
  }

  return object_data(id)->visible;
}

pxr::SdfPath BlenderSceneDelegate::GetInstancerId(pxr::SdfPath const &prim_id)
{
  CLOG_INFO(LOG_BSD, 3, "%s", prim_id.GetText());
  InstancerData *i_data = instancer_data(prim_id, true);
  if (i_data) {
    return i_data->instancer_id;
  }
  return pxr::SdfPath();
}

pxr::SdfPathVector BlenderSceneDelegate::GetInstancerPrototypes(pxr::SdfPath const &instancer_id)
{
  CLOG_INFO(LOG_BSD, 3, "%s", instancer_id.GetText());
  pxr::SdfPathVector paths;
  paths.push_back(instancer_id.GetParentPath());
  return paths;
}

pxr::VtIntArray BlenderSceneDelegate::GetInstanceIndices(pxr::SdfPath const &instancer_id,
                                                         pxr::SdfPath const &prototype_id)
{
  CLOG_INFO(LOG_BSD, 3, "%s, %s", instancer_id.GetText(), prototype_id.GetText());
  InstancerData *i_data = instancer_data(instancer_id);
  return i_data->instance_indices();
}

pxr::GfMatrix4d BlenderSceneDelegate::GetInstancerTransform(pxr::SdfPath const &instancer_id)
{
  CLOG_INFO(LOG_BSD, 3, "%s", instancer_id.GetText());
  InstancerData *i_data = instancer_data(instancer_id);
  return i_data->transform();
}

void BlenderSceneDelegate::update_collection(bool remove, bool visibility)
{
  if (visibility) {
    /* Check and update visibility */
    for (auto &obj : objects) {
      obj.second->update_visibility(view3d);
    }
  }

  /* Export of new visible objects which were not exported before */
  std::set<pxr::SdfPath> available_objects;
  pxr::SdfPath id;

  DEGObjectIterSettings settings = {0};
  settings.depsgraph = depsgraph;
  settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY | DEG_ITER_OBJECT_FLAG_VISIBLE |
                   DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET;
  DEGObjectIterData data = {0};
  data.settings = &settings;
  data.graph = settings.depsgraph;
  data.flag = settings.flags;
  ITER_BEGIN (DEG_iterator_objects_begin,
              DEG_iterator_objects_next,
              DEG_iterator_objects_end,
              &data,
              Object *,
              object) {

    CLOG_INFO(LOG_BSD, 2, "Add %s", ((ID *)object)->name);
    if (!ObjectData::supported(object)) {
      continue;
    }

    id = ObjectData::prim_id(this, object);
    if (remove) {
      available_objects.insert(id);
      if ((object->transflag & OB_DUPLI) && InstancerData::supported(object)) {
        available_objects.insert(InstancerData::prim_id(this, object));
      }
    }

    if (!object_data(id)) {
      add_update_object(object);
    }
  }
  ITER_END;

  if (remove) {
    /* remove unused objects */
    for (auto it = objects.begin(); it != objects.end(); ++it) {
      if (available_objects.find(it->first) != available_objects.end()) {
        continue;
      }
      it->second->remove();
      objects.erase(it);
      it = objects.begin();
    }

    /* remove unused materials */
    std::set<pxr::SdfPath> available_materials;
    for (auto &obj : objects) {
      MeshData *m_data = dynamic_cast<MeshData *>(obj.second.get());
      if (!m_data) {
        continue;
      }
      pxr::SdfPath mat_id = m_data->material_id();
      if (!mat_id.IsEmpty()) {
        available_materials.insert(mat_id);
      }
    }
    for (auto it = materials.begin(); it != materials.end(); ++it) {
      if (available_materials.find(it->first) != available_materials.end()) {
        continue;
      }
      it->second->remove();
      materials.erase(it);
      it = materials.begin();
    }
  }
}

void BlenderSceneDelegate::add_update_object(Object *object)
{
  if ((object->transflag & OB_DUPLI) && InstancerData::supported(object)) {
    add_update_instancer(object);
  }
  InstancerData *i_data = instancer_data(object);
  if (i_data) {
    i_data->update();
  }

  pxr::SdfPath id = ObjectData::prim_id(this, object);
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    obj_data->update();
    return;
  }
  if (view3d && !BKE_object_is_visible_in_viewport(view3d, object)) {
    return;
  }
  objects[id] = ObjectData::create(this, object);
  obj_data = object_data(id);
  obj_data->update_visibility(view3d);
}

void BlenderSceneDelegate::add_update_instancer(Object *object)
{
  pxr::SdfPath id = InstancerData::prim_id(this, object);
  InstancerData *i_data = instancer_data(id, true);
  if (i_data) {
    i_data->update();
    return;
  }
  objects[id] = InstancerData::create(this, object);
  i_data = instancer_data(id, true);
  i_data->update_visibility(view3d);
}

ObjectData *BlenderSceneDelegate::object_data(pxr::SdfPath const &id)
{
  auto it = objects.find(id);
  if (it == objects.end()) {
    return nullptr;
  }
  return it->second.get();
}

MeshData *BlenderSceneDelegate::mesh_data(pxr::SdfPath const &id)
{
  return dynamic_cast<MeshData *>(object_data(id));
}

LightData *BlenderSceneDelegate::light_data(pxr::SdfPath const &id)
{
  return dynamic_cast<LightData *>(object_data(id));
}

MaterialData *BlenderSceneDelegate::material_data(pxr::SdfPath const &id)
{
  auto it = materials.find(id);
  if (it == materials.end()) {
    return nullptr;
  }
  return it->second.get();
}

InstancerData *BlenderSceneDelegate::instancer_data(pxr::SdfPath const &id, bool base_prim)
{
  if (base_prim) {
    return dynamic_cast<InstancerData *>(object_data(id));
  }
  return dynamic_cast<InstancerData *>(object_data(id.GetParentPath()));
}

InstancerData *BlenderSceneDelegate::instancer_data(Object *object)
{
  InstancerData *i_data;
  for (auto &it : objects) {
    i_data = dynamic_cast<InstancerData *>(it.second.get());
    if (i_data && i_data->is_base(object)) {
      return i_data;
    }
  }
  return nullptr;
}

void BlenderSceneDelegate::populate(Depsgraph *deps, bContext *cont)
{
  bool is_populated = depsgraph != nullptr;

  depsgraph = deps;
  context = cont;
  scene = DEG_get_input_scene(depsgraph);
  view3d = CTX_wm_view3d(context);

  if (!is_populated) {
    /* Export initial objects */
    update_collection(false, false);
    update_world();
    return;
  }

  /* Working with updates */
  bool do_update_collection = false;
  bool do_update_visibility = false;
  bool do_update_world = false;

  unsigned int scene_recalc = ((ID *)scene)->recalc;
  if (scene_recalc) {
    /* Checking scene updates */
    CLOG_INFO(LOG_BSD,
              2,
              "Update: %s [%s]",
              ((ID *)scene)->name,
              std::bitset<32>(scene_recalc).to_string().c_str());

    if (scene_recalc & ID_RECALC_BASE_FLAGS) {
      do_update_visibility = true;
    }
    if (scene_recalc & (ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY)) {
      do_update_collection = true;
    }
    if (scene_recalc & ID_RECALC_AUDIO_VOLUME) {
      if ((scene->world && !world_data) || (!scene->world && world_data)) {
        do_update_world = true;
      }
    }
    if (do_update_collection || do_update_visibility) {
      update_collection(do_update_collection, do_update_visibility);
    }
  }

  /* Checking other objects updates */
  DEGIDIterData data = {0};
  data.graph = depsgraph;
  data.only_updated = true;
  ITER_BEGIN (
      DEG_iterator_ids_begin, DEG_iterator_ids_next, DEG_iterator_ids_end, &data, ID *, id) {

    CLOG_INFO(
        LOG_BSD, 2, "Update: %s [%s]", id->name, std::bitset<32>(id->recalc).to_string().c_str());

    switch (GS(id->name)) {
      case ID_OB: {
        Object *object = (Object *)id;
        if (!ObjectData::supported(object)) {
          break;
        }
        add_update_object(object);
      } break;

      case ID_MA: {
        MaterialData *mat_data = material_data(MaterialData::prim_id(this, (Material *)id));
        if (mat_data) {
          mat_data->update();
        }
      } break;

      case ID_WO: {
        if (id->recalc & ID_RECALC_SHADING) {
          do_update_world = true;
        }
      } break;

      default:
        break;
    }
  }
  ITER_END;

  if (do_update_world) {
    update_world();
  }
}

pxr::HdMeshTopology BlenderSceneDelegate::GetMeshTopology(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_BSD, 3, "%s", id.GetText());
  MeshData *m_data = mesh_data(id);
  return m_data->mesh_topology();
}

pxr::VtValue BlenderSceneDelegate::Get(pxr::SdfPath const &id, pxr::TfToken const &key)
{
  CLOG_INFO(LOG_BSD, 3, "%s, %s", id.GetText(), key.GetText());

  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    return obj_data->get_data(key);
  }

  MaterialData *mat_data = material_data(id);
  if (mat_data) {
    return mat_data->get_data(key);
  }

  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    return i_data->get_data(key);
  }

  return pxr::VtValue();
}

pxr::HdPrimvarDescriptorVector BlenderSceneDelegate::GetPrimvarDescriptors(
    pxr::SdfPath const &id, pxr::HdInterpolation interpolation)
{
  CLOG_INFO(LOG_BSD, 3, "%s, %d", id.GetText(), interpolation);

  MeshData *m_data = mesh_data(id);
  if (m_data) {
    return m_data->primvar_descriptors(interpolation);
  }

  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    return i_data->instancer_primvar_descriptors(interpolation);
  }

  return pxr::HdPrimvarDescriptorVector();
}

pxr::SdfPath BlenderSceneDelegate::GetMaterialId(pxr::SdfPath const &rprim_id)
{
  return mesh_data(rprim_id)->material_id();
}

pxr::VtValue BlenderSceneDelegate::GetMaterialResource(pxr::SdfPath const &id)
{
  MaterialData *mat_data = material_data(id);
  if (mat_data) {
    return mat_data->material_resource();
  }
  return pxr::VtValue();
}

pxr::GfMatrix4d BlenderSceneDelegate::GetTransform(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_BSD, 3, "%s", id.GetText());
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    return obj_data->transform();
  }

  if (id == WorldData::prim_id(this)) {
    return world_data->transform();
  }

  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    return i_data->transform();
  }

  return pxr::GfMatrix4d();
}

pxr::VtValue BlenderSceneDelegate::GetLightParamValue(pxr::SdfPath const &id,
                                                      pxr::TfToken const &key)
{
  LightData *l_data = light_data(id);
  if (l_data) {
    return l_data->get_data(key);
  }
  if (id == WorldData::prim_id(this)) {
    return world_data->get_data(key);
  }
  return pxr::VtValue();
}

}  // namespace blender::render::hydra
