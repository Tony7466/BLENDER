/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "DEG_depsgraph_query.h"
#include "DNA_scene_types.h"

#include "glog/logging.h"

#include "blenderSceneDelegate.h"

using namespace pxr;

namespace blender::render::hydra {

BlenderSceneDelegate::BlenderSceneDelegate(HdRenderIndex* parentIndex, SdfPath const& delegateID, BlenderSceneDelegate::EngineType engine_type)
  : HdSceneDelegate(parentIndex, delegateID)
  , engine_type(engine_type)
  , depsgraph(nullptr)
  , context(nullptr)
  , view3d(nullptr)
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
  Scene *scene = DEG_get_input_scene(depsgraph);
  World *world = scene->world;
  if (!world_data) {
    if (world) {
      world_data = WorldData::init(this, world, context);
      world_data->insert_prim();
    }
  }
  else {
    if (world) {
      world_data = WorldData::init(this, world, context);
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

void BlenderSceneDelegate::update_collection(bool remove, bool visibility)
{
  if (visibility) {
    /* Check and update visibility */
    for (auto &obj : objects) {
      if (obj.second->update_visibility(view3d)) {
        obj.second->mark_prim_dirty(IdData::DirtyBits::DirtyVisibility);
      };
    }
  }

  /* Export of new visible objects which were not exported before */
  std::set<SdfPath> available_objects;

  DEGObjectIterSettings settings = {0};
  settings.depsgraph = depsgraph;
  settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY | DEG_ITER_OBJECT_FLAG_VISIBLE |
                   DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET | DEG_ITER_OBJECT_FLAG_DUPLI;
  DEGObjectIterData data = {0};
  data.settings = &settings;
  data.graph = settings.depsgraph;
  data.flag = settings.flags;
  ITER_BEGIN (DEG_iterator_objects_begin,
              DEG_iterator_objects_next,
              DEG_iterator_objects_end,
              &data, Object *, object) {
    if (data.dupli_object_current != nullptr) {
      InstanceData i_data(this, data.dupli_object_current);
      LOG(INFO) << "Instance: " << i_data.name() << " " << i_data.random_id();
      continue;
    }
    if (!ObjectData::supported(object)) {
      continue;
    }

    SdfPath id = ObjectData::prim_id(this, object);
    if (remove) {
      available_objects.insert(id);
    }

    if (!object_data(id)) {
      add_update_object(object, true, true, true);
    }
  }
  ITER_END;

  if (remove) {
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

void BlenderSceneDelegate::populate(Depsgraph *deps, bContext *cont)
{
  bool is_populated = depsgraph != nullptr;

  depsgraph = deps;
  context = cont;
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
  bool transform, geometry, shading;

  DEGIDIterData data = {0};
  data.graph = depsgraph;
  data.only_updated = true;
  ITER_BEGIN (DEG_iterator_ids_begin,
              DEG_iterator_ids_next,
              DEG_iterator_ids_end,
              &data, ID *, id) {

    transform = (id->recalc & ID_RECALC_TRANSFORM) != 0;
    shading = (id->recalc & (ID_RECALC_SHADING | ID_RECALC_ANIMATION)) != 0;
    geometry = (id->recalc & ID_RECALC_GEOMETRY) != 0;

    LOG(INFO) << "Update: " << id->name << " [" << transform << geometry << shading << "]";

    switch (GS(id->name)) {
      case ID_OB: {
          Object *object = (Object *)id;
          if (!ObjectData::supported(object)) {
            break;
          }
          geometry |= (((ID *)object->data)->recalc & ID_RECALC_GEOMETRY) != 0;
          add_update_object(object, geometry, transform, shading);
        }
        break;

      case ID_MA:
        if (shading) {
          Material *material = (Material *)id;
          update_material(material);
        }
        break;

      case ID_GR:
        if (transform && geometry) {
          do_update_collection = true;
        }
        break;

      case ID_SCE:
        if (!geometry && !transform && !shading) {
          Scene *scene = (Scene *)id;
          do_update_visibility = true;

          if ((scene->world && !world_data) || (!scene->world && world_data)) {
            do_update_world = true;
          }
        }
        break;

      case ID_WO:
        if (shading) {
          do_update_world = true;
        }
        break;

      default:
        break;
    }
  }
  ITER_END;

  if (do_update_collection || do_update_visibility) {
    update_collection(do_update_collection, do_update_visibility);
  }
  if (do_update_world) {
    update_world();
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
