/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <bitset>

#include "DEG_depsgraph_query.h"
#include "DNA_scene_types.h"

#include "blender_scene_delegate.h"

namespace blender::render::hydra {

CLG_LOGREF_DECLARE_GLOBAL(LOG_RENDER_HYDRA_SCENE, "render.hydra.scene");

BlenderSceneDelegate::BlenderSceneDelegate(pxr::HdRenderIndex *parent_index,
                                           pxr::SdfPath const &delegate_id,
                                           Engine *engine)
    : HdSceneDelegate(parent_index, delegate_id), engine(engine)
{
}

pxr::HdMeshTopology BlenderSceneDelegate::GetMeshTopology(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  MeshData *m_data = mesh_data(id);
  return m_data->mesh_topology();
}

pxr::GfMatrix4d BlenderSceneDelegate::GetTransform(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  InstancerData *i_data = instancer_data(id, true);
  if (i_data) {
    return i_data->get_transform(id);
  }
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    return obj_data->transform;
  }
  if (id == world_prim_id()) {
    return world_data_->transform;
  }
  return pxr::GfMatrix4d();
}

pxr::VtValue BlenderSceneDelegate::Get(pxr::SdfPath const &id, pxr::TfToken const &key)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s, %s", id.GetText(), key.GetText());
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

pxr::VtValue BlenderSceneDelegate::GetLightParamValue(pxr::SdfPath const &id,
                                                      pxr::TfToken const &key)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s, %s", id.GetText(), key.GetText());
  LightData *l_data = light_data(id);
  if (l_data) {
    return l_data->get_data(key);
  }
  if (id == world_prim_id()) {
    return world_data_->get_data(key);
  }
  return pxr::VtValue();
}

pxr::HdPrimvarDescriptorVector BlenderSceneDelegate::GetPrimvarDescriptors(
    pxr::SdfPath const &id, pxr::HdInterpolation interpolation)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s, %d", id.GetText(), interpolation);
  MeshData *m_data = mesh_data(id);
  if (m_data) {
    return m_data->primvar_descriptors(interpolation);
  }
  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    return i_data->primvar_descriptors(interpolation);
  }
  return pxr::HdPrimvarDescriptorVector();
}

pxr::SdfPath BlenderSceneDelegate::GetMaterialId(pxr::SdfPath const &rprim_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", rprim_id.GetText());
  return mesh_data(rprim_id)->material_id();
}

pxr::VtValue BlenderSceneDelegate::GetMaterialResource(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  MaterialData *mat_data = material_data(id);
  if (mat_data) {
    return mat_data->get_material_resource();
  }
  return pxr::VtValue();
}

bool BlenderSceneDelegate::GetVisible(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  if (id == world_prim_id()) {
    return true;
  }
  InstancerData *i_data = instancer_data(id, true);
  if (i_data) {
    return i_data->visible;
  }
  return object_data(id)->visible;
}

bool BlenderSceneDelegate::GetDoubleSided(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  return mesh_data(id)->double_sided();
}

pxr::HdCullStyle BlenderSceneDelegate::GetCullStyle(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  return mesh_data(id)->cull_style;
}

pxr::SdfPath BlenderSceneDelegate::GetInstancerId(pxr::SdfPath const &prim_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", prim_id.GetText());
  InstancerData *i_data = instancer_data(prim_id.GetParentPath());
  if (i_data) {
    return i_data->prim_id;
  }
  return pxr::SdfPath();
}

pxr::SdfPathVector BlenderSceneDelegate::GetInstancerPrototypes(pxr::SdfPath const &instancer_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", instancer_id.GetText());
  InstancerData *i_data = instancer_data(instancer_id);
  return i_data->prototypes();
}

pxr::VtIntArray BlenderSceneDelegate::GetInstanceIndices(pxr::SdfPath const &instancer_id,
                                                         pxr::SdfPath const &prototype_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s, %s", instancer_id.GetText(), prototype_id.GetText());
  InstancerData *i_data = instancer_data(instancer_id);
  return i_data->indices(prototype_id);
}

pxr::GfMatrix4d BlenderSceneDelegate::GetInstancerTransform(pxr::SdfPath const &instancer_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", instancer_id.GetText());
  InstancerData *i_data = instancer_data(instancer_id);
  return i_data->transform;
}

void BlenderSceneDelegate::populate(Depsgraph *deps, bContext *cont)
{
  bool is_populated = depsgraph != nullptr;

  depsgraph = deps;
  context = cont;
  scene = DEG_get_input_scene(depsgraph);
  view3d = CTX_wm_view3d(context);

  if (is_populated) {
    check_updates();
  }
  else {
    add_new_objects();
    update_world();
  }
}

void BlenderSceneDelegate::clear()
{
  for (auto &it : objects_) {
    it.second->remove();
  }
  for (auto &it : instancers_) {
    it.second->remove();
  }
  for (auto &it : materials_) {
    it.second->remove();
  }

  objects_.clear();
  instancers_.clear();
  materials_.clear();

  depsgraph = nullptr;
  context = nullptr;
  scene = nullptr;
  view3d = nullptr;
}

void BlenderSceneDelegate::set_setting(const std::string &key, const pxr::VtValue &val)
{
  if (key == "MaterialXFilenameKey") {
    settings.mx_filename_key = pxr::TfToken(val.Get<std::string>());
  }
}

pxr::SdfPath BlenderSceneDelegate::prim_id(ID *id, const char *prefix) const
{
  /* Making id of object in form like <prefix>_<pointer in 16 hex digits format> */
  char str[32];
  snprintf(str, 32, "%s_%016llx", prefix, (uintptr_t)id);
  return GetDelegateID().AppendElementString(str);
}

pxr::SdfPath BlenderSceneDelegate::object_prim_id(Object *object) const
{
  return prim_id((ID *)object, "O");
}

pxr::SdfPath BlenderSceneDelegate::material_prim_id(Material *mat) const
{
  return prim_id((ID *)mat, "M");
}

pxr::SdfPath BlenderSceneDelegate::instancer_prim_id(Object *object) const
{
  return prim_id((ID *)object, "I");
}

pxr::SdfPath BlenderSceneDelegate::world_prim_id() const
{
  return GetDelegateID().AppendElementString("World");
}

ObjectData *BlenderSceneDelegate::object_data(pxr::SdfPath const &id) const
{
  auto it = objects_.find(id);
  if (it != objects_.end()) {
    return it->second.get();
  }
  InstancerData *i_data = instancer_data(id, true);
  if (i_data) {
    return i_data->object_data(id);
  }
  return nullptr;
}

MeshData *BlenderSceneDelegate::mesh_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<MeshData *>(object_data(id));
}

LightData *BlenderSceneDelegate::light_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<LightData *>(object_data(id));
}

MaterialData *BlenderSceneDelegate::material_data(pxr::SdfPath const &id) const
{
  auto it = materials_.find(id);
  if (it == materials_.end()) {
    return nullptr;
  }
  return it->second.get();
}

InstancerData *BlenderSceneDelegate::instancer_data(pxr::SdfPath const &id, bool child_id) const
{
  pxr::SdfPath p_id;
  if (child_id) {
    /* Getting instancer path id from child Mesh instance (consist with 3 path elements) and
     * Light instance (consist with 4 path elements) */
    int n = id.GetPathElementCount();
    if (n == 3) {
      p_id = id.GetParentPath();
    }
    else if (n == 4) {
      p_id = id.GetParentPath().GetParentPath();
    }
  }
  else {
    p_id = id;
  }

  auto it = instancers_.find(p_id);
  if (it != instancers_.end()) {
    return it->second.get();
  }
  return nullptr;
}

void BlenderSceneDelegate::update_objects(Object *object)
{
  if (!ObjectData::is_supported(object)) {
    return;
  }
  pxr::SdfPath id = object_prim_id(object);
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    obj_data->update_parent();
    obj_data->update();
    return;
  }
  if (view3d && !BKE_object_is_visible_in_viewport(view3d, object)) {
    return;
  }
  objects_[id] = ObjectData::create(this, object, id);
  obj_data = object_data(id);
  obj_data->update_parent();
  obj_data->init();
  obj_data->insert();
  obj_data->update_visibility();
}

void BlenderSceneDelegate::update_instancers(Object *object)
{
  /* Check object inside instancers */
  for (auto &it : instancers_) {
    it.second->check_update(object);
  }

  pxr::SdfPath id = instancer_prim_id(object);
  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    if (object->transflag & OB_DUPLI) {
      i_data->update();
    }
    else {
      i_data->remove();
      instancers_.erase(id);
    }
    return;
  }
  if ((object->transflag & OB_DUPLI) == 0) {
    return;
  }
  if (view3d && !BKE_object_is_visible_in_viewport(view3d, object)) {
    return;
  }
  instancers_[id] = std::make_unique<InstancerData>(this, object, id);
  i_data = instancer_data(id);
  i_data->init();
  i_data->insert();
  i_data->update_visibility();
}

void BlenderSceneDelegate::update_world()
{
  World *world = scene->world;
  if (!world_data_) {
    if (world) {
      world_data_ = std::make_unique<WorldData>(this, world, world_prim_id());
      world_data_->init();
      world_data_->insert();
    }
  }
  else {
    if (world) {
      world_data_->update(world);
    }
    else {
      world_data_->remove();
      world_data_ = nullptr;
    }
  }
}

void BlenderSceneDelegate::check_updates()
{
  bool do_update_collection = false;
  bool do_update_visibility = false;
  bool do_update_world = false;

  DEGIDIterData data = {0};
  data.graph = depsgraph;
  data.only_updated = true;
  ITER_BEGIN (DEG_iterator_ids_begin, DEG_iterator_ids_next, DEG_iterator_ids_end, &data, ID *, id)
  {

    CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
              2,
              "Update: %s [%s]",
              id->name,
              std::bitset<32>(id->recalc).to_string().c_str());

    switch (GS(id->name)) {
      case ID_OB: {
        Object *object = (Object *)id;
        update_objects(object);
        update_instancers(object);
      } break;

      case ID_MA: {
        MaterialData *mat_data = material_data(material_prim_id((Material *)id));
        if (mat_data) {
          mat_data->update();
        }
      } break;

      case ID_WO: {
        if (id->recalc & ID_RECALC_SHADING) {
          do_update_world = true;
        }
      } break;

      case ID_SCE: {
        if (id->recalc & ID_RECALC_BASE_FLAGS) {
          do_update_visibility = true;
        }
        if (id->recalc & (ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY)) {
          do_update_collection = true;
        }
        if (id->recalc & ID_RECALC_AUDIO_VOLUME) {
          if ((scene->world && !world_data_) || (!scene->world && world_data_)) {
            do_update_world = true;
          }
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
  if (do_update_collection) {
    remove_unused_objects();
  }
  if (do_update_visibility) {
    update_visibility();
  }
}

void BlenderSceneDelegate::add_new_objects()
{
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
              object)
  {

    update_objects(object);
    update_instancers(object);
  }
  ITER_END;
}

void BlenderSceneDelegate::remove_unused_objects()
{
  /* Get available objects */
  std::set<std::string> available_objects;

  DEGObjectIterSettings settings = {0};
  settings.depsgraph = depsgraph;
  settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY | DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET;
  DEGObjectIterData data = {0};
  data.settings = &settings;
  data.graph = settings.depsgraph;
  data.flag = settings.flags;
  ITER_BEGIN (DEG_iterator_objects_begin,
              DEG_iterator_objects_next,
              DEG_iterator_objects_end,
              &data,
              Object *,
              object)
  {
    if (ObjectData::is_supported(object)) {
      available_objects.insert(object_prim_id(object).GetName());
    }
    available_objects.insert(instancer_prim_id(object).GetName());
  }
  ITER_END;

  /* Remove unused instancers */
  for (auto it = instancers_.begin(); it != instancers_.end(); ++it) {
    if (available_objects.find(it->first.GetName()) != available_objects.end()) {
      /* Remove objects from instancers */
      it->second->check_remove(available_objects);
      continue;
    }
    it->second->remove();
    instancers_.erase(it);
    it = instancers_.begin();
  }

  /* Remove unused objects */
  for (auto it = objects_.begin(); it != objects_.end(); ++it) {
    if (available_objects.find(it->first.GetName()) != available_objects.end()) {
      continue;
    }
    it->second->remove();
    objects_.erase(it);
    it = objects_.begin();
  }

  /* Remove unused materials */
  std::set<pxr::SdfPath> available_materials;
  for (auto &it : objects_) {
    MeshData *m_data = dynamic_cast<MeshData *>(it.second.get());
    if (!m_data) {
      continue;
    }
    pxr::SdfPath mat_id = m_data->material_id();
    if (!mat_id.IsEmpty()) {
      available_materials.insert(mat_id);
    }
  }
  for (auto &it : instancers_) {
    it.second->available_materials(available_materials);
  }
  for (auto it = materials_.begin(); it != materials_.end(); ++it) {
    if (available_materials.find(it->first) != available_materials.end()) {
      continue;
    }
    it->second->remove();
    materials_.erase(it);
    it = materials_.begin();
  }
}

void BlenderSceneDelegate::update_visibility()
{
  for (auto &it : objects_) {
    it.second->update_visibility();
  }
  for (auto &it : instancers_) {
    it.second->update_visibility();
  }

  /* Add objects which were invisible before and not added yet */
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
              object)
  {

    if (!object_data(object_prim_id(object))) {
      update_objects(object);
    }
    if (!instancer_data(instancer_prim_id(object))) {
      update_instancers(object);
    }
  }
  ITER_END;
}

}  // namespace blender::render::hydra
