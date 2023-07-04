/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <bitset>

#include "BKE_object.h"
#include "BLI_set.hh"
#include "DEG_depsgraph_query.h"
#include "DNA_scene_types.h"

#include "blender_scene_delegate.h"

namespace blender::render::hydra {

CLG_LOGREF_DECLARE_GLOBAL(LOG_RENDER_HYDRA_SCENE, "render.hydra.scene");

bool BlenderSceneDelegate::ShadingSettings::operator==(const ShadingSettings &other)
{
  bool ret = use_scene_lights == other.use_scene_lights &&
             use_scene_world == other.use_scene_world;
  if (ret && !use_scene_world) {
    /* compare studiolight settings when studiolight is using */
    ret = studiolight_name == other.studiolight_name &&
          studiolight_rotation == other.studiolight_rotation &&
          studiolight_intensity == other.studiolight_intensity;
  }
  return ret;
}

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
  return m_data->mesh_topology(id);
}

pxr::HdBasisCurvesTopology BlenderSceneDelegate::GetBasisCurvesTopology(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  CurvesData *c_data = curves_data(id);
  return c_data->curves_topology(id);
};

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
  MeshData *m_data = mesh_data(id);
  if (m_data) {
    return m_data->get_data(id, key);
  }
  CurvesData *c_data = curves_data(id);
  if (c_data) {
    return c_data->get_data(id, key);
  }
  VolumeData *v_data = volume_data(id);
  if (v_data) {
    return v_data->get_data(id, key);
  }
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
  CurvesData *c_data = curves_data(id);
  if (c_data) {
    return c_data->primvar_descriptors(interpolation);
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
  MeshData *m_data = mesh_data(rprim_id);
  if (m_data) {
    return m_data->material_id(rprim_id);
  }
  CurvesData *c_data = curves_data(rprim_id);
  if (c_data) {
    return c_data->material_id();
  }
  VolumeData *v_data = volume_data(rprim_id);
  if (v_data) {
    return v_data->material_id();
  }
  return pxr::SdfPath();
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
  return mesh_data(id)->double_sided(id);
}

pxr::HdCullStyle BlenderSceneDelegate::GetCullStyle(pxr::SdfPath const &id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", id.GetText());
  return mesh_data(id)->cull_style;
}

pxr::SdfPath BlenderSceneDelegate::GetInstancerId(pxr::SdfPath const &prim_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", prim_id.GetText());
  InstancerData *i_data = instancer_data(prim_id, true);
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

pxr::HdVolumeFieldDescriptorVector BlenderSceneDelegate::GetVolumeFieldDescriptors(
    pxr::SdfPath const &volume_id)
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 3, "%s", volume_id.GetText());
  VolumeData *v_data = volume_data(volume_id);
  return v_data->field_descriptors();
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
    set_light_shading_settings();
    set_world_shading_settings();
    add_new_objects();
    update_world();
  }
}

void BlenderSceneDelegate::clear()
{
  for (auto &obj_data : objects_.values()) {
    obj_data->remove();
  }
  for (auto &i_data : instancers_.values()) {
    i_data->remove();
  }
  for (auto &mat_data : materials_.values()) {
    mat_data->remove();
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
  else {
    settings.render_tokens.add_overwrite(pxr::TfToken(key), val);
  }
}

pxr::SdfPath BlenderSceneDelegate::prim_id(ID *id, const char *prefix) const
{
  /* Making id of object in form like <prefix>_<pointer in 16 hex digits format> */
  char name[32];
  snprintf(name, sizeof(name), "%s_%016llx", prefix, (uintptr_t)id);
  return GetDelegateID().AppendElementString(name);
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
  pxr::SdfPath p_id = (id.GetName().find("SM_") == 0 || id.GetName().find("VF_") == 0) ?
                          id.GetParentPath() :
                          id;
  auto obj_data = objects_.lookup_ptr(p_id);
  if (obj_data) {
    return obj_data->get();
  }
  InstancerData *i_data = instancer_data(p_id, true);
  if (i_data) {
    return i_data->object_data(id);
  }
  return nullptr;
}

MeshData *BlenderSceneDelegate::mesh_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<MeshData *>(object_data(id));
}

CurvesData *BlenderSceneDelegate::curves_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<CurvesData *>(object_data(id));
}

LightData *BlenderSceneDelegate::light_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<LightData *>(object_data(id));
}

MaterialData *BlenderSceneDelegate::material_data(pxr::SdfPath const &id) const
{
  auto mat_data = materials_.lookup_ptr(id);
  if (!mat_data) {
    return nullptr;
  }
  return mat_data->get();
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

  auto i_data = instancers_.lookup_ptr(p_id);
  if (i_data) {
    return i_data->get();
  }
  return nullptr;
}

VolumeData *BlenderSceneDelegate::volume_data(pxr::SdfPath const &id) const
{
  return dynamic_cast<VolumeData *>(object_data(id));
}

void BlenderSceneDelegate::update_objects(Object *object)
{
  if (!ObjectData::is_supported(object)) {
    return;
  }
  if (!shading_settings.use_scene_lights && object->type == OB_LAMP) {
    return;
  }
  pxr::SdfPath id = object_prim_id(object);
  ObjectData *obj_data = object_data(id);
  if (obj_data) {
    obj_data->update_parent();
    obj_data->update();
    obj_data->update_visibility();
    return;
  }

  if (!ObjectData::is_visible(this, object)) {
    /* Do not export new object if it is invisible */
    return;
  }

  objects_.add_new(id, ObjectData::create(this, object, id));
  obj_data = object_data(id);
  obj_data->update_parent();
  obj_data->init();
  obj_data->insert();
}

void BlenderSceneDelegate::update_instancers(Object *object)
{
  /* Check object inside instancers */
  for (auto &i_data : instancers_.values()) {
    i_data->check_update(object);
  }

  pxr::SdfPath id = instancer_prim_id(object);
  InstancerData *i_data = instancer_data(id);
  if (i_data) {
    if ((object->transflag & OB_DUPLI) == 0) {
      /* Object isn't instancer anymore and should be removed */
      i_data->remove();
      instancers_.remove(id);
      return;
    }

    i_data->update();
    return;
  }

  if ((object->transflag & OB_DUPLI) == 0) {
    return;
  }

  if (!InstancerData::is_visible(this, object)) {
    /* Do not export new instancer if it is invisible */
    return;
  }

  i_data = instancers_.lookup_or_add(id, std::make_unique<InstancerData>(this, object, id)).get();
  i_data->init();
  i_data->insert();
}

void BlenderSceneDelegate::update_world()
{
  if (!world_data_) {
    if (!shading_settings.use_scene_world || (shading_settings.use_scene_world && scene->world)) {
      world_data_ = std::make_unique<WorldData>(this, world_prim_id());
      world_data_->init();
      world_data_->insert();
    }
  }
  else {
    if (!shading_settings.use_scene_world || (shading_settings.use_scene_world && scene->world)) {
      world_data_->update();
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

  if (set_world_shading_settings()) {
    do_update_world = true;
  }

  if (set_light_shading_settings()) {
    if (shading_settings.use_scene_lights) {
      add_new_objects();
    }
    else {
      do_update_collection = true;
    }
  }

  DEGIDIterData data = {0};
  data.graph = depsgraph;
  data.only_updated = true;
  eEvaluationMode deg_mode = DEG_get_mode(depsgraph);

  ITER_BEGIN (DEG_iterator_ids_begin, DEG_iterator_ids_next, DEG_iterator_ids_end, &data, ID *, id)
  {
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
              0,
              "Update: %s [%s]",
              id->name,
              std::bitset<32>(id->recalc).to_string().c_str());

    switch (GS(id->name)) {
      case ID_OB: {
        Object *object = (Object *)id;
        CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
                  2,
                  "Visibility: %s [%s]",
                  object->id.name,
                  std::bitset<3>(BKE_object_visibility(object, deg_mode)).to_string().c_str());
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
        if (shading_settings.use_scene_world && id->recalc & ID_RECALC_SHADING) {
          do_update_world = true;
        }
      } break;

      case ID_SCE: {
        if (id->recalc & ID_RECALC_COPY_ON_WRITE && !(id->recalc & ID_RECALC_SELECT)) {
          do_update_collection = true;
          do_update_visibility = true;
        }
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
  settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY | DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET;
  DEGObjectIterData data = {0};
  data.settings = &settings;
  data.graph = settings.depsgraph;
  data.flag = settings.flags;
  eEvaluationMode deg_mode = DEG_get_mode(depsgraph);

  ITER_BEGIN (DEG_iterator_objects_begin,
              DEG_iterator_objects_next,
              DEG_iterator_objects_end,
              &data,
              Object *,
              object)
  {
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
              2,
              "Visibility: %s [%s]",
              object->id.name,
              std::bitset<3>(BKE_object_visibility(object, deg_mode)).to_string().c_str());
    if (object_data(object_prim_id(object))) {
      continue;
    }
    update_objects(object);
    update_instancers(object);
  }
  ITER_END;
}

void BlenderSceneDelegate::remove_unused_objects()
{
  /* Get available objects */
  Set<std::string> available_objects;

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
    available_objects.add(instancer_prim_id(object).GetName());
    if (ObjectData::is_supported(object)) {
      if (!shading_settings.use_scene_lights && object->type == OB_LAMP) {
        continue;
      }
      available_objects.add(object_prim_id(object).GetName());
    }
  }
  ITER_END;

  /* Remove unused instancers */
  instancers_.remove_if([&](auto item) {
    bool ret = !available_objects.contains(item.key.GetName());
    if (ret) {
      item.value->remove();
    }
    else {
      item.value->check_remove(available_objects);
    }
    return ret;
  });

  /* Remove unused objects */
  objects_.remove_if([&](auto item) {
    bool ret = !available_objects.contains(item.key.GetName());
    if (ret) {
      item.value->remove();
    }
    return ret;
  });

  /* Remove unused materials */
  Set<pxr::SdfPath> available_materials;
  for (auto &val : objects_.values()) {
    MeshData *m_data = dynamic_cast<MeshData *>(val.get());
    if (m_data) {
      m_data->available_materials(available_materials);
    }
    CurvesData *c_data = dynamic_cast<CurvesData *>(val.get());
    if (c_data) {
      c_data->available_materials(available_materials);
    }
    VolumeData *v_data = dynamic_cast<VolumeData *>(val.get());
    if (v_data) {
      v_data->available_materials(available_materials);
    }
  }
  for (auto &val : instancers_.values()) {
    val->available_materials(available_materials);
  }

  materials_.remove_if([&](auto item) {
    bool ret = !available_materials.contains(item.key);
    if (ret) {
      item.value->remove();
    }
    return ret;
  });
}

void BlenderSceneDelegate::update_visibility()
{
  /* Updating visibility of existing objects/instancers */
  for (auto &val : objects_.values()) {
    val->update_visibility();
  }
  for (auto &val : instancers_.values()) {
    val->update_visibility();
  }

  /* Add objects/instancers which were invisible before and not added yet */
  DEGObjectIterSettings settings = {0};
  settings.depsgraph = depsgraph;
  settings.flags = DEG_ITER_OBJECT_FLAG_LINKED_DIRECTLY | DEG_ITER_OBJECT_FLAG_LINKED_VIA_SET;
  DEGObjectIterData data = {0};
  data.settings = &settings;
  data.graph = settings.depsgraph;
  data.flag = settings.flags;
  eEvaluationMode deg_mode = DEG_get_mode(depsgraph);

  ITER_BEGIN (DEG_iterator_objects_begin,
              DEG_iterator_objects_next,
              DEG_iterator_objects_end,
              &data,
              Object *,
              object)
  {
    CLOG_INFO(LOG_RENDER_HYDRA_SCENE,
              2,
              "Visibility: %s [%s]",
              object->id.name,
              std::bitset<3>(BKE_object_visibility(object, deg_mode)).to_string().c_str());
    if (!object_data(object_prim_id(object))) {
      update_objects(object);
    }
    if (!instancer_data(instancer_prim_id(object))) {
      update_instancers(object);
    }
  }
  ITER_END;
}

bool BlenderSceneDelegate::set_light_shading_settings()
{
  if (!view3d) {
    return false;
  }
  ShadingSettings prev_settings(shading_settings);
  shading_settings.use_scene_lights = V3D_USES_SCENE_LIGHTS(view3d);
  return !(shading_settings == prev_settings);
}

bool BlenderSceneDelegate::set_world_shading_settings()
{
  if (!view3d) {
    return false;
  }
  ShadingSettings prev_settings(shading_settings);
  shading_settings.use_scene_world = V3D_USES_SCENE_WORLD(view3d);
  shading_settings.studiolight_name = view3d->shading.lookdev_light;
  shading_settings.studiolight_rotation = view3d->shading.studiolight_rot_z;
  shading_settings.studiolight_intensity = view3d->shading.studiolight_intensity;
  return !(shading_settings == prev_settings);
}

}  // namespace blender::render::hydra
