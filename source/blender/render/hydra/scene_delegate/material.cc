/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <Python.h>

#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_lib_id.h"
#include "BKE_material.h"

#include "MEM_guardedalloc.h"
#include "RNA_blender_cpp.h"
#include "bpy_rna.h"

#include "blender_scene_delegate.h"
#include "material.h"
#include "mtlx_hydra_adapter.h"

namespace blender::render::hydra {

MaterialData::MaterialData(BlenderSceneDelegate *scene_delegate, Material *material)
    : IdData(scene_delegate, (ID *)material)
{
  p_id_ = prim_id(scene_delegate, material);
  CLOG_INFO(LOG_BSD, 2, "%s, id=%s", id_->name, p_id_.GetText());
}

std::unique_ptr<MaterialData> MaterialData::create(BlenderSceneDelegate *scene_delegate,
                                                   Material *material)
{
  auto data = std::make_unique<MaterialData>(scene_delegate, material);
  data->init();
  data->insert();
  return data;
}

pxr::SdfPath MaterialData::prim_id(BlenderSceneDelegate *scene_delegate, Material *material)
{
  /* Making id of material in form like M_<pointer in 16 hex digits format>.
   * Example: M_000002074e812088 */
  char str[32];
  snprintf(str, 32, "M_%016llx", (uint64_t)material);
  return scene_delegate->GetDelegateID().AppendElementString(str);
}

void MaterialData::init()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id_->name);

  material_network_map_ = pxr::VtValue();

  /* Call of python function hydra.export_mtlx() */

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject *module, *dict, *func, *result;
  module = PyImport_ImportModule("bpy_hydra");
  dict = PyModule_GetDict(module);
  func = PyDict_GetItemString(dict, "export_mtlx");

  PointerRNA materialptr;
  RNA_pointer_create(NULL, &RNA_Material, id_, &materialptr);
  PyObject *material = pyrna_struct_CreatePyObject(&materialptr);

  result = PyObject_CallFunction(func, "O", material);

  Py_DECREF(material);

  std::string path;

  if (!PyErr_Occurred()) {
    path = PyUnicode_AsUTF8(result);
    Py_DECREF(result);
  }
  else {
    CLOG_ERROR(LOG_BSD, "Export error for %s", id_->name);
    PyErr_Print();
  }
  Py_DECREF(module);

  PyGILState_Release(gstate);

  mtlx_path_ = pxr::SdfAssetPath(path, path);
  CLOG_INFO(LOG_BSD, 2, "Export: %s, mtlx=%s", id_->name, mtlx_path_.GetResolvedPath().c_str());
}

void MaterialData::insert()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id_->name);
  scene_delegate_->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->material, scene_delegate_, p_id_);
}

void MaterialData::remove()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id_->name);
  scene_delegate_->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->material, p_id_);
}

void MaterialData::update()
{
  CLOG_INFO(LOG_BSD, 2, "%s", id_->name);
  init();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id_,
                                                                      pxr::HdMaterial::AllDirty);
}

pxr::VtValue MaterialData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  if (key.GetString() == "MaterialXFilename") {
    if (!mtlx_path_.GetResolvedPath().empty()) {
      ret = mtlx_path_;
    }
    CLOG_INFO(LOG_BSD, 3, "%s", key.GetText());
  }
  return ret;
}

pxr::VtValue MaterialData::get_material_resource()
{
  if (material_network_map_.IsEmpty()) {
    const std::string &path = mtlx_path_.GetResolvedPath();
    if (!path.empty()) {
      pxr::HdRenderDelegate *render_delegate =
          scene_delegate_->GetRenderIndex().GetRenderDelegate();
      pxr::TfTokenVector shader_source_types = render_delegate->GetShaderSourceTypes();
      pxr::TfTokenVector render_contexts = render_delegate->GetMaterialRenderContexts();

      pxr::HdMaterialNetworkMap network_map;
      hdmtlx_convert_to_materialnetworkmap(
          path, shader_source_types, render_contexts, &network_map);

      material_network_map_ = network_map;
    }
  }
  return material_network_map_;
}

}  // namespace blender::render::hydra
