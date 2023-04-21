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

MaterialData::MaterialData(BlenderSceneDelegate *scene_delegate,
                           Material *material,
                           pxr::SdfPath const &prim_id)
    : IdData(scene_delegate, (ID *)material, prim_id)
{
}

std::unique_ptr<MaterialData> MaterialData::create(BlenderSceneDelegate *scene_delegate,
                                                   Material *material,
                                                   pxr::SdfPath const &prim_id)
{
  auto data = std::make_unique<MaterialData>(scene_delegate, material, prim_id);
  data->init();
  data->insert();
  return data;
}

void MaterialData::init()
{
  ID_LOG(2, "");

  material_network_map_ = pxr::VtValue();

  /* Call of python function hydra.export_mtlx() */

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject *module, *dict, *func, *result;
  module = PyImport_ImportModule("bpy_hydra");
  dict = PyModule_GetDict(module);
  func = PyDict_GetItemString(dict, "export_mtlx");

  PointerRNA materialptr;
  RNA_pointer_create(NULL, &RNA_Material, id, &materialptr);
  PyObject *material = pyrna_struct_CreatePyObject(&materialptr);

  result = PyObject_CallFunction(func, "O", material);

  Py_DECREF(material);

  std::string path;

  if (!PyErr_Occurred()) {
    path = PyUnicode_AsUTF8(result);
    Py_DECREF(result);
  }
  else {
    CLOG_ERROR(LOG_RENDER_HYDRA_SCENE, "Export error for %s", id->name);
    PyErr_Print();
  }
  Py_DECREF(module);

  PyGILState_Release(gstate);

  mtlx_path_ = pxr::SdfAssetPath(path, path);
  ID_LOG(2, "mtlx=%s", mtlx_path_.GetResolvedPath().c_str());

  /* Calculate material network map */
  if (!path.empty()) {
    pxr::HdRenderDelegate *render_delegate = scene_delegate_->GetRenderIndex().GetRenderDelegate();
    pxr::TfTokenVector shader_source_types = render_delegate->GetShaderSourceTypes();
    pxr::TfTokenVector render_contexts = render_delegate->GetMaterialRenderContexts();

    pxr::HdMaterialNetworkMap network_map;
    hdmtlx_convert_to_materialnetworkmap(path, shader_source_types, render_contexts, &network_map);

    material_network_map_ = network_map;
  }
  else {
    material_network_map_ = pxr::VtValue();
  }
}

void MaterialData::insert()
{
  ID_LOG(2, "");
  scene_delegate_->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->material, scene_delegate_, prim_id);
}

void MaterialData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 2, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->material, prim_id);
}

void MaterialData::update()
{
  ID_LOG(2, "");
  init();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id,
                                                                      pxr::HdMaterial::AllDirty);
}

pxr::VtValue MaterialData::get_data(pxr::TfToken const &key) const
{
  pxr::VtValue ret;
  if (key.GetString() == "MaterialXFilename") {
    if (!mtlx_path_.GetResolvedPath().empty()) {
      ret = mtlx_path_;
    }
    ID_LOG(3, "%s", key.GetText());
  }
  return ret;
}

pxr::VtValue MaterialData::get_material_resource() const
{
  return material_network_map_;
}

}  // namespace blender::render::hydra
