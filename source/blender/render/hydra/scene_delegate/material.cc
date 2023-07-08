/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <Python.h>
#include <unicodeobject.h>

#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/tokens.h>

#include "BKE_lib_id.h"
#include "BKE_material.h"

#include "MEM_guardedalloc.h"
#include "RNA_blender_cpp.h"
#include "bpy_rna.h"

#include "../engine.h"
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

void MaterialData::init()
{
  ID_LOG(1, "");
  double_sided = (((Material *)id)->blend_flag & MA_BL_CULL_BACKFACE) == 0;

  export_mtlx();
  if (scene_delegate_->settings.mx_filename_key.IsEmpty()) {
    write_material_network_map();
  }
}

void MaterialData::insert()
{
  ID_LOG(1, "");
  scene_delegate_->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->material, scene_delegate_, prim_id);
}

void MaterialData::remove()
{
  CLOG_INFO(LOG_RENDER_HYDRA_SCENE, 1, "%s", prim_id.GetText());
  scene_delegate_->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->material, prim_id);
}

void MaterialData::update()
{
  ID_LOG(1, "");
  bool prev_double_sided = double_sided;
  init();
  scene_delegate_->GetRenderIndex().GetChangeTracker().MarkSprimDirty(prim_id,
                                                                      pxr::HdMaterial::AllDirty);
  if (prev_double_sided != double_sided) {
    for (auto &obj_data : scene_delegate_->objects_.values()) {
      MeshData *m_data = dynamic_cast<MeshData *>(obj_data.get());
      if (m_data) {
        m_data->update_double_sided(this);
      }
    }
    scene_delegate_->instancer_data_->update_double_sided(this);
  }
}

pxr::VtValue MaterialData::get_data(pxr::TfToken const &key) const
{
  if (key == scene_delegate_->settings.mx_filename_key) {
    ID_LOG(3, "%s", key.GetText());
    if (!mtlx_path_.GetResolvedPath().empty()) {
      return pxr::VtValue(mtlx_path_);
    }
  }
  return pxr::VtValue();
}

pxr::VtValue MaterialData::get_material_resource() const
{
  return material_network_map_;
}

void MaterialData::export_mtlx()
{
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
    if (PyUnicode_Check(result)) {
      path = PyUnicode_AsUTF8(result);
    }
    Py_DECREF(result);
  }
  else {
    /* Clearing and logging exception data */
    PyObject *type, *value, *traceback;
    PyErr_Fetch(&type, &value, &traceback);
    PyErr_NormalizeException(&type, &value, &traceback);
    std::string err_str = ((PyTypeObject *)type)->tp_name;
    if (value) {
      PyObject *pstr = PyObject_Str(value);
      err_str += ": ";
      err_str += PyUnicode_AsUTF8(pstr);
      Py_DECREF(pstr);
    }
    CLOG_ERROR(LOG_RENDER_HYDRA_SCENE,
               "Export error for %s (%s): %s",
               prim_id.GetText(),
               id->name,
               err_str.c_str());
    if (traceback) {
      PyTraceBack_Print(traceback, PySys_GetObject("stderr"));
    }
    Py_XDECREF(traceback);
    Py_XDECREF(value);
    Py_DECREF(type);
  }
  Py_DECREF(module);

  PyGILState_Release(gstate);

  mtlx_path_ = pxr::SdfAssetPath(path, path);
  ID_LOG(1, "mtlx=%s", mtlx_path_.GetResolvedPath().c_str());
}

void MaterialData::write_material_network_map()
{
  ID_LOG(1, "");
  if (mtlx_path_.GetResolvedPath().empty()) {
    material_network_map_ = pxr::VtValue();
    return;
  }

  pxr::HdRenderDelegate *render_delegate = scene_delegate_->GetRenderIndex().GetRenderDelegate();
  pxr::TfTokenVector shader_source_types = render_delegate->GetShaderSourceTypes();
  pxr::TfTokenVector render_contexts = render_delegate->GetMaterialRenderContexts();

  pxr::HdMaterialNetworkMap network_map;
  hdmtlx_convert_to_materialnetworkmap(
      mtlx_path_.GetResolvedPath(), shader_source_types, render_contexts, &network_map);

  material_network_map_ = network_map;
}

}  // namespace blender::render::hydra
