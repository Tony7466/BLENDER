/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <Python.h>

#include <pxr/imaging/hd/material.h>
#include <pxr/imaging/hd/renderDelegate.h>
#include <pxr/imaging/hd/tokens.h>

#include "glog/logging.h"

#include "BKE_lib_id.h"
#include "BKE_material.h"

#include "MEM_guardedalloc.h"
#include "RNA_blender_cpp.h"
#include "bpy_rna.h"

#include "blender_scene_delegate.h"
#include "material.h"
#include "mtlx_hydra_adapter.h"

namespace blender::render::hydra {

std::unique_ptr<MaterialData> MaterialData::init(BlenderSceneDelegate *scene_delegate,
                                                 Material *material)
{
  return std::make_unique<MaterialData>(scene_delegate, material);
}

pxr::SdfPath MaterialData::prim_id(BlenderSceneDelegate *scene_delegate, Material *material)
{
  /* Making id of material in form like M_<pointer in 16 hex digits format>.
   * Example: M_000002074e812088 */
  char str[32];
  snprintf(str, 32, "M_%016llx", (uint64_t)material);
  return scene_delegate->GetDelegateID().AppendElementString(str);
}

MaterialData::MaterialData(BlenderSceneDelegate *scene_delegate, Material *material)
    : IdData(scene_delegate, (ID *)material)
{
}

pxr::VtValue MaterialData::get_data(pxr::TfToken const &key)
{
  pxr::VtValue ret;
  if (key.GetString() == "MaterialXFilename") {
    if (!mtlx_path.GetResolvedPath().empty()) {
      ret = mtlx_path;
    }
  }
  return ret;
}

pxr::VtValue MaterialData::material_resource()
{
  std::string const &path = mtlx_path.GetResolvedPath();
  if (!path.empty()) {
    pxr::HdRenderDelegate *render_delegate = scene_delegate->GetRenderIndex().GetRenderDelegate();
    pxr::TfTokenVector shader_source_types = render_delegate->GetShaderSourceTypes();
    pxr::TfTokenVector render_contexts = render_delegate->GetMaterialRenderContexts();

    pxr::HdMaterialNetworkMap material_network_map;
    HdMtlxConvertToMaterialNetworkMap(
        path, shader_source_types, render_contexts, &material_network_map);
    return pxr::VtValue(material_network_map);
  }

  return pxr::VtValue();
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
  if (result) {
    path = PyUnicode_AsUTF8(result);
    Py_DECREF(result);
  }
  else {
    PyErr_Print();
  }
  Py_DECREF(module);

  PyGILState_Release(gstate);

  mtlx_path = pxr::SdfAssetPath(path, path);
  LOG(INFO) << "Material export: " << name() << " mtlx=" << mtlx_path.GetResolvedPath();
}

void MaterialData::insert_prim()
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().InsertSprim(
      pxr::HdPrimTypeTokens->material, scene_delegate, p_id);
  LOG(INFO) << "Add material: " << name() << " id=" << p_id.GetAsString();
}

void MaterialData::remove_prim()
{
  pxr::SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().RemoveSprim(pxr::HdPrimTypeTokens->material, p_id);
  LOG(INFO) << "Remove material: " << name();
}

void MaterialData::mark_prim_dirty(DirtyBits dirty_bits)
{
  pxr::HdDirtyBits bits = pxr::HdMaterial::Clean;
  switch (dirty_bits) {
    case DirtyBits::ALL_DIRTY:
      bits = pxr::HdMaterial::AllDirty;
      break;
    default:
      break;
  }
  pxr::SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
  LOG(INFO) << "Update material: " << name() << ", mtlx=" << mtlx_path.GetResolvedPath();
}

}  // namespace blender::render::hydra
