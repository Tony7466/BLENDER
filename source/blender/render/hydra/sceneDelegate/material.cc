/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <Python.h>

#include <pxr/imaging/hd/tokens.h>
#include <pxr/imaging/hd/material.h>

#include "glog/logging.h"

#include "BKE_material.h"
#include "BKE_lib_id.h"

#include "material.h"

using namespace pxr;

namespace blender::render::hydra {

std::unique_ptr<MaterialData> MaterialData::init(pxr::HdSceneDelegate *scene_delegate, Material *material)
{
  return std::make_unique<MaterialData>(scene_delegate, material);
}

pxr::SdfPath MaterialData::prim_id(pxr::HdSceneDelegate *scene_delegate, Material *material)
{
  /* Making id of material in form like M_<pointer in 16 hex digits format>.
   * Example: M_000002074e812088 */
  char str[32];
  snprintf(str, 32, "M_%016llx", (uint64_t)material);
  return scene_delegate->GetDelegateID().AppendElementString(str);
}

MaterialData::MaterialData(pxr::HdSceneDelegate *scene_delegate, Material *material)
  : IdData(scene_delegate, (ID *)material)
{
}

VtValue MaterialData::get_data(TfToken const &key)
{
  VtValue ret;
  if (key.GetString() == "MaterialXFilename") {
    if (!mtlx_path.GetResolvedPath().empty()) {
      ret = mtlx_path;
    }
  }
  return ret;
}

pxr::VtValue MaterialData::material_resource()
{
  /* TODO: Implement return of HdMaterialNetwork */
  return pxr::VtValue();
}

void MaterialData::export_mtlx()
{
  /* Call of python function hydra.export_mtlx() */

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject *module, *dict, *func, *result;
  module = PyImport_ImportModule("hydra");
  dict = PyModule_GetDict(module);
  func = PyDict_GetItemString(dict, "export_mtlx");
  result = PyObject_CallFunction(func, "s", name().c_str());

  std::string path = PyUnicode_AsUTF8(result);

  Py_DECREF(result);
  Py_DECREF(module);

  PyGILState_Release(gstate);

  mtlx_path = SdfAssetPath(path, path);
  LOG(INFO) << "Material export: " << name() << " mtlx=" << mtlx_path.GetResolvedPath();
}

void MaterialData::insert_prim()
{
  SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().InsertSprim(HdPrimTypeTokens->material, scene_delegate, p_id);
  LOG(INFO) << "Add material: " << name() << " id=" << p_id.GetAsString();
}

void MaterialData::remove_prim()
{
  SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().RemoveSprim(HdPrimTypeTokens->material, p_id);
  LOG(INFO) << "Remove material: " << name();
}

void MaterialData::mark_prim_dirty(DirtyBits dirty_bits)
{
  HdDirtyBits bits = HdMaterial::Clean;
  switch (dirty_bits) {
    case DirtyBits::AllDirty:
      bits = HdMaterial::AllDirty;
      break;
    default:
      break;
  }
  SdfPath p_id = prim_id(scene_delegate, (Material *)id);
  scene_delegate->GetRenderIndex().GetChangeTracker().MarkSprimDirty(p_id, bits);
  LOG(INFO) << "Update material: " << name() << ", mtlx=" << mtlx_path.GetResolvedPath();
}

} // namespace blender::render::hydra
