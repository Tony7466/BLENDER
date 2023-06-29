/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd.h"

#include "usd_hook.h"

#include <boost/python/import.hpp>
#include <boost/python/object.hpp>
#include <boost/python/call_method.hpp>
#include <boost/python/class.hpp>
#include <boost/python/return_value_policy.hpp>
#include <boost/python/to_python_converter.hpp>

#include "BLI_listbase.h"

#include "bpy_rna.h"
#include "RNA_access.h"
#include "RNA_prototypes.h"
#include "RNA_types.h"

#include <list>

using namespace boost;

using USDHookList = std::list<USDHook*>;

/* USD hook type declarations */
static USDHookList g_usd_hooks;

void USD_register_hook(struct USDHook *hook)
{
  if (std::find(g_usd_hooks.begin(), g_usd_hooks.end(), hook) != g_usd_hooks.end()) {
    /* The hook is already in the list. */
    return;
  }

  /* Add hook type to the list. */
  g_usd_hooks.push_back(hook);
}


void USD_unregister_hook(struct USDHook *hook)
{
  g_usd_hooks.remove(hook);
}

USDHook *USD_find_hook_name(const char name[])
{
  /* sanity checks */
  if (g_usd_hooks.empty() || (name == NULL) || (name[0] == 0)) {
    return NULL;
  }

  USDHookList::iterator hook_iter = std::find_if(
      g_usd_hooks.begin(), g_usd_hooks.end(), [name](USDHook *hook) {
        return strcmp(hook->idname, name) == 0;
      });

  return (hook_iter == g_usd_hooks.end()) ? NULL : *hook_iter;
}

namespace blender::io::usd {

/* Convert PointerRNA to a PyObject*. */
struct PointerRNAToPython {

  /* We pass the argument by value because we need
   * to obtain a non-const pointer to it. */
  static PyObject *convert(PointerRNA ptr)
  {
    return pyrna_struct_CreatePyObject(&ptr);
  }

};

/* Encapsulate arguments for scene export. */
struct USDSceneExportContext {

  USDSceneExportContext() : depsgraph_ptr({})
  {
  }

  USDSceneExportContext(pxr::UsdStageRefPtr in_stage, Depsgraph *depsgraph) : stage(in_stage)
  {
    RNA_pointer_create(NULL, &RNA_Depsgraph, depsgraph, &depsgraph_ptr);
  }

  pxr::UsdStageRefPtr get_stage()
  {
    return stage;
  }

  const PointerRNA &get_depsgraph() const
  {
    return depsgraph_ptr;
  }

  pxr::UsdStageRefPtr stage;
  PointerRNA depsgraph_ptr;
};

/* Encapsulate arguments for material export. */
struct USDMaterialExportContext {

  USDMaterialExportContext() : material_ptr({}) {}

  USDMaterialExportContext(pxr::UsdStageRefPtr in_stage,
                           Material *material,
                           pxr::UsdShadeMaterial &in_usd_material)
      : stage(in_stage), usd_material(in_usd_material)
  {
    RNA_pointer_create(NULL, &RNA_Material, material, &material_ptr);
  }

  pxr::UsdStageRefPtr get_stage()
  {
    return stage;
  }

  const PointerRNA &get_blender_material() const
  {
    return material_ptr;
  }

  pxr::UsdShadeMaterial &get_usd_material()
  {
    return usd_material;
  }

  pxr::UsdStageRefPtr stage;
  pxr::UsdShadeMaterial usd_material;
  PointerRNA material_ptr;
};

void register_export_hook_converters()
{
  static bool registered = false;

  /* No need to register if there are no hooks. */
  if (g_usd_hooks.empty()) {
    return;
  }

  if (registered) {
    return;
  }

  registered = true;

  PyGILState_STATE gilstate = PyGILState_Ensure();

  /* We must import these modules for the USD type converters to work. */
  python::import("pxr.Usd");
  python::import("pxr.UsdShade");

  /* Register converter from PoinerRNA to a PyObject*. */
  python::to_python_converter<PointerRNA, PointerRNAToPython>();

  /* Register context class converters. */
  python::class_<USDSceneExportContext>("USDSceneExportContext")
      .def("get_stage", &USDSceneExportContext::get_stage)
      .def("get_depsgraph",
           &USDSceneExportContext::get_depsgraph,
           python::return_value_policy<python::return_by_value>())
    ;

  python::class_<USDMaterialExportContext>("USDMaterialExportContext")
      .def("get_stage", &USDMaterialExportContext::get_stage)
      .def("get_blender_material",
           &USDMaterialExportContext::get_blender_material,
           python::return_value_policy<python::return_by_value>())
      .def("get_usd_material",
           &USDMaterialExportContext::get_usd_material,
           python::return_value_policy<python::return_by_value>());
    ;

  PyGILState_Release(gilstate);
}

/* Invoke the member function with the given name of all registered hook instances.
 * The given context will be provided as the function argument. */
template<class T>
void call_hooks(const char *func_name, T &hook_context)
{
  if (g_usd_hooks.empty()) {
    return;
  }

  PyGILState_STATE gilstate = PyGILState_Ensure();

  /* Iterate over the hooks and invoke the hook function, if it's defined. */
  USDHookList::const_iterator hook_iter = g_usd_hooks.begin();
  while (hook_iter != g_usd_hooks.end()) {

    /* XXX: Not sure if this is necessary:
     * Advance the iterator before invoking the callback, to guard
     * against the unlikely error where the hook is deregistered in
     * the callback. This would prevent a crash due to the iterator
     * getting invalidated. */
    USDHook *hook = *hook_iter;
    ++hook_iter;

    if (!hook->rna_ext.data) {
      continue;
    }

    try {
      PyObject *hook_obj = static_cast<PyObject *>(hook->rna_ext.data);

      if (!PyObject_HasAttrString(hook_obj, func_name)) {
        continue;
      }

      python::call_method<void>(hook_obj, func_name, hook_context);
    }
    catch (...) {
      if (PyErr_Occurred()) {
        PyErr_Print();
      }
    }
  }

  PyGILState_Release(gilstate);
}


void call_export_hooks(pxr::UsdStageRefPtr stage, Depsgraph *depsgraph)
{
  if (g_usd_hooks.empty()) {
    return;
  }

  USDSceneExportContext export_context(stage, depsgraph);
  call_hooks("on_export", export_context);
}

void call_material_export_hooks(pxr::UsdStageRefPtr stage,
                                Material *material,
                                pxr::UsdShadeMaterial &usd_material)
{
  if (g_usd_hooks.empty()) {
    return;
  }

  USDMaterialExportContext export_context(stage, material, usd_material);
  call_hooks("on_material_export", export_context);
}

}  // namespace blender::io::usd
