/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd.h"

#include "usd_hook.h"

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

  static PyObject *convert(const PointerRNA &pRNA)
  {
    return pyrna_struct_CreatePyObject(const_cast<PointerRNA*>(&pRNA));
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

  pxr::UsdStageRefPtr GetStage()
  {
    return stage;
  }

  const PointerRNA &GetDepsgraph()
  {
    return depsgraph_ptr;
  }

  pxr::UsdStageRefPtr stage;
  PointerRNA depsgraph_ptr;
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

  python::to_python_converter<PointerRNA, PointerRNAToPython>();

  python::class_<USDSceneExportContext>("USDSceneExportContext")
      .def("GetStage", &USDSceneExportContext::GetStage)
      .def("GetDepsgraph",
           &USDSceneExportContext::GetDepsgraph,
           python::return_value_policy<python::return_by_value>())
    ;

  PyGILState_Release(gilstate);
}


void call_export_hooks(pxr::UsdStageRefPtr stage, Depsgraph *depsgraph)
{
  if (g_usd_hooks.empty()) {
    return;
  }

  PyGILState_STATE gilstate = PyGILState_Ensure();

  /* The chaser function name. */
  const char *func_name = "on_export";

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

      USDSceneExportContext hook_context(stage, depsgraph);

      python::call_method<void>(hook_obj,
                                func_name,
                                hook_context);
    }
    catch (...) {
      if (PyErr_Occurred()) {
        PyErr_Print();
      }
    }
  }

  PyGILState_Release(gilstate);
}

}  // namespace blender::io::usd
