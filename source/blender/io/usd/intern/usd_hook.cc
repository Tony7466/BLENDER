/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "usd.h"

#include "usd_hook.h"

#include <boost/python/object.hpp>
#include <boost/python/call_method.hpp>

#include "BLI_ListBase.h"

#include "bpy_rna.h"
#include "RNA_access.h"
#include "RNA_prototypes.h"
#include "RNA_types.h"

using namespace boost;

/* USD Hook Type declarations */
static ListBase usd_hook_types = {NULL, NULL};

void USD_register_hook(struct USDHook *hook)
{
  /* add type-info to the list */
  BLI_addtail(&usd_hook_types, hook);
}


void USD_unregister_hook(struct USDHook *hook)
{
  /* free the type info */
  BLI_freelinkN(&usd_hook_types, hook);
}

USDHook *USD_find_hook_name(const char name[])
{
  /* sanity checks */
  if ((name == NULL) || (name[0] == 0)) {
    return NULL;
  }

  /* search by comparing names */
  return static_cast<USDHook*>(BLI_findstring(&usd_hook_types, name, offsetof(USDHook, idname)));
}

namespace blender::io::usd {


void call_export_hooks(pxr::UsdStageRefPtr stage, Depsgraph *depsgraph)
{
  if (BLI_listbase_is_empty(&usd_hook_types)) {
    return;
  }

  PyGILState_STATE gilstate = PyGILState_Ensure();

  /* The chaser function name. */
  const char *func_name = "on_export";

  PointerRNA depsgraph_ptr;
  PyObject *depsgraph_obj = nullptr;

  /* Use mutable iteration in case hooks are unregistered. */
  LISTBASE_FOREACH_MUTABLE (USDHook *, hook, &usd_hook_types) {
    if (!hook->rna_ext.data) {
      continue;
    }

    try {
      PyObject *hook_obj = static_cast<PyObject *>(hook->rna_ext.data);

      if (!PyObject_HasAttrString(hook_obj, func_name)) {
        continue;
      }

      if (!depsgraph_obj) {
        RNA_pointer_create(NULL, &RNA_Depsgraph, depsgraph, &depsgraph_ptr);
        depsgraph_obj = pyrna_struct_CreatePyObject(&depsgraph_ptr);
      }

      if (!depsgraph_obj) {
        continue;
      }

      /* Invoke the chaser. Additional arguments could be
       * provided, e.g., a dictionary mapping Blender objects
       * to USD prims. */
      python::call_method<void>(hook_obj,
                                func_name,
                                python::object(python::handle<>(depsgraph_obj)),
                                stage);
    }
    catch (...) {
      if (PyErr_Occurred()) {
        PyErr_Print();
      }
    }
  }

  if (depsgraph_obj) {
    Py_DECREF(depsgraph_obj);
  }

  PyGILState_Release(gilstate);
}

}  // namespace blender::io::usd
