/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pythonintern
 * This file defines the '_bpy_asset_system' module, exposed as 'bpy.asset_system'.
 */

#include <Python.h>

#include "bpy_asset_system.h"

static struct PyMethodDef BPy_asset_system_methods[] = {
    {NULL, NULL, 0, NULL},
};

static struct PyModuleDef _bpy_asset_system_def = {
    PyModuleDef_HEAD_INIT,
    /*m_name*/ "asset_system",
    /*m_doc*/ NULL,
    /*m_size*/ 0,
    /*m_methods*/ BPy_asset_system_methods,
    /*m_slots*/ NULL,
    /*m_traverse*/ NULL,
    /*m_clear*/ NULL,
    /*m_free*/ NULL,
};

PyObject *BPY_asset_system_module(void)
{
  PyObject *submodule;

  submodule = PyModule_Create(&_bpy_asset_system_def);

  return submodule;
}
