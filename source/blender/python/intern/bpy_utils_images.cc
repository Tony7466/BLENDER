/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pythonintern
 *
 * This file defines a singleton py object accessed via 'bpy.utils.images',
 */

#include <Python.h>
#include <structmember.h>

#include <string.h>

#include "BLI_utildefines.h"
#include "BLI_listbase.h"
#include "BLI_string.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"
#include "RNA_types.hh"

#include "BPY_extern.h"
#include "bpy_rna.h"
#include "bpy_utils_images.h"

#include "../generic/py_capi_utils.h"

#include "MEM_guardedalloc.h"

#include "IMB_imbuf.h"

#include "../generic/python_utildefines.h"


struct bpy_image_data{
  bpy_image_data *prev;
  bpy_image_data *next;

  int id;
  ImBuf *ibuf;
  std::string name;
  std::string path;
};

int bpy_images_last_id = 0;
ListBase bpy_images_list;


bpy_image_data* BPY_utils_images_get_data(int image_id) {
  return (bpy_image_data *)BLI_listbase_bytes_find(
      &bpy_images_list, &image_id, sizeof(int), sizeof(bpy_image_data *) * 2);
}

void* BPY_utils_images_get(int image_id)
{
  bpy_image_data *el = BPY_utils_images_get_data(image_id);
  return (el != nullptr) ? el->ibuf : nullptr;
}

PyDoc_STRVAR(bpy_utils_images_load_doc,
             ".. method:: load(name, filepath)\n"
             "\n"
             "   Generate a new preview from given file path.\n"
             "\n"
             "   :arg name: The name identifying the image.\n"
             "   :type name: string\n"
             "   :arg filepath: The file path to the image.\n"
             "   :type filepath: string or bytes\n"
             "   :return: image id.\n"
             "   :rtype: long`\n");
static PyObject *bpy_utils_images_load(PyObject * /*self*/, PyObject *args)
{
  char *name = NULL;
  PyC_UnicodeAsBytesAndSize_Data filepath_data = {nullptr};

  if (!PyArg_ParseTuple(args,
                        "s"  /* `name` */
                        "O&" /* `filepath` */
                        ":load",
                        &name,
                        PyC_ParseUnicodeAsBytesAndSize,
                        &filepath_data,
                        0))
  {
    return nullptr;
  }

  if (!filepath_data.value || !name) {
    Py_RETURN_NONE;
  }

  ImBuf *ibuf = IMB_loadiffname(filepath_data.value, 0, nullptr);

  if (ibuf) {
    bpy_image_data *data = MEM_new<bpy_image_data>(__func__);
    data->id = ++bpy_images_last_id;
    data->ibuf = ibuf;
    data->name = name;
    data->path = filepath_data.value;

    BLI_addtail(&bpy_images_list, data);

    return PyLong_FromLong(data->id);
  }
  else {
    Py_RETURN_NONE;
  }
}

PyDoc_STRVAR(bpy_utils_images_release_doc,
             ".. method:: release(image_id)\n"
             "\n"
             "   Release (free) a previously added image.\n"
             "\n"
             "\n"
             "   :arg image_id: The id identifying the image.\n"
             "   :type name: long\n"
             "   :return: true if release.\n"
             "   :rtype: bool`\n");
static PyObject *bpy_utils_images_release(PyObject * /*self*/, PyObject *args)
{
  int image_id = -1;

  if (!PyArg_ParseTuple(args, "i:release", &image_id)) {
    return nullptr;
  }

    bpy_image_data *el = BPY_utils_images_get_data(image_id);

    if (el != nullptr) {
    BLI_remlink(&bpy_images_list, el);
    IMB_freeImBuf(el->ibuf);
    MEM_freeN(el);
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

static PyMethodDef bpy_utils_images_methods[] = {
    /* Can't use METH_KEYWORDS alone, see http://bugs.python.org/issue11587 */
    {"load", (PyCFunction)bpy_utils_images_load, METH_VARARGS, bpy_utils_images_load_doc},
    {"release",(PyCFunction)bpy_utils_images_release,METH_VARARGS, bpy_utils_images_release_doc},
    {nullptr, nullptr, 0, nullptr},
};

PyDoc_STRVAR(
    bpy_utils_images_doc,
    "This object contains basic static methods to handle cached (non-ID) previews in Blender\n"
    "(low-level API, not exposed to final users).");
static PyModuleDef bpy_utils_images_module = {
    /*m_base*/ PyModuleDef_HEAD_INIT,
    /*m_name*/ "bpy._utils_images",
    /*m_doc*/ bpy_utils_images_doc,
    /*m_size*/ 0,
    /*m_methods*/ bpy_utils_images_methods,
    /*m_slots*/ nullptr,
    /*m_traverse*/ nullptr,
    /*m_clear*/ nullptr,
    /*m_free*/ nullptr,
};

PyObject *BPY_utils_images_module()
{
  PyObject *submodule;

  submodule = PyModule_Create(&bpy_utils_images_module);

  return submodule;
}
