/* SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2011-2022 Blender Foundation */

#include "final_engine.h"
#include "preview_engine.h"
#include "viewport_engine.h"

#include <Python.h>

#include <boost/python/extract.hpp>

#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usdImaging/usdImagingGL/engine.h>

#include "BLI_fileops.h"
#include "BLI_path_util.h"

#include "BKE_appdir.h"

#include "RE_engine.h"

#include "hydra/image.h"

namespace blender::render::hydra {

static PyObject *register_plugins_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyplugin_dirs;
  if (!PyArg_ParseTuple(args, "O", &pyplugin_dirs)) {
    Py_RETURN_NONE;
  }

  std::vector<std::string> plugin_dirs;
  PyObject *pyiter, *pyitem;

  pyiter = PyObject_GetIter(pyplugin_dirs);
  if (pyiter) {
    while ((pyitem = PyIter_Next(pyiter))) {
      plugin_dirs.push_back(PyUnicode_AsUTF8(pyitem));
      Py_DECREF(pyitem);
    }
    Py_DECREF(pyiter);
  }

  pxr::PlugRegistry &registry = pxr::PlugRegistry::GetInstance();
  registry.RegisterPlugins(plugin_dirs);

  /* logging */
  std::stringstream ss;
  ss << "plugins=[";
  for (auto &s : plugin_dirs) {
    ss << s << ", ";
  }
  ss << "]";
  CLOG_INFO(LOG_RENDER_HYDRA, 0, "Register %s", ss.str().c_str());

  Py_RETURN_NONE;
}

static PyObject *engine_create_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine;
  char *engine_type, *render_delegate_id;
  if (!PyArg_ParseTuple(args, "Oss", &pyengine, &engine_type, &render_delegate_id)) {
    Py_RETURN_NONE;
  }

  RenderEngine *bl_engine = static_cast<RenderEngine *>(PyLong_AsVoidPtr(pyengine));

  Engine *engine = nullptr;
  try {
    if (STREQ(engine_type, "VIEWPORT")) {
      engine = new ViewportEngine(bl_engine, render_delegate_id);
    }
    else if (STREQ(engine_type, "PREVIEW")) {
      engine = new PreviewEngine(bl_engine, render_delegate_id);
    }
    else {
      engine = new FinalEngine(bl_engine, render_delegate_id);
    }
  }
  catch (std::runtime_error &e) {
    CLOG_ERROR(LOG_RENDER_HYDRA, "%s", e.what());
  }

  if (engine) {
    CLOG_INFO(LOG_RENDER_HYDRA, 1, "Engine %p %s", engine, engine_type);
  }
  return PyLong_FromVoidPtr(engine);
}

static PyObject *engine_free_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine;
  if (!PyArg_ParseTuple(args, "O", &pyengine)) {
    Py_RETURN_NONE;
  }

  Engine *engine = static_cast<Engine *>(PyLong_AsVoidPtr(pyengine));
  delete engine;

  CLOG_INFO(LOG_RENDER_HYDRA, 1, "Engine %p", engine);
  Py_RETURN_NONE;
}

static PyObject *engine_sync_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext;
  if (!PyArg_ParseTuple(args, "OOO", &pyengine, &pydepsgraph, &pycontext)) {
    Py_RETURN_NONE;
  }

  Engine *engine = static_cast<Engine *>(PyLong_AsVoidPtr(pyengine));
  Depsgraph *depsgraph = static_cast<Depsgraph *>(PyLong_AsVoidPtr(pydepsgraph));
  bContext *context = static_cast<bContext *>(PyLong_AsVoidPtr(pycontext));

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %p", engine);
  engine->sync(depsgraph, context);

  Py_RETURN_NONE;
}

static PyObject *engine_render_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph;

  if (!PyArg_ParseTuple(args, "OO", &pyengine, &pydepsgraph)) {
    Py_RETURN_NONE;
  }

  Engine *engine = static_cast<Engine *>(PyLong_AsVoidPtr(pyengine));
  Depsgraph *depsgraph = static_cast<Depsgraph *>(PyLong_AsVoidPtr(pydepsgraph));

  CLOG_INFO(LOG_RENDER_HYDRA, 2, "Engine %p", engine);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS;
  engine->render(depsgraph);
  Py_END_ALLOW_THREADS;

  Py_RETURN_NONE;
}

static PyObject *engine_view_draw_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext;
  if (!PyArg_ParseTuple(args, "OOO", &pyengine, &pydepsgraph, &pycontext)) {
    Py_RETURN_NONE;
  }

  ViewportEngine *engine = static_cast<ViewportEngine *>(PyLong_AsVoidPtr(pyengine));
  Depsgraph *depsgraph = static_cast<Depsgraph *>(PyLong_AsVoidPtr(pydepsgraph));
  bContext *context = static_cast<bContext *>(PyLong_AsVoidPtr(pycontext));

  CLOG_INFO(LOG_RENDER_HYDRA, 3, "Engine %p", engine);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS;
  engine->render(depsgraph, context);
  Py_END_ALLOW_THREADS;

  Py_RETURN_NONE;
}

static pxr::VtValue get_setting_val(PyObject *pyval)
{
  pxr::VtValue val;
  if (PyBool_Check(pyval)) {
    val = Py_IsTrue(pyval);
  }
  else if (PyLong_Check(pyval)) {
    val = PyLong_AsLong(pyval);
  }
  else if (PyFloat_Check(pyval)) {
    val = PyFloat_AsDouble(pyval);
  }
  else if (PyUnicode_Check(pyval)) {
    val = std::string(PyUnicode_AsUTF8(pyval));
  }
  return val;
}

static PyObject *engine_set_sync_setting_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pyval;
  char *key;
  if (!PyArg_ParseTuple(args, "OsO", &pyengine, &key, &pyval)) {
    Py_RETURN_NONE;
  }

  Engine *engine = static_cast<Engine *>(PyLong_AsVoidPtr(pyengine));

  CLOG_INFO(LOG_RENDER_HYDRA, 3, "Engine %p: %s", engine, key);
  engine->set_sync_setting(key, get_setting_val(pyval));

  Py_RETURN_NONE;
}

static PyObject *engine_set_render_setting_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pyval;
  char *key;
  if (!PyArg_ParseTuple(args, "OsO", &pyengine, &key, &pyval)) {
    Py_RETURN_NONE;
  }

  Engine *engine = static_cast<Engine *>(PyLong_AsVoidPtr(pyengine));

  CLOG_INFO(LOG_RENDER_HYDRA, 3, "Engine %p: %s", engine, key);
  engine->set_render_setting(key, get_setting_val(pyval));

  Py_RETURN_NONE;
}

static PyObject *cache_or_get_image_file_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pycontext, *pyimage;
  if (!PyArg_ParseTuple(args, "OO", &pycontext, &pyimage)) {
    Py_RETURN_NONE;
  }

  bContext *context = static_cast<bContext *>(PyLong_AsVoidPtr(pycontext));
  Image *image = static_cast<Image *>(PyLong_AsVoidPtr(pyimage));

  std::string image_path = io::hydra::cache_or_get_image_file(
      CTX_data_main(context), CTX_data_scene(context), image, nullptr);
  return PyUnicode_FromString(image_path.c_str());
}

static PyMethodDef methods[] = {
    {"register_plugins", register_plugins_func, METH_VARARGS, ""},

    {"engine_create", engine_create_func, METH_VARARGS, ""},
    {"engine_free", engine_free_func, METH_VARARGS, ""},
    {"engine_sync", engine_sync_func, METH_VARARGS, ""},
    {"engine_render", engine_render_func, METH_VARARGS, ""},
    {"engine_view_draw", engine_view_draw_func, METH_VARARGS, ""},
    {"engine_set_sync_setting", engine_set_sync_setting_func, METH_VARARGS, ""},
    {"engine_set_render_setting", engine_set_render_setting_func, METH_VARARGS, ""},

    {"cache_or_get_image_file", cache_or_get_image_file_func, METH_VARARGS, ""},

    {NULL, NULL, 0, NULL},
};

static struct PyModuleDef module = {
    PyModuleDef_HEAD_INIT,
    "_bpy_hydra",
    "Hydra render API",
    -1,
    methods,
    NULL,
    NULL,
    NULL,
    NULL,
};

}  // namespace blender::render::hydra

#ifdef __cplusplus
extern "C" {
#endif

PyObject *BPyInit_hydra(void);

PyObject *BPyInit_hydra(void)
{
  PyObject *mod = PyModule_Create(&blender::render::hydra::module);
  return mod;
}

#ifdef __cplusplus
}
#endif
