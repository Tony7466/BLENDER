/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include <iostream>
#include <cstdlib>

#include <Python.h>

#include <pxr/pxr.h>
#include <pxr/base/plug/plugin.h>
#include <pxr/base/plug/registry.h>
#include <pxr/usdImaging/usdImagingGL/engine.h>

#include "glog/logging.h"
#include "BKE_appdir.h"

#include "finalEngine.h"
#include "viewportEngine.h"
#include "previewEngine.h"

using namespace std;

namespace blender::render::hydra {

static PyObject *init_func(PyObject * /*self*/, PyObject *args)
{
  LOG(INFO) << "init_func";

  pxr::PlugRegistry::GetInstance().RegisterPlugins(string(BKE_appdir_program_dir()) + "/blender.shared/usd");

  Py_RETURN_NONE;
}

static PyObject *register_plugins_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyplugin_dirs, *pypath_dirs;
  if (!PyArg_ParseTuple(args, "OO", &pyplugin_dirs, &pypath_dirs)) {
    Py_RETURN_NONE;
  }

  LOG(INFO) << "register_plugins_func";

  vector<string> plugin_dirs, path_dirs;
  PyObject *pyiter, *pyitem;

  pyiter = PyObject_GetIter(pyplugin_dirs);
  if (pyiter) {
    while (pyitem = PyIter_Next(pyiter)) {
      plugin_dirs.push_back(PyUnicode_AsUTF8(pyitem));
      Py_DECREF(pyitem);
    }
    Py_DECREF(pyiter);
  }

  pyiter = PyObject_GetIter(pypath_dirs);
  if (pyiter) {
    while (pyitem = PyIter_Next(pyiter)) {
      path_dirs.push_back(PyUnicode_AsUTF8(pyitem));
      Py_DECREF(pyitem);
    }
    Py_DECREF(pyiter);
  }
  
  if (!path_dirs.empty()) {
    stringstream ss;
    ss << "PATH=";
    for (string &s : path_dirs) {
      ss << s;
#ifdef _WIN32
      ss << ";";
#else
      ss << ":";
#endif
    }
    ss << getenv("PATH");
    putenv(ss.str().c_str());
  }

  pxr::PlugRegistry &registry = pxr::PlugRegistry::GetInstance();
  registry.RegisterPlugins(plugin_dirs);

  Py_RETURN_NONE;
}

static PyObject *get_render_plugins_func(PyObject * /*self*/, PyObject *args)
{
  LOG(INFO) << "get_render_plugins_func";

  PlugRegistry &registry = PlugRegistry::GetInstance();
  TfTokenVector pluginsIds = UsdImagingGLEngine::GetRendererPlugins();
  PyObject *ret = PyTuple_New(pluginsIds.size());
  PyObject *val;
  for (int i = 0; i < pluginsIds.size(); ++i) {
    PyObject *descr = PyDict_New();

    PyDict_SetItemString(descr, "id", val = PyUnicode_FromString(pluginsIds[i].GetText()));
    Py_DECREF(val);

    PyDict_SetItemString(descr, "name", 
      val = PyUnicode_FromString(UsdImagingGLEngine::GetRendererDisplayName(pluginsIds[i]).c_str()));
    Py_DECREF(val);

    string plugin_name = pluginsIds[i];
    plugin_name = plugin_name.substr(0, plugin_name.size() - 6);
    plugin_name[0] = tolower(plugin_name[0]);
    string path = "";
    PlugPluginPtr plugin = registry.GetPluginWithName(plugin_name);
    if (plugin) {
      path = plugin->GetPath();
    }
    PyDict_SetItemString(descr, "path", val = PyUnicode_FromString(path.c_str()));
    Py_DECREF(val);

    PyTuple_SetItem(ret, i, descr);
  }
  return ret;
}

static PyObject *engine_create_func(PyObject * /*self*/, PyObject *args)
{
  LOG(INFO) << "create_func";

  PyObject *pyengine;
  char *engineType, *delegateId;
  if (!PyArg_ParseTuple(args, "Oss", &pyengine, &engineType, &delegateId)) {
    Py_RETURN_NONE;
  }

  PointerRNA engineptr;
  RNA_pointer_create(NULL, &RNA_RenderEngine, (void *)PyLong_AsVoidPtr(pyengine), &engineptr);
  BL::RenderEngine b_engine(engineptr);

  Engine *engine;

  if (string(engineType) == "VIEWPORT") {
    engine = new ViewportEngine(b_engine, delegateId);
  }
  else if (string(engineType) == "PREVIEW") {
    engine = new PreviewEngine(b_engine, delegateId);
  }
  else {
    if (b_engine.bl_use_gpu_context()) {
      engine = new FinalEngineGL(b_engine, delegateId);
    }
    else {
      engine = new FinalEngine(b_engine, delegateId);
    }
  }

  return PyLong_FromVoidPtr(engine);
}

static PyObject *engine_free_func(PyObject * /*self*/, PyObject *args)
{
  LOG(INFO) << "free_func";

  PyObject *pyengine;
  if (!PyArg_ParseTuple(args, "O", &pyengine)) {
    Py_RETURN_NONE;
  }

  delete (Engine *)PyLong_AsVoidPtr(pyengine);
  Py_RETURN_NONE;
}

static PyObject *engine_sync_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext, *pysettings;
  if (!PyArg_ParseTuple(args, "OOOO", &pyengine, &pydepsgraph, &pycontext, &pysettings)) {
    Py_RETURN_NONE;
  }

  Engine *engine = (Engine *)PyLong_AsVoidPtr(pyengine);

  PointerRNA depsgraphptr;
  RNA_pointer_create(NULL, &RNA_Depsgraph, (ID *)PyLong_AsVoidPtr(pydepsgraph), &depsgraphptr);
  BL::Depsgraph b_depsgraph(depsgraphptr);

  PointerRNA contextptr;
  RNA_pointer_create(NULL, &RNA_Context, (ID *)PyLong_AsVoidPtr(pycontext), &contextptr);
  BL::Context b_context(contextptr);

  HdRenderSettingsMap settings;
  PyObject *pyiter = PyObject_GetIter(pysettings);
  if (pyiter) {
    PyObject *pykey, *pyval;
    while (pykey = PyIter_Next(pyiter)) {
      TfToken key(PyUnicode_AsUTF8(pykey));
      pyval = PyDict_GetItem(pysettings, pykey);

      if (PyLong_Check(pyval)) {
        settings[key] = PyLong_AsLong(pyval);
      }
      else if (PyFloat_Check(pyval)) {
        settings[key] = PyFloat_AsDouble(pyval);
      }
      else if (PyUnicode_Check(pyval)) {
        settings[key] = PyUnicode_AsUTF8(pyval);
      }
      Py_DECREF(pykey);
    }
    Py_DECREF(pyiter);
  }

  engine->sync(b_depsgraph, b_context, settings);
  Py_RETURN_NONE;
}

static PyObject *engine_render_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph;

  if (!PyArg_ParseTuple(args, "OO", &pyengine, &pydepsgraph)) {
    Py_RETURN_NONE;
  }

  Engine *engine = (Engine *)PyLong_AsVoidPtr(pyengine);

  PointerRNA depsgraphptr;
  RNA_pointer_create(NULL, &RNA_Depsgraph, (ID *)PyLong_AsVoidPtr(pydepsgraph), &depsgraphptr);
  BL::Depsgraph depsgraph(depsgraphptr);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS
    engine->render(depsgraph);
  Py_END_ALLOW_THREADS

  Py_RETURN_NONE;
}

static PyObject *engine_view_draw_func(PyObject * /*self*/, PyObject *args)
{
  PyObject *pyengine, *pydepsgraph, *pycontext;
  if (!PyArg_ParseTuple(args, "OOO", &pyengine, &pydepsgraph, &pycontext)) {
    Py_RETURN_NONE;
  }

  ViewportEngine *engine = (ViewportEngine *)PyLong_AsVoidPtr(pyengine);

  PointerRNA depsgraphptr;
  RNA_pointer_create(NULL, &RNA_Depsgraph, (ID *)PyLong_AsVoidPtr(pydepsgraph), &depsgraphptr);
  BL::Depsgraph b_depsgraph(depsgraphptr);

  PointerRNA contextptr;
  RNA_pointer_create(NULL, &RNA_Context, (ID *)PyLong_AsVoidPtr(pycontext), &contextptr);
  BL::Context b_context(contextptr);

  /* Allow Blender to execute other Python scripts. */
  Py_BEGIN_ALLOW_THREADS
    engine->render(b_depsgraph, b_context);
  Py_END_ALLOW_THREADS

  Py_RETURN_NONE;
}

static PyMethodDef methods[] = {
  {"init", init_func, METH_VARARGS, ""},
  {"register_plugins", register_plugins_func, METH_VARARGS, ""},
  {"get_render_plugins", get_render_plugins_func, METH_VARARGS, ""},

  {"engine_create", engine_create_func, METH_VARARGS, ""},
  {"engine_free", engine_free_func, METH_VARARGS, ""},
  {"engine_render", engine_render_func, METH_VARARGS, ""},
  {"engine_sync", engine_sync_func, METH_VARARGS, ""},
  {"engine_view_draw", engine_view_draw_func, METH_VARARGS, ""},

  {NULL, NULL, 0, NULL},
};

static struct PyModuleDef module = {
  PyModuleDef_HEAD_INIT,
  "_hydra",
  "Hydra render API",
  -1,
  methods,
  NULL,
  NULL,
  NULL,
  NULL,
};

} // namespace blender::render::hydra

#ifdef __cplusplus
extern "C" {
#endif

PyObject *Hydra_initPython(void)
{
  PyObject *mod = PyModule_Create(&blender::render::hydra::module);
  return mod;
}

#ifdef __cplusplus
}
#endif
