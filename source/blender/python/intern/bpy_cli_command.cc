/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pythonintern
 *
 * Wrap `BKE_command_cli_*` to support custom CLI commands.
 */
#include <Python.h>

#include "BLI_utildefines.h"

#include "bpy_capi_utils.h"

#include "MEM_guardedalloc.h"

#include "BKE_blender_cli_command.hh"

#include "../generic/py_capi_utils.h"
#include "../generic/python_compat.h"
#include "../generic/python_utildefines.h"

static const char *bpy_cli_command_capsule_name = "bpy_cli_command";
static const char *bpy_cli_command_capsule_name_invalid = "bpy_cli_command<invalid>";

/* -------------------------------------------------------------------- */
/** \name Internal Implementation
 * \{ */

/**
 * Return a list of strings, compatible with the construction of Python's `sys.argv`.
 */
static PyObject *py_argv_from_bytes(const int argc, const char **argv)
{
  /* Copy functionality from Python's internal `sys.argv` initialization. */
  bool success = true;
  PyObject *args = PyList_New(argc);
  for (int i = 0; i < argc; i++) {
    size_t arg_len;
    wchar_t *arg = Py_DecodeLocale(argv[i], &arg_len);
    if (arg == nullptr) {
      success = false;
      break;
    }
    PyList_SET_ITEM(args, i, PyUnicode_FromWideChar(arg, arg_len));
    PyMem_RawFree(arg);
  }

  if (success == false) {
    /* It's OK if this is only partially initialized. */
    Py_DECREF(args);
    args = nullptr;
  }

  return args;
}

struct BPyCLI_CommandUserData {
  PyObject *py_execute_fn = nullptr;
};

static int bpy_cli_command_exec(struct bContext *C,
                                void *user_data,
                                const int argc,
                                const char **argv)
{
  int exit_code = EXIT_FAILURE;
  PyGILState_STATE gilstate;
  bpy_context_set(C, &gilstate);

  /* For the most part `sys.argv[-argc:]` is sufficient & less trouble than re-creating this list.
   * Don't do this because:
   * - Python scripts *could* have manipulated `sys.argv` (although it's bad practice).
   * - We may want to support invoking commands directly,
   *   where the arguments aren't necessarily from `sys.argv`.
   */
  PyObject *py_argv = py_argv_from_bytes(argc, argv);

  if (py_argv) {
    PyObject *exec_args = PyTuple_New(1);
    PyTuple_SET_ITEM(exec_args, 0, py_argv);

    BPyCLI_CommandUserData *data = static_cast<BPyCLI_CommandUserData *>(user_data);
    PyObject *result = PyObject_Call(data->py_execute_fn, exec_args, nullptr);

    Py_DECREF(exec_args); /* Frees `py_argv` too. */

    bool has_error = false;

    /* Convert `sys.exit` into a return-value.
     * NOTE: typically `sys.exit` *doesn't* need any special handling,
     * however it's neater if we use the same code paths for exiting either way. */
    if ((result == nullptr) && PyErr_ExceptionMatches(PyExc_SystemExit)) {
      PyObject *error_type, *error_value, *error_traceback;
      PyErr_Fetch(&error_type, &error_value, &error_traceback);
      if (PyObject_TypeCheck(error_value, (PyTypeObject *)PyExc_SystemExit) &&
          (((PySystemExitObject *)error_value)->code != nullptr))
      {
        /* When `SystemExit(..)` is raised. */
        result = ((PySystemExitObject *)error_value)->code;
      }
      else {
        /* When `sys.exit()` is called. */
        result = error_value;
      }
      Py_INCREF(result);
      PyErr_Restore(error_type, error_value, error_traceback);
      PyErr_Clear();
    }

    if (result == nullptr) {
      has_error = true;
    }
    else {
      if (!PyLong_Check(result)) {
        PyErr_Format(PyExc_TypeError,
                     "Expected an int return value, not a %.200s",
                     Py_TYPE(result)->tp_name);
        has_error = true;
      }
      else {
        const int exit_code_test = PyC_Long_AsI32(result);
        if ((exit_code_test == -1) && PyErr_Occurred()) {
          exit_code = EXIT_SUCCESS;
          has_error = true;
        }
        else {
          exit_code = exit_code_test;
        }
      }
      Py_DECREF(result);
    }

    if (has_error) {
      PyErr_Print();
      PyErr_Clear();
    }
  }

  bpy_context_clear(C, &gilstate);

  return exit_code;
}

static void bpy_cli_command_free(void *user_data)
{
  BPyCLI_CommandUserData *data = static_cast<BPyCLI_CommandUserData *>(user_data);
  Py_DECREF(data->py_execute_fn);
  MEM_delete(data);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public Methods
 * \{ */

PyDoc_STRVAR(
    /* Wrap. */
    bpy_cli_command_register_doc,
    ".. method:: register_cli_command(id, execute)\n"
    "\n"
    "   Register a command, accessible via the (``-c`` / ``--command``) command-line argument.\n"
    "\n"
    "   :arg id: The command identifier (must pass an ``str.isidentifier`` check).\n"
    "\n"
    "      If the ``id`` is already registered, a warning is printed and "
    "the command is inaccessible to prevent accidents invoking the wrong command."
    "   :type id: str\n"
    "   :arg execute: Callback, taking a single list of strings and returns an int.\n"
    "      The arguments are built from all command-line arguments following the command id.\n"
    "      The return value should be 0 for success, 1 on failure "
    "(specific error codes from the ``os`` module can also be used).\n"
    "   :type execute: callable\n"
    "   :return: The command handle which can be passed to :func:`unregister_cli_command`.\n"
    "   :rtype: capsule\n");
static PyObject *bpy_cli_command_register(PyObject * /*self*/, PyObject *args, PyObject *kw)
{
  PyObject *py_id;
  PyObject *py_execute_fn;

  static const char *_keywords[] = {
      "id",
      "execute",
      nullptr,
  };
  static _PyArg_Parser _parser = {
      PY_ARG_PARSER_HEAD_COMPAT()
      "O!"  /* `id` */
      "O"  /* `execute` */
      ":register_cli_command",
      _keywords,
      nullptr,
  };
  if (!_PyArg_ParseTupleAndKeywordsFast(
          args, kw, &_parser, &PyUnicode_Type, &py_id, &py_execute_fn))
  {
    return nullptr;
  }
  if (!PyUnicode_IsIdentifier(py_id)) {
    PyErr_SetString(PyExc_ValueError, "The command id is not a valid identifier");
    return nullptr;
  }
  if (!PyCallable_Check(py_execute_fn)) {
    PyErr_SetString(PyExc_ValueError, "The execute argument must be callable");
    return nullptr;
  }

  BPyCLI_CommandUserData *data = MEM_new<BPyCLI_CommandUserData>(__func__);
  data->py_execute_fn = Py_INCREF_RET(py_execute_fn);

  const char *id = PyUnicode_AsUTF8(py_id);
  CommandHandle *cmd_handle = BKE_blender_cli_command_register(
      id, bpy_cli_command_exec, bpy_cli_command_free, static_cast<void *>(data));

  return PyCapsule_New(cmd_handle, bpy_cli_command_capsule_name, nullptr);
}

PyDoc_STRVAR(
    /* Wrap. */
    bpy_cli_command_unregister_doc,
    ".. method:: unregister_cli_command(handle)\n"
    "\n"
    "   Unregister a CLI command.\n"
    "\n"
    "   :arg handle: The return value of :func:`register_cli_command`.\n"
    "   :type handle: capsule\n");
static PyObject *bpy_cli_command_unregister(PyObject * /*self*/, PyObject *value)
{
  if (!PyCapsule_CheckExact(value)) {
    PyErr_Format(PyExc_TypeError,
                 "Expected a capsule returned from register_cli_command(...), found a: %.200s",
                 Py_TYPE(value)->tp_name);
    return nullptr;
  }

  CommandHandle *cmd_handle = PyCapsule_GetPointer(value, bpy_cli_command_capsule_name);
  if (cmd_handle == nullptr) {
    const char *capsule_name = PyCapsule_GetName(value);
    if (capsule_name == bpy_cli_command_capsule_name_invalid) {
      PyErr_SetString(PyExc_ValueError, "The command has already been removed");
    }
    else {
      PyErr_Format(PyExc_ValueError,
                   "Unrecognized capsule ID \"%.200s\"",
                   capsule_name ? capsule_name : "<null>");
    }
    return nullptr;
  }
  /* Don't allow removing again. */
  PyCapsule_SetName(value, bpy_cli_command_capsule_name_invalid);

  BKE_blender_cli_command_unregister(cmd_handle);

  Py_RETURN_NONE;
}

#if (defined(__GNUC__) && !defined(__clang__))
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

PyMethodDef BPY_cli_command_register_def = {
    "register_cli_command",
    (PyCFunction)bpy_cli_command_register,
    METH_STATIC | METH_VARARGS | METH_KEYWORDS,
    bpy_cli_command_register_doc,
};
PyMethodDef BPY_cli_command_unregister_def = {
    "unregister_cli_command",
    (PyCFunction)bpy_cli_command_unregister,
    METH_STATIC | METH_O,
    bpy_cli_command_unregister_doc,
};

#if (defined(__GNUC__) && !defined(__clang__))
#  pragma GCC diagnostic pop
#endif

/** \} */
