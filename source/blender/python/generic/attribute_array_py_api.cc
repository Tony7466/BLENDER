/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pygen
 */

#include "attribute_array_py_types.hh"

#include "RNA_access.hh"

namespace blender::python::attributearray {

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Mapping
 * \{ */

static int BPy_AttributeArray_assign_subscript_int(BPy_AttributeArray *self,
                                                   Py_ssize_t index,
                                                   PyObject *value_to_assign)
{
  Py_ssize_t index_abs = index >= 0 ? index : index + self->data_layer->length;
  if (index_abs < 0 || index_abs >= self->data_layer->length) {
    PyErr_Format(PyExc_IndexError,
                 "AttributeArray[index] = value: index %d out of range, size %d",
                 index,
                 self->data_layer->length);
    return -1;
  }

  if (!check_attribute_type(self)) {
    return -1;
  }

  if (!set_attribute_by_type(self, int(index), value_to_assign)) {
    StringRef type_description = get_attribute_type_description(self);
    PyErr_Format(PyExc_ValueError,
                 "AttributeArray[index] = value: invalid value type, expected a %.200s",
                 type_description.data());
    return -1;
  }
  return 0;
}

static int BPy_AttributeArray_assign_subscript_slice(BPy_AttributeArray *self,
                                                     Py_ssize_t start,
                                                     Py_ssize_t stop,
                                                     PyObject *values_to_assign)
{
  if (!check_attribute_type(self)) {
    return -1;
  }

  if (!PySequence_Check(values_to_assign)) {
    StringRef type_description = get_attribute_type_description(self);
    PyErr_Format(PyExc_TypeError,
                 "AttributeArray[:] = value: invalid value type, expected a sequence of %.200s",
                 type_description.data());
    return -1;
  }

  const Py_ssize_t count = PySequence_Fast_GET_SIZE(values_to_assign);
  if (count != stop - start) {
    PyErr_Format(PyExc_ValueError,
                 "AttributeArray[:] = value: size mismatch in assignment (expected %d, got %d)",
                 stop - start,
                 count);
    return -1;
  }

  /* Assign sequence to attribute array. */
  PyObject **values_fast = PySequence_Fast_ITEMS(values_to_assign);
  for (int index = start; index < stop; index++) {
    PyObject *item = values_fast[index - start];
    if (item) {
      if (!set_attribute_by_type(self, index, item)) {
        StringRef type_description = get_attribute_type_description(self);
        PyErr_Format(PyExc_ValueError,
                     "AttributeArray[index] = value: invalid value type, expected a %.200s",
                     type_description.data());
        return -1;
      }
    }
    else {
      return -1;
    }
  }
  return 0;
}

static int BPy_AttributeArray_assign_subscript(BPy_AttributeArray *self,
                                               PyObject *key,
                                               PyObject *value)
{
  if (value == nullptr) {
    PyErr_SetString(PyExc_ValueError, "del AttributeArray[key]: not supported");
    return -1;
  }

  if (PyIndex_Check(key)) {
    const Py_ssize_t index = PyNumber_AsSsize_t(key, PyExc_IndexError);
    if (index == -1 && PyErr_Occurred()) {
      return -1;
    }

    return BPy_AttributeArray_assign_subscript_int(self, index, value);
  }
  else if (PySlice_Check(key)) {
    Py_ssize_t step = 1;
    PySliceObject *key_slice = (PySliceObject *)key;

    if (key_slice->step != Py_None && !_PyEval_SliceIndex(key, &step)) {
      return -1;
    }
    if (step != 1) {
      PyErr_SetString(PyExc_IndexError, "AttributeArray[slice]: slice steps not supported");
      return -1;
    }

    Py_ssize_t start, stop, slicelength;
    if (PySlice_GetIndicesEx(key, self->data_layer->length, &start, &stop, &step, &slicelength) <
        0)
    {
      return -1;
    }
    if (slicelength <= 0) {
      return 0;
    }

    return BPy_AttributeArray_assign_subscript_slice(self, start, stop, value);
  }

  PyErr_Format(PyExc_IndexError,
               "AttributeArray[key] = value: invalid key, must be an int or slice, not %.200s",
               Py_TYPE(key)->tp_name);
  return -1;
}

static PyObject *BPy_AttributeArray_subscript_int(BPy_AttributeArray *self, Py_ssize_t index)
{
  Py_ssize_t index_abs = index >= 0 ? index : index + self->data_layer->length;
  if (index_abs < 0 || index_abs >= self->data_layer->length) {
    PyErr_Format(PyExc_IndexError,
                 "AttributeArray[index]: index %d out of range, size %d",
                 index,
                 self->data_layer->length);
    return nullptr;
  }

  if (!check_attribute_type(self)) {
    return nullptr;
  }

  return get_attribute_by_type(self, int(index));
}

static PyObject *BPy_AttributeArray_subscript_slice(BPy_AttributeArray *self,
                                                    Py_ssize_t start,
                                                    Py_ssize_t stop)
{
  PyObject *list = PyList_New(stop - start);
  for (int i = 0; i < stop - start; i++) {
    PyObject *item = BPy_AttributeArray_subscript_int(self, start + i);
    PyList_SET_ITEM(list, i, item);
  }
  return list;
}

static PyObject *BPy_AttributeArray_subscript(BPy_AttributeArray *self, PyObject *key)
{
  if (PyIndex_Check(key)) {
    const Py_ssize_t keynum = PyNumber_AsSsize_t(key, PyExc_IndexError);
    if (keynum == -1 && PyErr_Occurred()) {
      return nullptr;
    }

    return BPy_AttributeArray_subscript_int(self, keynum);
  }
  else if (PySlice_Check(key)) {
    Py_ssize_t step = 1;
    PySliceObject *key_slice = (PySliceObject *)key;

    if (key_slice->step != Py_None && !_PyEval_SliceIndex(key, &step)) {
      return nullptr;
    }
    if (step != 1) {
      PyErr_SetString(PyExc_IndexError, "AttributeArray[slice]: slice steps not supported");
      return nullptr;
    }

    Py_ssize_t start, stop, slicelength;
    if (PySlice_GetIndicesEx(key, self->data_layer->length, &start, &stop, &step, &slicelength) <
        0)
    {
      return nullptr;
    }
    if (slicelength <= 0) {
      return PyList_New(0);
    }

    return BPy_AttributeArray_subscript_slice(self, start, stop);
  }

  PyErr_Format(PyExc_IndexError,
               "AttributeArray[key]: invalid key, must be an int or slice, not %.200s",
               Py_TYPE(key)->tp_name);
  return nullptr;
}

static Py_ssize_t BPy_AttributeArray_length(BPy_AttributeArray *self)
{
  return self->data_layer->length;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Foreach get/set
 * \{ */

static PyObject *foreach_getset(BPy_AttributeArray *self,
                                PyObject *args,
                                const char *function_name,
                                const bool use_get)
{
  PyObject *buffer_obj;
  if (!PyArg_ParseTuple(args, "O:foreach_get/set", &buffer_obj)) {
    return nullptr;
  }

  if (!PyObject_CheckBuffer(buffer_obj)) {
    PyErr_Format(
        PyExc_TypeError,
        "%s(array) expected the argument to be a contiguous array (like a Python or numpy array), "
        "not a %.200s",
        function_name,
        Py_TYPE(buffer_obj)->tp_name);
    return nullptr;
  }

  /* Compare buffer with attribute array size and type. */
  Py_buffer buffer;
  if (PyObject_GetBuffer(buffer_obj, &buffer, PyBUF_ND | PyBUF_FORMAT) == -1) {
    PyErr_SetString(PyExc_BufferError, "%s(array): couldn't access the buffer");
    return nullptr;
  }

  if (!check_attribute_type(self)) {
    return nullptr;
  }

  const bool type_is_compatible = check_buffer_format(self, buffer.format);
  const int item_size = get_attribute_item_size(self);
  const bool size_matches = (buffer.len == int64_t(self->data_layer->length) * item_size);

  if (!type_is_compatible || !size_matches) {
    PyBuffer_Release(&buffer);
  }

  if (!type_is_compatible) {
    const char expected_buffer_format = get_buffer_format_by_type(self);
    PyErr_Format(PyExc_TypeError,
                 "%s(array): array type mismatch (expected type '%.1s', got '%.1s')",
                 function_name,
                 &expected_buffer_format,
                 buffer.format);
    return nullptr;
  }

  if (!size_matches) {
    PyErr_Format(PyExc_ValueError,
                 "%s(array): array length mismatch (expected %lld, got %lld)",
                 function_name,
                 int64_t(self->data_layer->length) * item_size,
                 buffer.len);
    return nullptr;
  }

  /* Source and destination arrays are contiguous, copy the data at once. */
  if (use_get) {
    memcpy(buffer.buf, self->data_layer->data, buffer.len);
  }
  else {
    memcpy(self->data_layer->data, buffer.buf, buffer.len);
  }

  PyBuffer_Release(&buffer);

  Py_RETURN_NONE;
}

PyDoc_STRVAR(
    /* Wrap. */
    BPy_AttributeArray_foreach_get_doc,
    ".. method:: foreach_get(array)\n"
    "\n"
    "   This is a function to give fast access to the values in an geometry attribute array.\n"
    "\n"
    "   :arg array: An contiguous array of the same size as the attribute array,\n"
    "               e.g. a Python array.array() or a numpy array.\n"
    "   :type array: The same type as the attribute array.\n");
static PyObject *BPy_AttributeArray_foreach_get(BPy_AttributeArray *self, PyObject *args)
{
  return foreach_getset(self, args, "foreach_get", true);
}

PyDoc_STRVAR(
    /* Wrap. */
    BPy_AttributeArray_foreach_set_doc,
    ".. method:: foreach_set(array)\n"
    "\n"
    "   This is a function to give fast access to the values in an geometry attribute array.\n"
    "\n"
    "   :arg array: An contiguous array of the same size as the attribute array,\n"
    "               e.g. a Python array.array() or a numpy array.\n"
    "   :type array: The same type as the attribute array.\n");
static PyObject *BPy_AttributeArray_foreach_set(BPy_AttributeArray *self, PyObject *args)
{
  return foreach_getset(self, args, "foreach_set", false);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Fill
 * \{ */

static std::optional<int> get_index_int(PyObject *py_index)
{
  const int index = PyLong_AsLong(py_index);
  if (index == -1 && PyErr_Occurred()) {
    return std::nullopt;
  }
  return index;
}

PyDoc_STRVAR(
    /* Wrap. */
    BPy_AttributeArray_fill_doc,
    ".. method:: fill(value, indices=None)\n"
    "\n"
    "   Fill the geometry attribute array with a value.\n"
    "\n"
    "   :arg value: Value to fill the array with.\n"
    "   :arg indices: Only fill the given indices (optional argument).\n"
    "   :type indices: List, array or numpy array of ints.\n");
static PyObject *BPy_AttributeArray_fill(BPy_AttributeArray *self, PyObject *args)
{
  /* Get fill value and (optionally) a list of indices. */
  PyObject *py_value, *py_indices = nullptr;
  if (!PyArg_ParseTuple(args, "O|O:AttributeArray.fill(value, indices)", &py_value, &py_indices)) {
    return nullptr;
  }
  if (!check_attribute_type(self)) {
    return nullptr;
  }

  /* Verify the indices object. */
  if (py_indices == Py_None) {
    py_indices = nullptr;
  }
  if (py_indices && !PySequence_Check(py_indices)) {
    PyErr_SetString(
        PyExc_TypeError,
        "AttributeArray.fill[value, indices]: invalid indices type, expected a list of ints");
    return nullptr;
  }

  Py_ssize_t indices_length;
  if (py_indices) {
    indices_length = PySequence_Size(py_indices);
    if (indices_length == 0) {
      Py_RETURN_NONE;
    }
  }

  /* Get the index of first value in the array. */
  int first_index = 0;
  if (py_indices) {
    PyObject *py_index = PySequence_GetItem(py_indices, 0);
    std::optional<int> maybe_first_index = get_index_int(py_index);
    if (!maybe_first_index.has_value()) {
      PyErr_Format(
          PyExc_IndexError,
          "AttributeArray.fill(value, indices): invalid index type (expected an int, got %.200s)",
          Py_TYPE(py_index)->tp_name);
      Py_DECREF(py_index);
      return nullptr;
    }

    Py_DECREF(py_index);
    first_index = maybe_first_index.value();
    if (first_index < 0 || first_index >= self->data_layer->length) {
      PyErr_Format(PyExc_IndexError,
                   "AttributeArray.fill(value, indices): index %d out of range, size %d",
                   first_index,
                   self->data_layer->length);
      return nullptr;
    }
  }

  /* Set the first value. */
  if (!set_attribute_by_type(self, first_index, py_value)) {
    StringRef type_description = get_attribute_type_description(self);
    PyErr_Format(PyExc_ValueError,
                 "AttributeArray.fill(value, ...): invalid value type, expected a %.200s",
                 type_description.data());
    return nullptr;
  }

  /* Fill the entire array. */
  void *data = self->data_layer->data;
  const int item_size = get_attribute_item_size(self);
  if (py_indices == nullptr) {
    for (int index = 1; index < self->data_layer->length; index++) {
      memcpy(static_cast<int8_t *>(data) + index * item_size, data, item_size);
    }
  }
  /* Fill the array using indices. */
  else {
    for (int i = 1; i < indices_length; i++) {
      PyObject *py_index = PySequence_GetItem(py_indices, i);
      std::optional<int> maybe_index = get_index_int(py_index);
      if (!maybe_index.has_value()) {
        PyErr_Format(PyExc_IndexError,
                     "AttributeArray.fill(value, indices): invalid index type (expected an int, "
                     "got %.200s)",
                     Py_TYPE(py_index)->tp_name);
        Py_DECREF(py_index);
        return nullptr;
      }

      Py_DECREF(py_index);
      const int index = maybe_index.value();
      if (index < 0 || index >= self->data_layer->length) {
        PyErr_Format(PyExc_IndexError,
                     "AttributeArray.fill(value, indices): index %d out of range, size %d",
                     index,
                     self->data_layer->length);
        return nullptr;
      }

      memcpy(static_cast<int8_t *>(data) + index * item_size,
             static_cast<int8_t *>(data) + first_index * item_size,
             item_size);
    }
  }

  Py_RETURN_NONE;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Miscellaneous
 * \{ */

static PyObject *BPy_AttributeArray_repr(BPy_AttributeArray *self)
{
  const CustomDataLayer &layer = *self->data_layer;

  return PyUnicode_FromFormat("<AttributeArray, '%.200s' at %p>", layer.name, layer.data);
}

static Py_hash_t BPy_AttributeArray_hash(BPy_AttributeArray *self)
{
  return _Py_HashPointer(self->data_layer);
}

/** \} */

}  // namespace blender::python::attributearray

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Create object
 * \{ */

PyObject *BPy_AttributeArray_CreatePyObject(PointerRNA *ptr)
{
  BPy_AttributeArray *self = PyObject_New(BPy_AttributeArray, &BPy_AttributeArray_Type);
  self->data_layer = static_cast<CustomDataLayer *>(ptr->data);
  return (PyObject *)self;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Type
 * \{ */

using namespace blender::python::attributearray;

static PyMethodDef BPy_AttributeArray_methods[] = {
    {"foreach_get",
     (PyCFunction)BPy_AttributeArray_foreach_get,
     METH_VARARGS,
     BPy_AttributeArray_foreach_get_doc},
    {"foreach_set",
     (PyCFunction)BPy_AttributeArray_foreach_set,
     METH_VARARGS,
     BPy_AttributeArray_foreach_set_doc},
    {"fill", (PyCFunction)BPy_AttributeArray_fill, METH_VARARGS, BPy_AttributeArray_fill_doc},
    {nullptr, nullptr, 0, nullptr},
};

static PySequenceMethods BPy_AttributeArray_as_sequence = {
    /*sq_length*/ (lenfunc)BPy_AttributeArray_length,
    /*sq_concat*/ nullptr,
    /*sq_repeat*/ nullptr,
    /*sq_item*/ (ssizeargfunc)BPy_AttributeArray_subscript_int,
    /*was_sq_slice*/ nullptr,
    /*sq_ass_item*/
    (ssizeobjargproc)BPy_AttributeArray_assign_subscript_int,
    /*was_sq_ass_slice*/ nullptr,
    /*sq_contains*/ nullptr,
    /*sq_inplace_concat*/ nullptr,
    /*sq_inplace_repeat*/ nullptr,
};

static PyMappingMethods BPy_AttributeArray_as_mapping = {
    /*mp_length*/ (lenfunc)BPy_AttributeArray_length,
    /*mp_subscript*/ (binaryfunc)BPy_AttributeArray_subscript,
    /*mp_ass_subscript*/
    (objobjargproc)BPy_AttributeArray_assign_subscript,
};

PyTypeObject BPy_AttributeArray_Type = {
    /*ob_base*/ PyVarObject_HEAD_INIT(nullptr, 0)
    /*tp_name*/ "AttributeArray",
    /*tp_basicsize*/ sizeof(BPy_AttributeArray),
    /*tp_itemsize*/ 0,
    /*tp_dealloc*/ nullptr,
    /*tp_vectorcall_offset */ 0,
    /*tp_getattr*/ nullptr,
    /*tp_setattr*/ nullptr,
    /*tp_as_async*/ nullptr,
    /*tp_repr*/ (reprfunc)BPy_AttributeArray_repr,
    /*tp_as_number*/ nullptr,
    /*tp_as_sequence*/ &BPy_AttributeArray_as_sequence,
    /*tp_as_mapping*/ &BPy_AttributeArray_as_mapping,
    /*tp_hash*/ (hashfunc)BPy_AttributeArray_hash,
    /*tp_call*/ nullptr,
    /*tp_str*/ nullptr,
    /*tp_getattro*/ nullptr,
    /*tp_setattro*/ nullptr,
    /*tp_as_buffer*/ nullptr,
    /*tp_flags*/ Py_TPFLAGS_DEFAULT,
    /*tp_doc*/ nullptr,
    /*tp_traverse*/ nullptr,
    /*tp_clear*/ nullptr,
    /*tp_richcompare*/ nullptr,
    /*tp_weaklistoffset*/ 0,
    /*tp_iter*/ nullptr,
    /*tp_iternext*/ nullptr,
    /*tp_methods*/ BPy_AttributeArray_methods,
    /*tp_members*/ nullptr,
    /*tp_getset*/ nullptr,
    /*tp_base*/ nullptr,
    /*tp_dict*/ nullptr,
    /*tp_descr_get*/ nullptr,
    /*tp_descr_set*/ nullptr,
    /*tp_dictoffset*/ 0,
    /*tp_init*/ nullptr,
    /*tp_alloc*/ nullptr,
    /*tp_new*/ nullptr,
    /*tp_free*/ nullptr,
    /*tp_is_gc*/ nullptr,
    /*tp_bases*/ nullptr,
    /*tp_mro*/ nullptr,
    /*tp_cache*/ nullptr,
    /*tp_subclasses*/ nullptr,
    /*tp_weaklist*/ nullptr,
    /*tp_del*/ nullptr,
    /*tp_version_tag*/ 0,
    /*tp_finalize*/ nullptr,
    /*tp_vectorcall*/ nullptr,
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Initialize Types
 * \{ */

void AttributeArray_Init_Types()
{
  PyType_Ready(&BPy_AttributeArray_Type);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public Module 'attribute_array.types'
 * \{ */

static PyModuleDef AttributeArray_types_module_def = {
    /*m_base*/ PyModuleDef_HEAD_INIT,
    /*m_name*/ "attribute_array.types",
    /*m_doc*/ nullptr,
    /*m_size*/ 0,
    /*m_methods*/ nullptr,
    /*m_slots*/ nullptr,
    /*m_traverse*/ nullptr,
    /*m_clear*/ nullptr,
    /*m_free*/ nullptr,
};

static PyObject *BPyInit_attribute_array_types()
{
  PyObject *submodule;

  submodule = PyModule_Create(&AttributeArray_types_module_def);

  AttributeArray_Init_Types();

  PyModule_AddType(submodule, &BPy_AttributeArray_Type);

  return submodule;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public Module 'attribute_array'
 * \{ */

PyDoc_STRVAR(
    /* Wrap. */
    AttributeArray_module_doc,
    "This module provides access to geometry attribute arrays.");
static PyModuleDef AttributeArray_module_def = {
    /*m_base*/ PyModuleDef_HEAD_INIT,
    /*m_name*/ "attribute_array",
    /*m_doc*/ AttributeArray_module_doc,
    /*m_size*/ 0,
    /*m_methods*/ nullptr,
    /*m_slots*/ nullptr,
    /*m_traverse*/ nullptr,
    /*m_clear*/ nullptr,
    /*m_free*/ nullptr,
};

PyObject *BPyInit_attribute_array()
{
  PyObject *mod;
  PyObject *submodule;
  PyObject *sys_modules = PyImport_GetModuleDict();

  mod = PyModule_Create(&AttributeArray_module_def);

  /* attribute_array.types */
  PyModule_AddObject(mod, "types", (submodule = BPyInit_attribute_array_types()));
  PyDict_SetItem(sys_modules, PyModule_GetNameObject(submodule), submodule);

  return mod;
}

/** \} */
