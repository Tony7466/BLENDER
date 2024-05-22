/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pygen
 */

#pragma once

#include <functional>
#include <map>

#include <Python.h>

#include "attribute_array_py_api.h"
#include "py_capi_utils.h"

#include "../mathutils/mathutils.h"

#include "DNA_meshdata_types.h"

#include "BKE_customdata.hh"

#include "BLI_color.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_quaternion_types.hh"
#include "BLI_math_vector_types.hh"

namespace blender::python::attributearray {

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Set attribute
 * \{ */

static bool set_attribute_bool(void *data, int index, PyObject *py_value)
{
  const int value = PyC_Long_AsBool(py_value);
  if (value == -1) {
    return false;
  }
  static_cast<bool *>(data)[index] = value;
  return true;
}

static bool set_attribute_float(void *data, int index, PyObject *py_value)
{
  if (PyLong_Check(py_value)) {
    /* Allow int values (like 0), convert them to float. */
    const long value = PyLong_AsLong(py_value);
    static_cast<float *>(data)[index] = float(value);
    return true;
  }
  if (!PyFloat_Check(py_value)) {
    return false;
  }
  static_cast<float *>(data)[index] = PyFloat_AsDouble(py_value);
  return true;
}

static bool set_attribute_int8(void *data, int index, PyObject *py_value)
{
  const int value = _PyLong_AsInt(py_value);
  if (UNLIKELY(value == -1 && PyErr_Occurred())) {
    return false;
  }
  if (UNLIKELY(value < INT8_MIN || value > INT8_MAX)) {
    return false;
  }
  static_cast<int8_t *>(data)[index] = int8_t(value);
  return true;
}

static bool set_attribute_uint8(void *data, int index, PyObject *py_value)
{
  const int value = _PyLong_AsInt(py_value);
  if (UNLIKELY(value == -1 && PyErr_Occurred())) {
    return false;
  }
  if (UNLIKELY(value < 0 || value > UINT8_MAX)) {
    return false;
  }
  static_cast<uint8_t *>(data)[index] = uint8_t(value);
  return true;
}

static bool set_attribute_int32(void *data, int index, PyObject *py_value)
{
  if (!PyLong_Check(py_value)) {
    return false;
  }
  const long value = PyLong_AsLong(py_value);
  if (UNLIKELY(value < INT32_MIN || value > INT32_MAX)) {
    return false;
  }
  static_cast<int32_t *>(data)[index] = int32_t(value);
  return true;
}

static bool set_attribute_int2(void *data, int index, PyObject *py_value)
{
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 2) {
      return false;
    }
    int2 &set_value = static_cast<int2 *>(data)[index];
    for (int i = 0; i < 2; i++) {
      if (!set_attribute_int32(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 2) {
      return false;
    }
    int2 &set_value = static_cast<int2 *>(data)[index];
    for (int i = 0; i < 2; i++) {
      if (!set_attribute_int32(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  return false;
}

static bool set_attribute_string(void *data, int index, PyObject *py_value)
{
  if (!PyUnicode_Check(py_value)) {
    return false;
  }
  const std::string value = std::string(PyUnicode_AsUTF8(py_value));
  if (value.length() > 254) {
    return false;
  }
  strcpy(static_cast<MStringProperty *>(data)[index].s, value.c_str());
  static_cast<MStringProperty *>(data)[index].s_len = value.length();
  return true;
}

static bool set_attribute_float2(void *data, int index, PyObject *py_value)
{
  if (VectorObject_Check(py_value)) {
    const VectorObject *value = reinterpret_cast<VectorObject *>(py_value);
    if (value->vec_num != 2) {
      return false;
    }
    static_cast<float2 *>(data)[index] = {value->vec[0], value->vec[1]};
    return true;
  }
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 2) {
      return false;
    }
    float2 &set_value = static_cast<float2 *>(data)[index];
    for (int i = 0; i < 2; i++) {
      if (!set_attribute_float(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 2) {
      return false;
    }
    float2 &set_value = static_cast<float2 *>(data)[index];
    for (int i = 0; i < 2; i++) {
      if (!set_attribute_float(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  return false;
}

static bool set_attribute_float3(void *data, int index, PyObject *py_value)
{
  if (VectorObject_Check(py_value)) {
    const VectorObject *value = reinterpret_cast<VectorObject *>(py_value);
    if (value->vec_num != 3) {
      return false;
    }
    static_cast<float3 *>(data)[index] = {value->vec[0], value->vec[1], value->vec[2]};
    return true;
  }
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 3) {
      return false;
    }
    float3 &set_value = static_cast<float3 *>(data)[index];
    for (int i = 0; i < 3; i++) {
      if (!set_attribute_float(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 3) {
      return false;
    }
    float3 &set_value = static_cast<float3 *>(data)[index];
    for (int i = 0; i < 3; i++) {
      if (!set_attribute_float(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  return false;
}

static bool set_attribute_quaternion(void *data, int index, PyObject *py_value)
{
  if (QuaternionObject_Check(py_value)) {
    const QuaternionObject *value = reinterpret_cast<QuaternionObject *>(py_value);
    static_cast<math::Quaternion *>(data)[index] = {
        value->quat[0], value->quat[1], value->quat[2], value->quat[3]};
    return true;
  }
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    float4 &set_value = static_cast<float4 *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_float(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    float4 &set_value = static_cast<float4 *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_float(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  return false;
}

static bool set_attribute_float4x4(void *data, int index, PyObject *py_value)
{
  if (!MatrixObject_Check(py_value)) {
    return false;
  }
  MatrixObject &value = *reinterpret_cast<MatrixObject *>(py_value);
  if (value.col_num != 4 || value.row_num != 4) {
    return false;
  }
  std::copy_n(value.matrix, 16, static_cast<float4x4 *>(data)[index].base_ptr());
  return true;
}

static bool set_attribute_color(void *data, int index, PyObject *py_value)
{
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    ColorGeometry4f &set_value = static_cast<ColorGeometry4f *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_float(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    ColorGeometry4f &set_value = static_cast<ColorGeometry4f *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_float(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  return false;
}

static bool set_attribute_byte_color(void *data, int index, PyObject *py_value)
{
  if (PyTuple_Check(py_value)) {
    const Py_ssize_t size = PyTuple_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    ColorGeometry4b &set_value = static_cast<ColorGeometry4b *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_uint8(set_value, i, PyTuple_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }
  if (PyList_Check(py_value)) {
    const Py_ssize_t size = PyList_GET_SIZE(py_value);
    if (size != 4) {
      return false;
    }
    ColorGeometry4b &set_value = static_cast<ColorGeometry4b *>(data)[index];
    for (int i = 0; i < 4; i++) {
      if (!set_attribute_uint8(set_value, i, PyList_GET_ITEM(py_value, i))) {
        return false;
      }
    }
    return true;
  }

  return false;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Get attribute
 * \{ */

PyObject *get_attribute_bool(void *data, int index)
{
  return PyBool_FromLong(static_cast<bool *>(data)[index]);
}

PyObject *get_attribute_float(void *data, int index)
{
  return PyFloat_FromDouble(static_cast<float *>(data)[index]);
}

PyObject *get_attribute_int8(void *data, int index)
{
  return PyLong_FromLong(static_cast<int8_t *>(data)[index]);
}

PyObject *get_attribute_int32(void *data, int index)
{
  return PyLong_FromLong(static_cast<int32_t *>(data)[index]);
}

PyObject *get_attribute_int2(void *data, int index)
{
  PyObject *result = PyTuple_New(2);
  for (int i = 0; i < 2; i++) {
    PyTuple_SET_ITEM(result, i, PyLong_FromLong((static_cast<int2 *>(data)[index])[i]));
  }
  return result;
}

PyObject *get_attribute_string(void *data, int index)
{
  return PyUnicode_FromStringAndSize(static_cast<MStringProperty *>(data)[index].s,
                                     static_cast<MStringProperty *>(data)[index].s_len);
}

PyObject *get_attribute_float2(void *data, int index)
{
  return Vector_CreatePyObject_wrap(static_cast<float2 *>(data)[index], 2, nullptr);
}

PyObject *get_attribute_float3(void *data, int index)
{
  return Vector_CreatePyObject_wrap(static_cast<float3 *>(data)[index], 3, nullptr);
}

PyObject *get_attribute_float4x4(void *data, int index)
{
  return Matrix_CreatePyObject_wrap(
      static_cast<float *>((static_cast<float4x4 *>(data)[index]).base_ptr()), 4, 4, nullptr);
}

PyObject *get_attribute_quaternion(void *data, int index)
{
  return Quaternion_CreatePyObject_wrap(static_cast<float4 *>(data)[index], nullptr);
}

PyObject *get_attribute_color(void *data, int index)
{
  PyObject *result = PyTuple_New(4);
  for (int i = 0; i < 4; i++) {
    PyTuple_SET_ITEM(
        result, i, PyFloat_FromDouble((static_cast<ColorGeometry4f *>(data)[index])[i]));
  }
  return result;
}

PyObject *get_attribute_byte_color(void *data, int index)
{
  PyObject *result = PyTuple_New(4);
  for (int i = 0; i < 4; i++) {
    PyTuple_SET_ITEM(result, i, PyLong_FromLong((static_cast<ColorGeometry4b *>(data)[index])[i]));
  }
  return result;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name BPy_AttributeArray Map attribute types to functions
 * \{ */

bool set_attribute_by_type(BPy_AttributeArray *self, int index, PyObject *value)
{
  void *data = self->data_layer->data;
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return set_attribute_bool(data, index, value);
    case CD_PROP_FLOAT:
      return set_attribute_float(data, index, value);
    case CD_PROP_INT8:
      return set_attribute_int8(data, index, value);
    case CD_PROP_INT32:
      return set_attribute_int32(data, index, value);
    case CD_PROP_INT32_2D:
      return set_attribute_int2(data, index, value);
    case CD_PROP_STRING:
      return set_attribute_string(data, index, value);
    case CD_PROP_FLOAT2:
      return set_attribute_float2(data, index, value);
    case CD_PROP_FLOAT3:
      return set_attribute_float3(data, index, value);
    case CD_PROP_FLOAT4X4:
      return set_attribute_float4x4(data, index, value);
    case CD_PROP_QUATERNION:
      return set_attribute_quaternion(data, index, value);
    case CD_PROP_COLOR:
      return set_attribute_color(data, index, value);
    case CD_PROP_BYTE_COLOR:
      return set_attribute_byte_color(data, index, value);
  }
  return false;
}

PyObject *get_attribute_by_type(const BPy_AttributeArray *self, int index)
{
  void *data = self->data_layer->data;
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return get_attribute_bool(data, index);
    case CD_PROP_FLOAT:
      return get_attribute_float(data, index);
    case CD_PROP_INT8:
      return get_attribute_int8(data, index);
    case CD_PROP_INT32:
      return get_attribute_int32(data, index);
    case CD_PROP_INT32_2D:
      return get_attribute_int2(data, index);
    case CD_PROP_STRING:
      return get_attribute_string(data, index);
    case CD_PROP_FLOAT2:
      return get_attribute_float2(data, index);
    case CD_PROP_FLOAT3:
      return get_attribute_float3(data, index);
    case CD_PROP_FLOAT4X4:
      return get_attribute_float4x4(data, index);
    case CD_PROP_QUATERNION:
      return get_attribute_quaternion(data, index);
    case CD_PROP_COLOR:
      return get_attribute_color(data, index);
    case CD_PROP_BYTE_COLOR:
      return get_attribute_byte_color(data, index);
  }
  return nullptr;
}

int get_attribute_item_size(BPy_AttributeArray *self)
{
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return sizeof(bool);
    case CD_PROP_FLOAT:
      return sizeof(float);
    case CD_PROP_INT8:
      return sizeof(int8_t);
    case CD_PROP_INT32:
      return sizeof(int32_t);
    case CD_PROP_INT32_2D:
      return sizeof(int2);
    case CD_PROP_STRING:
      return sizeof(MStringProperty);
    case CD_PROP_FLOAT2:
      return sizeof(float2);
    case CD_PROP_FLOAT3:
      return sizeof(float3);
    case CD_PROP_FLOAT4X4:
      return sizeof(float4x4);
    case CD_PROP_QUATERNION:
      return sizeof(math::Quaternion);
    case CD_PROP_COLOR:
      return sizeof(ColorGeometry4f);
    case CD_PROP_BYTE_COLOR:
      return sizeof(ColorGeometry4b);
  }
  return 0;
}

StringRef get_attribute_type_description(const BPy_AttributeArray *self)
{
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return "bool value True/False or 0/1";
    case CD_PROP_FLOAT:
      return "float";
    case CD_PROP_INT8:
      return "int8 between -128 and 127";
    case CD_PROP_INT32:
      return "int";
    case CD_PROP_INT32_2D:
      return "tuple (x, y) or list [x, y] of ints";
    case CD_PROP_STRING:
      return "string with a maximum of 254 characters";
    case CD_PROP_FLOAT2:
      return "mathutils.Vector((x, y)), tuple (x, y) or list [x, y]";
    case CD_PROP_FLOAT3:
      return "mathutils.Vector((x, y, z)), tuple (x, y, z) or list [x, y, z]";
    case CD_PROP_FLOAT4X4:
      return "mathutils.Matrix() with 4x4 dimensions";
    case CD_PROP_QUATERNION:
      return "mathutils.Quaternion((w, x, y, z)), tuple (w, x, y, z) or list [w, x, y, z]";
    case CD_PROP_COLOR:
      return "tuple (r, g, b, a) or list [r, g, b, a] of floats";
    case CD_PROP_BYTE_COLOR:
      return "tuple (r, g, b, a) or list [r, g, b, a] of int in the range 0-255";
  }
  return "";
}

char get_buffer_format_by_type(BPy_AttributeArray *self)
{
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return '?';
    case CD_PROP_FLOAT:
      return 'f';
    case CD_PROP_INT8:
      return 'b';
    case CD_PROP_INT32:
      return 'i';
    case CD_PROP_INT32_2D:
      return 'i';
    case CD_PROP_STRING:
      return 'B';
    case CD_PROP_FLOAT2:
      return 'f';
    case CD_PROP_FLOAT3:
      return 'f';
    case CD_PROP_FLOAT4X4:
      return 'f';
    case CD_PROP_QUATERNION:
      return 'f';
    case CD_PROP_COLOR:
      return 'f';
    case CD_PROP_BYTE_COLOR:
      return 'B';
  }
  return ' ';
}

bool check_buffer_format(const BPy_AttributeArray *self, const char *buffer_format)
{
  const char f = buffer_format ? *buffer_format : 'B';
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
      return f == '?';
    case CD_PROP_FLOAT:
      return f == 'f';
    case CD_PROP_INT8:
      return f == 'b';
    case CD_PROP_INT32:
      return f == 'i' || f == 'l';
    case CD_PROP_INT32_2D:
      return f == 'i' || f == 'l';
    case CD_PROP_STRING:
      return f == 'B';
    case CD_PROP_FLOAT2:
      return f == 'f';
    case CD_PROP_FLOAT3:
      return f == 'f';
    case CD_PROP_FLOAT4X4:
      return f == 'f';
    case CD_PROP_QUATERNION:
      return f == 'f';
    case CD_PROP_COLOR:
      return f == 'f';
    case CD_PROP_BYTE_COLOR:
      return f == 'B';
  }
  return false;
}

bool check_attribute_type(const BPy_AttributeArray *self)
{
  switch (self->data_layer->type) {
    case CD_PROP_BOOL:
    case CD_PROP_FLOAT:
    case CD_PROP_INT8:
    case CD_PROP_INT32:
    case CD_PROP_INT32_2D:
    case CD_PROP_STRING:
    case CD_PROP_FLOAT2:
    case CD_PROP_FLOAT3:
    case CD_PROP_FLOAT4X4:
    case CD_PROP_QUATERNION:
    case CD_PROP_COLOR:
    case CD_PROP_BYTE_COLOR:
      return true;
  }

  PyErr_Format(
      PyExc_TypeError,
      "AttributeArray[index]: unexpected error, data type for attribute '%.200s' not found",
      self->data_layer->name);
  return false;
}

}  // namespace blender::python::attributearray
