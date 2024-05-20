/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup pygen
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

struct PointerRNA;
struct CustomDataLayer;

extern PyTypeObject BPy_AttributeArray_Type;

typedef struct BPy_AttributeArray {
  PyObject_VAR_HEAD
  CustomDataLayer *data_layer;
} BPy_AttributeArray;

PyObject *BPy_AttributeArray_CreatePyObject(PointerRNA *ptr);

void AttributeArray_Init_Types(void);

PyObject *BPyInit_attribute_array(void);

#ifdef __cplusplus
}
#endif
