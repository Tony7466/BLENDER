/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup gpu
 */

#pragma once

#include "gpu_shader_create_info.hh"

namespace blender::gpu {

/**
 * Information about alignment/components and memory size for types when using std430 layout.
 */
struct Std430 {
  /** Get the memory size in bytes of a single component using by the given type.*/
  static uint32_t component_mem_size(const shader::Type type);
  /** Get to alignment of the given type in bytes.*/
  static uint32_t element_alignment(const shader::Type type, bool is_array);
  /** Get the number of components that should be allocated for the given type.*/
  static uint32_t element_components_len(const shader::Type type);
  /** Get the number of components of the given type when used in an array.*/
  static uint32_t array_components_len(const shader::Type type);
};

/**
 * Information about alignment/components and memory size for types when using std140 layout.
 */
struct Std140 {
  /** Get the memory size in bytes of a single component using by the given type.*/
  static uint32_t component_mem_size(const shader::Type type);
  /** Get to alignment of the given type in bytes.*/
  static uint32_t element_alignment(const shader::Type type, bool is_array);
  /** Get the number of components that should be allocated for the given type.*/
  static uint32_t element_components_len(const shader::Type type);
  /** Get the number of components of the given type when used in an array.*/
  static uint32_t array_components_len(const shader::Type type);
};

template<typename Layout>
static void align(const shader::Type &type, const int32_t array_size, uint32_t *r_offset)
{
  uint32_t alignment = Layout::element_alignment(type, array_size != 0);
  uint32_t alignment_mask = alignment - 1;
  uint32_t offset = *r_offset;
  if ((offset & alignment_mask) != 0) {
    offset &= ~alignment_mask;
    offset += alignment;
    *r_offset = offset;
  }
}

template<typename Layout> static uint32_t element_stride(const shader::Type type)
{
  return Layout::element_components_len(type) * Layout::component_mem_size(type);
}

template<typename Layout> static uint32_t array_stride(const shader::Type type)
{
  return Layout::array_components_len(type) * Layout::component_mem_size(type);
}

template<typename Layout>
static void reserve(const shader::Type type, int32_t array_size, uint32_t *r_offset)
{
  uint32_t size = array_size == 0 ? element_stride<Layout>(type) :
                                    array_stride<Layout>(type) * array_size;
  *r_offset += size;
}

template<typename Layout> static void align_end_of_struct(uint32_t *r_offset)
{
  align<Layout>(shader::Type::VEC4, 0, r_offset);
}

}  // namespace blender::gpu
