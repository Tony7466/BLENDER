/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation */

/** \file
 * \ingroup gpu
 *
 * Mimics old style OpenGL immediate mode drawing.
 */

#pragma once

#include "MEM_guardedalloc.h"

#include "gpu_immediate_private.hh"

#include "vk_buffer.hh"
#include "vk_context.hh"
#include "vk_mem_alloc.h"
#include "vk_vertex_attribute_object.hh"

namespace blender::gpu {

/* Size of internal buffer. */
constexpr size_t DEFAULT_INTERNAL_BUFFER_SIZE = (4 * 1024 * 1024);

class VKImmediate : public Immediate {
 private:
  VKBuffer buffer_;
  VKVertexAttributeObject vertex_attributes_;

 public:
  VKImmediate();
  virtual ~VKImmediate();

  uchar *begin(void) override;
  void end(void) override;

  friend class VKVertexAttributeObject;
};

}  // namespace blender::gpu