/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#pragma once

#include "BLI_math_vector_types.hh"

namespace blender::io::stl {

struct Triangle {
  float3 normal;
  float3 vertices[3];
};

class FileWriter {
 public:
  enum class Type { BINARY, ASCII };

  virtual ~FileWriter() = default;
  virtual void write_triangle(const Triangle &t) = 0;
};

std::unique_ptr<FileWriter> create_writer(const char *filepath, FileWriter::Type type);

}  // namespace blender::io::stl
