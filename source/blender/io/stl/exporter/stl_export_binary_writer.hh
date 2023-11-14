/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "BLI_assert.h"
#include "BLI_fileops.h"
#include "BLI_utility_mixins.hh"

#include "stl_export_writer.hh"

namespace blender::io::stl {

#pragma pack(push, 1)
struct STLBinaryTriangle {
  float normal[3]{};
  float vertices[3][3]{};
  uint16_t attribute_byte_count{};
};
#pragma pack(pop)
BLI_STATIC_ASSERT_ALIGN(STLBinaryTriangle,
                        sizeof(float[3]) + sizeof(float[3][3]) + sizeof(uint16_t));

class BinaryFileWriter : public FileWriter, NonCopyable {
 private:
  FILE *file_ = nullptr;
  uint32_t tris_num_ = 0;
  static constexpr size_t BINARY_HEADER_SIZE = 80;

 public:
  explicit BinaryFileWriter(const char *filepath);
  ~BinaryFileWriter() override;
  void write_triangle(const Triangle *t) override;
};

BinaryFileWriter::BinaryFileWriter(const char *filepath)
{
  file_ = BLI_fopen(filepath, "wb");
  if (file_ == nullptr) {
    throw std::runtime_error("Failed to open file");
  }

  char header[BINARY_HEADER_SIZE] = {};
  fwrite(header, 1, BINARY_HEADER_SIZE, file_);
  /* Write placeholder for number of triangles, so that it can be updated later (after all
   * triangles have been written). */
  fwrite(&tris_num_, sizeof(uint32_t), 1, file_);
}

void BinaryFileWriter::write_triangle(const Triangle *t)
{
  STLBinaryTriangle packed_triangle{};
  memcpy(packed_triangle.normal, t->normal, sizeof(float[3]));
  memcpy(packed_triangle.vertices, t->vertices, sizeof(float[3][3]));
  packed_triangle.attribute_byte_count = 0;

  if (fwrite(&packed_triangle, sizeof(STLBinaryTriangle), 1, file_) == 1) {
    tris_num_++;
  }
}

BinaryFileWriter::~BinaryFileWriter()
{
  assert(file_ != nullptr);
  fseek(file_, BINARY_HEADER_SIZE, SEEK_SET);
  fwrite(&tris_num_, sizeof(uint32_t), 1, file_);
  fclose(file_);
}

}  // namespace blender::io::stl
