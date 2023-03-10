/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup stl
 */

#pragma once

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "BLI_utility_mixins.hh"

#include "stl_export_writer.hh"

namespace blender::io::stl {

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
  file_ = fopen(filepath, "wb");
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
  bool success = (fwrite(t->normal, sizeof(float[3]), 1, file_) == 1);
  success = success && (fwrite(t->vertices, sizeof(float[3][3]), 1, file_) == 1);

  uint16_t attribute_byte_count = 0;
  success = success && (fwrite(&attribute_byte_count, sizeof(uint16_t), 1, file_) == 1);
  if (success) {
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
