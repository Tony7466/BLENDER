/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#pragma once

#include <stddef.h>
#include <stdio.h>

#include "BLI_array.hh"
#include "BLI_span.hh"

namespace blender::io::ply {

class PlyReadBuffer {
 public:
  PlyReadBuffer(const char *file_path, size_t read_buffer_size = 64 * 1024);
  ~PlyReadBuffer();

  void after_header(bool is_binary);

  Span<char> read_line();
  bool read_bytes(void *dst, size_t size);

 private:
  bool refill_buffer();

 private:
  FILE *file_ = nullptr;
  Array<char> buffer_;
  int pos_ = 0;
  int buf_used_ = 0;
  int last_newline_ = 0;
  size_t read_buffer_size_ = 0;
  bool at_eof_ = false;
  bool is_binary_ = false;
};

}  // namespace blender::io::ply
