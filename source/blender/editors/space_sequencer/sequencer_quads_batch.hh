/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#pragma once

#include "BLI_sys_types.h"

struct GPUVertBuf;
struct GPUIndexBuf;
struct GPUBatch;
struct ColorVertex;

class SeqQuadsBatch {
 public:
  SeqQuadsBatch();
  ~SeqQuadsBatch();

  void draw();
  void add_quad(float x1, float y1, float x2, float y2, const uchar color[4])
  {
    add_quad(x1, y1, x1, y2, x2, y1, x2, y2, color);
  }
  void add_quad(float x1,
                float y1,
                float x2,
                float y2,
                float x3,
                float y3,
                float x4,
                float y4,
                const uchar color[4]);
  void add_wire_quad(float x1, float y1, float x2, float y2, const uchar color[4]);
  void add_line(float x1, float y1, float x2, float y2, const uchar color[4]);

 private:
  static constexpr int MAX_QUADS = 1024;
  static constexpr int MAX_LINES = 4096;

  GPUVertBuf *vbo_quads = nullptr;
  GPUIndexBuf *ibo_quads = nullptr;
  GPUBatch *batch_quads = nullptr;
  ColorVertex *verts_quads = nullptr;
  int quads_num = 0;

  GPUVertBuf *vbo_lines = nullptr;
  GPUBatch *batch_lines = nullptr;
  ColorVertex *verts_lines = nullptr;
  int lines_num = 0;
};
