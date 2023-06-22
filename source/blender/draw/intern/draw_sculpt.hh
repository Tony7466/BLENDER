/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include "draw_manager.hh"

namespace blender::draw {

struct SculptBatch {
  GPUBatch *batch;
  int material_slot;
};

enum class SculptBatchFeature { WIRE, MASK, FSET, VERTEX_COLOR, UV };

Vector<SculptBatch> sculpt_batches_get(
    Object *ob, bool use_wire, bool use_mask, bool use_fset, bool use_color, bool use_uv);

}  // namespace blender::draw
