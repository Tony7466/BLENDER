/* SPDX-FileCopyrightText: 2011 by Bastien Montagne. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MOD_grease_pencil_util.hh"

#include "BLI_set.hh"

#include "DNA_grease_pencil_types.h"

#include "BKE_grease_pencil.hh"

namespace blender::greasepencil {

using bke::greasepencil::Drawing;
using bke::greasepencil::Layer;

Vector<bke::greasepencil::Drawing *> get_drawings_for_write(GreasePencil &grease_pencil, int frame)
{
  /* Set of unique drawing indices. */
  Set<int> drawing_indices;
  for (Layer *layer : grease_pencil.layers_for_write()) {
    const int drawing_index = layer->drawing_index_at(frame);
    if (drawing_index >= 0) {
      drawing_indices.add(drawing_index);
    }
  }

  /* List of owned drawings, ignore drawing references to other data blocks. */
  Vector<bke::greasepencil::Drawing *> drawings;
  for (const int drawing_index : drawing_indices) {
    GreasePencilDrawingBase *drawing_base = grease_pencil.drawing(drawing_index);
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
      drawings.append(&drawing->wrap());
    }
  }
  return drawings;
}

}  // namespace blender::greasepencil
