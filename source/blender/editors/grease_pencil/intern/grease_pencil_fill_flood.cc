/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "grease_pencil_fill.hh"

namespace blender::ed::greasepencil::fill {

bool flood_fill_do(FillData *fd)
{
  /* TODO... */
  printf("Perform flood fill algorithm on %lld strokes at frame %d \n",
         fd->curves_2d.point_offset.size(),
         fd->frame_number);

  return true;
}

}  // namespace blender::ed::greasepencil::fill
