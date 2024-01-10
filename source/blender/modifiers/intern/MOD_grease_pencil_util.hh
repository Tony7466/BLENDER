/* SPDX-FileCopyrightText: 2011 by Bastien Montagne. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_vector.hh"

struct GreasePencil;
struct GreasePencilModifierFilterData;
namespace blender::bke::greasepencil {
class Drawing;
}

namespace blender::greasepencil {

IndexMask get_filtered_layer_mask(const GreasePencil &grease_pencil,
                                  const GreasePencilModifierFilterData &filter_data,
                                  IndexMaskMemory &memory);

Vector<bke::greasepencil::Drawing *> get_drawings_for_write(GreasePencil &grease_pencil,
                                                            const IndexMask &layer_mask,
                                                            int frame);

}  // namespace blender::greasepencil
