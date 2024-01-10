/* SPDX-FileCopyrightText: 2011 by Bastien Montagne. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#pragma once

#include "BLI_vector.hh"

struct GreasePencil;
namespace blender::bke::greasepencil {
class Drawing;
}

namespace blender::greasepencil {

Vector<bke::greasepencil::Drawing *> get_drawings_for_write(GreasePencil &grease_pencil,
                                                            int frame);

}
