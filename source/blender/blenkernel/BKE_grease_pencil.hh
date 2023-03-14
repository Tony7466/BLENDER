/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_map.hh"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

class GreasePencilLayerRuntime {
 public:
  Map<int, int> frames;
};

namespace gpencil::convert {

CurvesGeometry &legacy_gpencil_frame_to_curves_geometry(bGPDframe &gpf);

}  // namespace gpencil::convert

}  // namespace blender::bke

inline const blender::Map<int, int> &GreasePencilLayer::frames() const
{
  return this->runtime->frames;
}
