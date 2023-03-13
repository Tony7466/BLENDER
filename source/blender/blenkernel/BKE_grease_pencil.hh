/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Low-level operations for grease pencil.
 */

#include "BLI_map.hh"
#include "DNA_grease_pencil_types.h"

namespace blender::bke {

class GreasePencilLayerRuntime {
 private:
  Map<int, int> frames;
};

// class GreasePencilLayer : ::GreasePencilLayer {
//  public:
//   GreasePencilLayer();
//   GreasePencilLayer(const GreasePencilLayer &other);
//   GreasePencilLayer(GreasePencilLayer &&other);
//   GreasePencilLayer &operator=(const GreasePencilLayer &other);
//   GreasePencilLayer &operator=(GreasePencilLayer &&other);
//   ~GreasePencilLayer();

//   /* --------------------------------------------------------------------
//    * Accessors.
//    */

//   int frames_num() const;
// };

class GreasePencil : ::GreasePencil {
 public:
  GreasePencil() = delete;
  GreasePencil(const GreasePencil &other);
  GreasePencil(GreasePencil &&other);
  GreasePencil &operator=(const GreasePencil &other);
  GreasePencil &operator=(GreasePencil &&other);
  ~GreasePencil();

  /* --------------------------------------------------------------------
   * Accessors.
   */

  Span<GreasePencilDrawing> drawings() const;
};

}  // namespace blender::bke

inline blender::Map<int, int> GreasePencilLayer::frames() const
{
  return this->runtime->frames;
}
