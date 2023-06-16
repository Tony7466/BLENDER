/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

class SpeakerPasses : public OverlayPasses {

  ExtraInstanceBuf speaker = extra_buf("speaker", shapes.speaker);

 public:
  SpeakerPasses(SelectionType selection_type,
                const ShapeCache &shapes,
                const GlobalsUboStorage &theme_colors,
                bool in_front)
      : OverlayPasses("Speakers", selection_type, shapes, theme_colors, in_front){};

  virtual void object_sync(const ObjectRef &ob_ref,
                           const select::ID select_id,
                           Resources &res,
                           const State &state) final override
  {
    if (ob_ref.object->type == OB_SPEAKER) {
      ExtraInstanceData data(ob_ref.object, res.object_wire_color(ob_ref, state));
      speaker.append(data, select_id);
    }
  }
};

using Speaker = OverlayType<SpeakerPasses>;

}  // namespace blender::draw::overlay
