/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 */

#include "ANIM_action.hh"
#include "ANIM_action_iterators.hh"
#include "BLI_assert.h"

namespace blender::animrig {

blender::Vector<FCurve *> foreach_fcurve(Action &action,
                                         slot_handle_t handle,
                                         FunctionRef<bool(FCurve &fcurve)> callback)
{
  blender::Vector<FCurve *> fcurves;
  for (Layer *layer : action.layers()) {
    for (Strip *strip : layer->strips()) {
      if (!strip->is<KeyframeStrip>()) {
        continue;
      }
      KeyframeStrip &key_strip = strip->as<KeyframeStrip>();
      for (ChannelBag *bag : key_strip.channelbags()) {
        if (bag->slot_handle != handle) {
          continue;
        }
        for (FCurve *fcu : bag->fcurves()) {
          BLI_assert(fcu != nullptr);
          if (callback(*fcu)) {
            fcurves.append(fcu);
          }
        }
      }
    }
  }
  return fcurves;
}

}  // namespace blender::animrig
