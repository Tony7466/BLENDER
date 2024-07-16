/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "ANIM_action.hh"
#include "ANIM_action_legacy.hh"

namespace blender::animrig::legacy {

constexpr const char *layer_default_name = "Layer";

static KeyframeStrip *first_keyframe_strip(Action &action)
{
  for (Layer *layer : action.layers()) {
    for (Strip *strip : layer->strips()) {
      if (strip->is<KeyframeStrip>()) {
        return &strip->as<KeyframeStrip>();
      }
    }
  }

  return nullptr;
}

ChannelBag *channelbag_get(Action &action)
{
  if (action.slots().is_empty()) {
    return nullptr;
  }

  KeyframeStrip *keystrip = first_keyframe_strip(action);
  if (!keystrip) {
    return nullptr;
  }

  return keystrip->channelbag_for_slot(*action.slot(0));
}

ChannelBag &channelbag_ensure(Action &action)
{
  Slot *slot;
  if (action.slots().is_empty()) {
    slot = &action.slot_add();
  }
  else {
    slot = action.slot(0);
  }

  KeyframeStrip *keystrip = first_keyframe_strip(action);
  if (!keystrip) {
    /* There are two scenarios now: there are no layers, or there are layers but
     * none have a keyframe strip. The good thing is that we can just add a
     * KeyframeStrip to the bottom-most empty layer without trouble, as that'll be
     * reliably found by first_keyframe_strip(). */
    for (Layer *layer : action.layers()) {
      if (layer->strips().is_empty()) {
        keystrip = &layer->strip_add<KeyframeStrip>();
        break;
      }
    }
  }
  if (!keystrip) {
    /* Either there were no layers, or none of them had space for a KeyframeStrip. */
    Layer &layer = action.layer_add(layer_default_name);
    keystrip = &layer.strip_add<KeyframeStrip>();
  }

  return keystrip->channelbag_for_slot_ensure(*slot);
}

}  // namespace blender::animrig::legacy
