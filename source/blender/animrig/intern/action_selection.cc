#include "DNA_action_types.h"
#include "DNA_anim_types.h"

#include "BLI_listbase.h"

#include "BKE_anim_data.hh"
#include "BKE_fcurve.hh"

#include "ANIM_action.hh"
#include "ANIM_fcurve.hh"

namespace blender::animrig {

static void clear_selection_legacy_action(Action &action)
{
  BLI_assert(action.is_action_legacy());
  LISTBASE_FOREACH (FCurve *, fcu, &action.curves) {
    if (!fcu->bezt) {
      continue;
    }
    BKE_fcurve_deselect_all_keys(*fcu);
  }
}

static void clear_selection_layered_action(Action &action)
{
  BLI_assert(action.is_action_layered());
  for (Layer *layer : action.layers()) {
    BLI_assert(layer != nullptr);
    for (Strip *strip : layer->strips()) {
      BLI_assert(strip != nullptr);
      if (!strip->is<KeyframeStrip>()) {
        continue;
      }
      KeyframeStrip &key_strip = strip->as<KeyframeStrip>();
      for (ChannelBag *ch_bag : key_strip.channelbags()) {
        BLI_assert(ch_bag != nullptr);
        for (FCurve *fcu : ch_bag->fcurves()) {
          BKE_fcurve_deselect_all_keys(*fcu);
        }
      }
    }
  }
}

void action_deselect_keys(Action &action)
{
  if (action.is_action_layered()) {
    clear_selection_layered_action(action);
  }
  else {
    BLI_assert(action.is_action_legacy());
    clear_selection_legacy_action(action);
  }
}

void deselect_keys_actions(Span<bAction *> actions)
{
  Set<bAction *> visited_actions;
  for (bAction *action : actions) {
    if (!visited_actions.add(action)) {
      continue;
    }
    action_deselect_keys(action->wrap());
  }
}

void deselect_keys_assigned_actions(Span<Object *> objects)
{
  Vector<bAction *> actions;
  for (Object *ob : objects) {
    AnimData *adt = BKE_animdata_from_id(&ob->id);
    if (!adt || !adt->action) {
      continue;
    }
    actions.append(adt->action);
  }

  deselect_keys_actions(actions);
}

}  // namespace blender::animrig
