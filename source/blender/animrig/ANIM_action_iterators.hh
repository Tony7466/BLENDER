#pragma once

#include "ANIM_action.hh"

struct FCurve;

namespace blender::animrig::iterators {

/* Iterator to run through all FCurves belonging to the handle of the Action. Only works on layered
 * actions.*/
class ActionFCurveIterator {
 public:
  ActionFCurveIterator(Action &action, slot_handle_t handle);
  ~ActionFCurveIterator() = default;

  ActionFCurveIterator &operator++();
  FCurve *operator*() const;
  FCurve *operator->();

  bool operator==(const ActionFCurveIterator &other);
  bool operator!=(const ActionFCurveIterator &other);

  ChannelBag *get_current_channel_bag();

 private:
  Action &action;
  slot_handle_t handle;
  int64_t current_layer_index;
  int64_t current_strip_index;
  int64_t current_fcurve_index;

  /* Those are pointers because they need to be able to be nullptr. */
  Layer *current_layer;
  Strip *current_strip;
  ChannelBag *current_channel_bag;
};

}  // namespace blender::animrig::iterators
