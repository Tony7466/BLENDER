#pragma once

#include <cstdint>

#include "BLI_vector.hh"
#include "DNA_action_types.h"

struct FCurve;
namespace blender::animrig {
class Action;
class Layer;
class Strip;
class ChannelBag;
}  // namespace blender::animrig

namespace blender::animrig::iterators {

using slot_handle_t = decltype(::ActionSlot::handle);

/* Iterator to run through all FCurves belonging to the handle of the Action. Only works on layered
 * actions.*/
class ActionFCurveIterator {
 public:
  ActionFCurveIterator(Action &action, slot_handle_t handle);
  ~ActionFCurveIterator() = default;

  ActionFCurveIterator &operator++();
  FCurve *operator*() const;
  FCurve *operator->();

  bool operator==(const ActionFCurveIterator &other) const;
  bool operator!=(const ActionFCurveIterator &other) const;

  ChannelBag *get_current_channel_bag();

 private:
  Action &action;
  slot_handle_t handle;
  int64_t current_fcurve_index = 0;

  /* Those are pointers because they need to be able to be nullptr. */
  Layer *current_layer = nullptr;
  Strip *current_strip = nullptr;
  ChannelBag *current_channel_bag = nullptr;
};

/**
 * Iterates over all FCurves of the given slot handle in the Action and executes the callback on
 * it. If the callback returns true, the FCurve is added to the Vector returned at the end of the
 * function.
 *
 * \param data Will get passed to the callback. Can be a nullptr if the callback can deal with
 * that.
 *
 * \returns A Vector of FCurves for which the callback returned true.
 */
blender::Vector<FCurve *> foreach_fcurve(Action &action,
                                         slot_handle_t handle,
                                         void *data,
                                         bool (*callback)(FCurve *fcurve, void *data));

}  // namespace blender::animrig::iterators
