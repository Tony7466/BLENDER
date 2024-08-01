#include "ANIM_action_iterators.hh"

namespace blender::animrig::iterators {

/* Action FCurve iterator implementation. */

ActionFCurveIterator::ActionFCurveIterator(Action &action, slot_handle_t handle)
    : action(action), handle(handle)
{
  BLI_assert(action.is_action_layered());
  current_layer_index = 0;
  current_strip_index = 0;
  current_fcurve_index = 0;
  /* Assuming that all strips are keyframe strips. */
  assert_baklava_phase_1_invariants(action);
  current_layer = action.layer(current_layer_index);
  if (!current_layer) {
    return;
  }
  current_strip = current_layer->strip(current_strip_index);
  if (!current_strip) {
    return;
  }
  current_channel_bag = current_strip->as<KeyframeStrip>().channelbag_for_slot(handle);
};

bool ActionFCurveIterator::operator==(const ActionFCurveIterator &other)
{
  /* Iterators are the same if they dereference to the same FCurve pointer. */
  return *(*this) == *other;
}
bool ActionFCurveIterator::operator!=(const ActionFCurveIterator &other)
{
  return !(*this == other);
}

ActionFCurveIterator &ActionFCurveIterator::operator++()
{
  current_fcurve_index++;
  bool layer_changed = false;
  bool strip_changed = false;

  /* Iterated over all FCurves of the channel bag. */
  if (current_channel_bag && current_fcurve_index >= current_channel_bag->fcurve_array_num) {
    current_fcurve_index = 0;
    current_strip_index++;
    strip_changed = true;
  }

  /* Iterated over all FCurves of the layer. */
  if (current_strip && current_strip_index >= current_layer->strip_array_num) {
    current_strip_index = 0;
    current_layer_index++;
    layer_changed = true;
  }

  if (layer_changed) {
    /* Cannot change layer without advancing the strip. */
    BLI_assert(strip_changed);
    if (current_layer_index < action.layer_array_num) {
      current_layer = action.layer(current_layer_index);
    }
    else {
      current_layer = nullptr;
      current_channel_bag = nullptr;
      current_strip = nullptr;
    }
  }

  if (strip_changed && current_layer) {
    if (current_strip_index < current_layer->strip_array_num) {
      current_strip = current_layer->strip(current_strip_index);
    }
    else {
      current_strip = nullptr;
    }
    if (current_strip) {
      current_channel_bag = current_strip->as<KeyframeStrip>().channelbag_for_slot(handle);
    }
  }
  return *this;
}

FCurve *ActionFCurveIterator::operator*() const
{
  if (!current_channel_bag) {
    return nullptr;
  }
  return current_channel_bag->fcurve(current_fcurve_index);
}

FCurve *ActionFCurveIterator::operator->()
{
  if (!current_channel_bag) {
    return nullptr;
  }
  return current_channel_bag->fcurve(current_fcurve_index);
}

ChannelBag *ActionFCurveIterator::get_current_channel_bag()
{
  return current_channel_bag;
}

}  // namespace blender::animrig::iterators
