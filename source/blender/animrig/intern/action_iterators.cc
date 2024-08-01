#include "ANIM_action_iterators.hh"

namespace blender::animrig::iterators {

/* Action FCurve iterator implementation. */

ActionFCurveIterator::ActionFCurveIterator(Action &action, slot_handle_t handle)
    : action(action), handle(handle)
{
  BLI_assert(action.is_action_layered());
  current_fcurve_index = 0;
  /* Assuming that there is a single keyframe strip on a single layer. */
  assert_baklava_phase_1_invariants(action);
  current_layer = action.layer(0);
  if (!current_layer) {
    return;
  }
  current_strip = current_layer->strip(0);
  if (!current_strip) {
    return;
  }
  current_channel_bag = current_strip->as<KeyframeStrip>().channelbag_for_slot(handle);
};

bool ActionFCurveIterator::operator==(const ActionFCurveIterator &other) const
{
  /* Iterators are the same if they dereference to the same FCurve pointer. */
  return *(*this) == *other;
}
bool ActionFCurveIterator::operator!=(const ActionFCurveIterator &other) const
{
  return !(*this == other);
}

ActionFCurveIterator &ActionFCurveIterator::operator++()
{
  current_fcurve_index++;
  return *this;
}

FCurve *ActionFCurveIterator::operator*() const
{
  if (!current_channel_bag) {
    return nullptr;
  }
  if (current_fcurve_index >= current_channel_bag->fcurve_array_num) {
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
