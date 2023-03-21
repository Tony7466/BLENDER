/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.hh"

using namespace blender::ui;

bool UI_drop_controller_apply_drop(bContext &C,
                                   const DropControllerInterface &drop_controller,
                                   const ListBase &drags)
{

  const char *disabled_hint_dummy = nullptr;
  LISTBASE_FOREACH (const wmDrag *, drag, &drags) {
    if (drop_controller.can_drop(*drag, &disabled_hint_dummy)) {
      return drop_controller.on_drop(&C, *drag);
    }
  }

  return false;
}

char *UI_drop_controller_drop_tooltip(const DropControllerInterface &drop_controller,
                                      const wmDrag &drag)
{
  const std::string tooltip = drop_controller.drop_tooltip(drag);
  return tooltip.empty() ? nullptr : BLI_strdup(tooltip.c_str());
}
