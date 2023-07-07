/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "UI_interface.hh"

namespace blender::ui {

DragInfo::DragInfo(const wmDrag &drag, const wmEvent &event, const DropLocation drop_location)
    : drag_data(drag), event(event), drop_location(drop_location)
{
}

std::optional<DropLocation> DropTargetInterface::determine_drop_location(
    const wmEvent & /*event*/) const
{
  return DROP_INTO;
}

bool drop_target_can_drop(const DropTargetInterface &drop_target,
                          const wmDrag &drag,
                          const char **r_disabled_hint)
{
  return drop_target.can_drop(drag, r_disabled_hint);
}

bool drop_target_apply_drop(bContext &C,
                            const wmEvent &event,
                            const DropTargetInterface &drop_target,
                            const ListBase &drags)
{

  const char *disabled_hint_dummy = nullptr;
  LISTBASE_FOREACH (const wmDrag *, drag, &drags) {
    if (!drop_target.can_drop(*drag, &disabled_hint_dummy)) {
      return false;
    }

    std::optional<DropLocation> drop_location = drop_target.determine_drop_location(event);
    if (!drop_location) {
      return false;
    }

    const DragInfo drag_info{*drag, event, *drop_location};
    return drop_target.on_drop(&C, drag_info);
  }

  return false;
}

char *drop_target_tooltip(const DropTargetInterface &drop_target,
                          const wmDrag &drag,
                          const wmEvent &event)
{
  const char *disabled_hint_dummy = nullptr;
  if (!drop_target.can_drop(drag, &disabled_hint_dummy)) {
    return nullptr;
  }

  std::optional<DropLocation> drop_location = drop_target.determine_drop_location(event);
  if (!drop_location) {
    return nullptr;
  }

  const DragInfo drag_info{drag, event, *drop_location};
  const std::string tooltip = drop_target.drop_tooltip(drag_info);
  return tooltip.empty() ? nullptr : BLI_strdup(tooltip.c_str());
}

}  // namespace blender::ui
