/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edinterface
 */

#include "BKE_context.hh"
#include "BKE_report.hh"
#include "BKE_screen.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_prototypes.h"

#include "UI_interface_c.hh"
#include "UI_resources.hh"
#include "interface_intern.hh"

#include "ED_asset_shelf.hh"

#include "WM_api.hh"

namespace blender::ui {

void template_asset_shelf_popover(uiLayout &layout,
                                  const bContext &C,
                                  const StringRefNull asset_shelf_id,
                                  const StringRefNull name,
                                  const BIFIconID icon)
{
  AssetShelfType *shelf_type = ed::asset::shelf::type_find_from_idname(asset_shelf_id);
  if (!shelf_type) {
    RNA_warning("Asset shelf type not found: %s", asset_shelf_id.c_str());
    return;
  }

  uiLayout *row = uiLayoutRow(&layout, true);
  const ARegion *region = CTX_wm_region(&C);
  const bool use_big_size = !RGN_TYPE_IS_HEADER_ANY(region->regiontype);
  const short width = [&]() -> short {
    if (use_big_size) {
      return UI_UNIT_X * 6;
    }
    return UI_UNIT_X * (name.is_empty() ? 1.6f : 7);
  }();
  const short height = UI_UNIT_Y * (use_big_size ? 6 : 1);

  uiLayoutSetContextString(row, "asset_shelf_idname", asset_shelf_id);
  uiLayoutSetUnitsX(row, width / UI_UNIT_X);
  uiLayoutSetUnitsY(row, height / UI_UNIT_Y);
  uiItemPopoverPanel(row, &C, "ASSETSHELF_PT_popover_panel", name.c_str(), icon);

#if 0
  const ARegion *region = CTX_wm_region(&C);
  const bool use_big_size = !RGN_TYPE_IS_HEADER_ANY(region->regiontype);
  const bool use_preview_icon = use_big_size;
  const short width = [&]() -> short {
    if (use_big_size) {
      return UI_UNIT_X * 6;
    }
    return UI_UNIT_X * (name.is_empty() ? 1.6f : 7);
  }();
  const short height = UI_UNIT_Y * (use_big_size ? 6 : 1);

  uiBlock *block = uiLayoutGetBlock(&layout);
  uiBut *but = uiDefBlockBut(
      block, asset_shelf_block_fn, shelf_type, name, 0, 0, width, height, "Select an asset");
  if (use_preview_icon) {
    ui_def_but_icon(but, icon, UI_HAS_ICON | UI_BUT_ICON_PREVIEW);
  }
  else {
    ui_def_but_icon(but, icon, UI_HAS_ICON);
    UI_but_drawflag_enable(but, UI_BUT_ICON_LEFT);
  }

  if (ed::asset::shelf::type_poll_for_popup(C, shelf_type) == false) {
    UI_but_flag_enable(but, UI_BUT_DISABLED);
  }
#endif
}

}  // namespace blender::ui

using namespace blender;

static int asset_shelf_popover_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  char *asset_shelf_id = RNA_string_get_alloc(op->ptr, "asset_shelf", nullptr, 0, nullptr);
  BLI_SCOPED_DEFER([&]() { MEM_freeN(asset_shelf_id); });

  AssetShelfType *shelf_type = ed::asset::shelf::type_find_from_idname(asset_shelf_id);
  if (ed::asset::shelf::type_poll_for_popup(*C, shelf_type) == false) {
    return OPERATOR_CANCELLED;
  }

  PanelType *pt = WM_paneltype_find("ASSETSHELF_PT_popover_panel", true);
  if (pt == nullptr) {
    BKE_reportf(op->reports, RPT_ERROR, "Asset shelf popover panel type not found");
    return OPERATOR_CANCELLED;
  }

  if (pt->poll && (pt->poll(C, pt) == false)) {
    /* cancel but allow event to pass through, just like operators do */
    return (OPERATOR_CANCELLED | OPERATOR_PASS_THROUGH);
  }

  std::string asset_shelf_id_str = asset_shelf_id;
#if 0
  // UI_popup_block_invoke(C, asset_shelf_block_fn, shelf_type, nullptr);
  ui_popover_panel_create(
      C,
      nullptr,
      nullptr,
      [asset_shelf_id_str](bContext *C, uiLayout *layout, void *arg_pt) {
        // uiLayoutsetP
        ui_item_paneltype_func(C, layout, arg_pt)
      },
      pt);
#endif

  return OPERATOR_INTERFACE;
}

void UI_OT_asset_shelf_popover(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Asset Shelf Popover";
  ot->idname = "UI_OT_asset_shelf_popover";

  /* api callbacks */
  ot->invoke = asset_shelf_popover_invoke;

  /* flags */
  ot->flag = OPTYPE_INTERNAL;

  RNA_def_string(ot->srna,
                 "asset_shelf",
                 nullptr,
                 0,
                 "Asset Shelf Name",
                 "Identifier of the asset shelf to display");
}
