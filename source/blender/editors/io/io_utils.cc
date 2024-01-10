/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "BLT_translation.h"

#include "DNA_space_types.h"

#include "ED_fileselect.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "BLI_math_vector_types.hh"
#include "UI_interface.hh"

#include "WM_api.hh"

#include "io_utils.hh"

#include <fmt/format.h>

namespace blender::ed::io {

struct OpPathDropPopUp {
  wmOperator *op;
  int width;
  int height;
};

static void operator_path_drop_dialog_exec(bContext *C, void *arg1, void *arg2)
{
  wmOperator *op;
  {
    /* Execute will free the operator.
     * In this case, #operator_path_drop_popup_cancel won't run. */
    OpPathDropPopUp *data = static_cast<OpPathDropPopUp *>(arg1);
    op = data->op;
    MEM_delete(data);
  }

  uiBlock *block = static_cast<uiBlock *>(arg2);
  /* Explicitly set #UI_RETURN_OK flag, otherwise the menu might be canceled
   * in case #WM_operator_call_ex exits/reloads the current file (#49199). */

  UI_popup_menu_retval_set(block, UI_RETURN_OK, true);

  /* Get context data *after* #WM_operator_call_ex
   * which might have closed the current file and changed context. */
  wmWindow *win = CTX_wm_window(C);
  UI_popup_block_close(C, win, block);

  WM_operator_call_ex(C, op, true);
}

static uiBlock *operator_path_drop_dialog_create(bContext *C, ARegion *region, void *user_data)
{
  OpPathDropPopUp *data = static_cast<OpPathDropPopUp *>(user_data);
  wmOperator *op = data->op;
  const uiStyle *style = UI_style_get_dpi();

  uiBlock *block = UI_block_begin(C, region, __func__, UI_EMBOSS);
  UI_block_flag_disable(block, UI_BLOCK_LOOP);
  UI_block_theme_style_set(block, UI_BLOCK_THEME_STYLE_REGULAR);

  /* Intentionally don't use #UI_BLOCK_MOVEMOUSE_QUIT, some dialogs have many items
   * where quitting by accident is very annoying. */
  UI_block_flag_enable(block, UI_BLOCK_KEEP_OPEN | UI_BLOCK_NUMSELECT);

  uiLayout *layout = UI_block_layout(
      block, UI_LAYOUT_VERTICAL, UI_LAYOUT_PANEL, 0, 0, data->width, data->height, 0, style);

  /* Draw our own title. */
  uiItemL(layout, WM_operatortype_name(op->type, op->ptr).c_str(), ICON_NONE);

  /* Draw our filepath label. */
  uiLayout *box = uiLayoutBox(layout);

  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "files");
  const int files_len = prop ? RNA_collection_length(op->ptr, "files") : 0;
  if (files_len < 2) {
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    uiItemL(box, filepath, ED_file_extension_icon(filepath));
  }
  else {
    std::string label = fmt::format(TIP_("Importing {} files."), files_len);
    uiItemL(box, label.c_str(), ICON_FILE_3D);
  }

  /* Invoke the Operator's draw callback. Note: do not use #UI_TEMPLATE_OP_PROPS_SHOW_TITLE
   * since we've drawn the title ourself above. */
  uiTemplateOperatorPropertyButs(C, layout, op, UI_BUT_LABEL_ALIGN_SPLIT_COLUMN, 0);

  /* Clear so the OK button is left alone */
  UI_block_func_set(block, nullptr, nullptr, nullptr);

  /* New column so as not to interfere with custom layouts #26436. */
  {
    uiLayout *col = uiLayoutColumn(layout, false);
    uiBlock *col_block = uiLayoutGetBlock(col);
    /* Create OK button, the callback of which will execute op. */
    uiBut *but = uiDefBut(
        col_block, UI_BTYPE_BUT, 0, IFACE_("OK"), 0, -30, 0, UI_UNIT_Y, nullptr, 0, 0, 0, 0, "");
    UI_but_flag_enable(but, UI_BUT_ACTIVE_DEFAULT);
    UI_but_func_set(but, operator_path_drop_dialog_exec, data, col_block);
  }

  /* Center around the mouse. */
  UI_block_bounds_set_popup(
      block, 6 * UI_SCALE_FAC, blender::int2{data->width / -2, data->height / 2});

  return block;
}

static void operator_path_drop_popup_cancel(bContext *C, void *user_data)
{
  OpPathDropPopUp *data = static_cast<OpPathDropPopUp *>(user_data);
  if (wmOperator *op = data->op; op) {
    if (op->type->cancel) {
      op->type->cancel(C, op);
    }
    WM_operator_free(op);
  }
  MEM_delete(data);
}

static void operator_path_drop_popup_ok(bContext *C, void *arg, int retval)
{
  OpPathDropPopUp *data = static_cast<OpPathDropPopUp *>(arg);
  if (data->op && retval > 0) {
    WM_operator_call_ex(C, data->op, true);
  }
  MEM_delete(data);
}

int operator_path_drop_dialog_popup(bContext *C, wmOperator *op, const int width)
{
  OpPathDropPopUp *data = MEM_cnew<OpPathDropPopUp>("OpPathDropPopUp");

  data->op = op;
  data->width = width * UI_SCALE_FAC;
  /* Actual height depends on the content. */
  data->height = 0;

  /* `op` is not executed until popup OK but is clicked. */
  UI_popup_block_ex(C,
                    operator_path_drop_dialog_create,
                    operator_path_drop_popup_ok,
                    operator_path_drop_popup_cancel,
                    data,
                    op);
  return OPERATOR_RUNNING_MODAL;
}

int filesel_drop_import_invoke(bContext *C, wmOperator *op, const wmEvent * /* event */)
{

  PropertyRNA *filepath_prop = RNA_struct_find_property(op->ptr, "filepath");
  PropertyRNA *directory_prop = RNA_struct_find_property(op->ptr, "directory");
  if ((filepath_prop && RNA_property_is_set(op->ptr, filepath_prop)) ||
      (directory_prop && RNA_property_is_set(op->ptr, directory_prop)))
  {
    return operator_path_drop_dialog_popup(C, op, 350);
  }

  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

void filepath_label_draw(const bContext *C, const wmOperator *op)
{
  if (SpaceFile *sfile = CTX_wm_space_file(C); sfile && sfile->op == op) {
    return;
  }
  /* Check if the operator is running in 3D viewport or the outliner, if so, that means that  the
   * operator is running as a dialog popup. */
  const View3D *v3d = CTX_wm_view3d(C);
  const SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  if (v3d || space_outliner) {
    uiLayout *box = uiLayoutBox(op->layout);

    PropertyRNA *prop = RNA_struct_find_property(op->ptr, "files");
    const int files_len = prop ? RNA_collection_length(op->ptr, "files") : 0;
    if (files_len < 2) {
      char filepath[FILE_MAX];
      RNA_string_get(op->ptr, "filepath", filepath);
      uiItemL(box, filepath, ED_file_extension_icon(filepath));
      return;
    }
    std::string label = fmt::format(TIP_("Importing {} files"), files_len);
    uiItemL(box, label.c_str(), ICON_FILE_3D);
  }
}

bool poll_file_object_drop(const bContext *C, blender::bke::FileHandlerType * /*fh*/)
{
  View3D *v3d = CTX_wm_view3d(C);
  SpaceOutliner *space_outliner = CTX_wm_space_outliner(C);
  ARegion *region = CTX_wm_region(C);
  if (!region || region->regiontype != RGN_TYPE_WINDOW) {
    return false;
  }
  if (v3d) {
    return true;
  }
  if (space_outliner && space_outliner->outlinevis == SO_VIEW_LAYER) {
    return true;
  }
  return false;
}
}  // namespace blender::ed::io
