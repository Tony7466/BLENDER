/* SPDX-License-Identifier: GPL-2.0-or-later */

#if defined(WITH_COLLADA) || defined(WITH_IO_GPENCIL) || defined(WITH_IO_WAVEFRONT_OBJ) || \
    defined(WITH_IO_PLY) || defined(WITH_IO_STL) || defined(WITH_USD)

#  include "DNA_space_types.h"

#  include "BKE_context.h"

#  include "BLI_path_util.h"
#  include "BLI_string.h"
#  include "BLI_utildefines.h"
#  include "BLT_translation.h"

#  include "ED_fileselect.h"

#  include "RNA_access.h"
#  include "RNA_define.h"

#  include "UI_interface.h"

#  include "WM_api.h"
#  include "io_utils.h"

int wm_io_import_invoke(bContext *C, wmOperator *op, const wmEvent *UNUSED(event))
{
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);
  if (filepath[0]) {
    return WM_operator_props_dialog_popup(C, op, 300);
  }
  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

void skip_save_import_paths_props(wmOperatorType *ot, const eFileSel_Flag flag)
{
  if (flag & WM_FILESEL_FILEPATH) {
    RNA_def_property_flag(RNA_struct_type_find_property(ot->srna, "filepath"), PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_FILENAME) {
    RNA_def_property_flag(RNA_struct_type_find_property(ot->srna, "filename"), PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_DIRECTORY) {
    RNA_def_property_flag(RNA_struct_type_find_property(ot->srna, "directory"), PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_FILES) {
    RNA_def_property_flag(RNA_struct_type_find_property(ot->srna, "files"), PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_RELPATH) {
    RNA_def_property_flag(RNA_struct_type_find_property(ot->srna, "relative_path"),
                          PROP_SKIP_SAVE);
  }
}

void files_drop_copy(bContext *UNUSED(C), wmDrag *drag, wmDropBox *drop)
{

  RNA_string_set(drop->ptr, "filepath", WM_drag_get_path(drag));

  // TODO(@guishe): Add support for multiple drag&drop files import
  char dir[FILE_MAX], file[FILE_MAX];
  BLI_split_dirfile(WM_drag_get_path(drag), dir, file, sizeof(dir), sizeof(file));

  RNA_string_set(drop->ptr, "directory", dir);

  RNA_collection_clear(drop->ptr, "files");
  PointerRNA itemptr;
  RNA_collection_add(drop->ptr, "files", &itemptr);
  RNA_string_set(&itemptr, "name", file);
}

void file_drop_copy(bContext *UNUSED(C), wmDrag *drag, wmDropBox *drop)
{
  RNA_string_set(drop->ptr, "filepath", WM_drag_get_path(drag));
}

void file_drop_info_draw(bContext *C, wmOperator *op, int icon)
{
  ScrArea *area = CTX_wm_area(C);
  if (area->spacetype != SPACE_FILE) {
    char filepath[FILE_MAX];
    RNA_string_get(op->ptr, "filepath", filepath);
    uiLayout *box = uiLayoutBox(op->layout);
    uiItemL(box, filepath, icon);
  }
}

void files_drop_info_draw(bContext *C, wmOperator *op, int icon)
{
  ScrArea *area = CTX_wm_area(C);
  if (area->spacetype != SPACE_FILE) {
    char label[FILE_MAX] = "Multiple files selected.";

    if (RNA_collection_length(op->ptr, "files") == 1) {
      RNA_string_get(op->ptr, "filepath", label);
    }
    uiLayout *box = uiLayoutBox(op->layout);
    uiItemL(box, label, icon);
  }
}

#endif
