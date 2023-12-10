/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_space_types.h"

#include "ED_fileselect.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"

#include "WM_api.hh"

#include "io_utils.hh"

int io_util_import_invoke(bContext *C, wmOperator *op, const wmEvent * /* event */)
{

  PropertyRNA *filepath_prop = RNA_struct_find_property(op->ptr, "filepath");
  PropertyRNA *directory_prop = RNA_struct_find_property(op->ptr, "directory");
  if ((filepath_prop && RNA_property_is_set(op->ptr, filepath_prop)) ||
      (directory_prop && RNA_property_is_set(op->ptr, directory_prop)))
  {
    return WM_operator_props_dialog_popup(C, op, 300);
  }

  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

void io_util_skip_save_filesel_props(wmOperatorType *ot, const eFileSel_Flag flag)
{
  PropertyRNA *prop;
  if (flag & WM_FILESEL_FILEPATH) {
    prop = RNA_struct_type_find_property(ot->srna, "filepath");
    RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_FILENAME) {
    prop = RNA_struct_type_find_property(ot->srna, "filename");
    RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_DIRECTORY) {
    prop = RNA_struct_type_find_property(ot->srna, "directory");
    RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_FILES) {
    prop = RNA_struct_type_find_property(ot->srna, "files");
    RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  }
  if (flag & WM_FILESEL_RELPATH) {
    prop = RNA_struct_type_find_property(ot->srna, "relative_path");
    RNA_def_property_flag(prop, PROP_SKIP_SAVE);
  }
}

void io_util_drop_file_label_draw(bContext *C, wmOperator *op, int icon, const char *extension)
{
  ScrArea *area = CTX_wm_area(C);

  if (area->spacetype == SPACE_FILE) {
    return;
  }

  char label[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", label);

  if (RNA_struct_find_property(op->ptr, "directory") &&
      RNA_collection_length(op->ptr, "files") > 1) {
    sprintf(label, "%d %s files dropped.", RNA_collection_length(op->ptr, "files"), extension);
  }

  uiLayout *box = uiLayoutBox(op->layout);
  uiItemL(box, label, icon);
}
