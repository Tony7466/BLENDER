/* SPDX-License-Identifier: GPL-2.0-or-later */

#if defined(WITH_COLLADA) || defined(WITH_IO_GPENCIL) || defined(WITH_IO_WAVEFRONT_OBJ) || \
    defined(WITH_IO_PLY) || defined(WITH_IO_STL) || defined(WITH_USD)

#  include "BKE_context.h"

#  include "BLI_path_util.h"
#  include "BLI_string.h"
#  include "BLI_utildefines.h"

#  include "BLT_translation.h"

#  include "DNA_space_types.h"

#  include "ED_fileselect.hh"

#  include "RNA_access.hh"
#  include "RNA_define.hh"

#  include "UI_interface.hh"

#  include "WM_api.hh"

#  include "io_utils.hh"

int wm_io_import_invoke(bContext *C, wmOperator *op, const wmEvent * /* event */)
{
  char filepath[FILE_MAX];
  RNA_string_get(op->ptr, "filepath", filepath);

  if (filepath[0]) {
    return WM_operator_props_dialog_popup(C, op, 300);
  }

  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
}

void skip_filesel_props(wmOperatorType *ot, const eFileSel_Flag flag)
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

void files_drop_label_draw(bContext *C, wmOperator *op, int icon, const char *extension)
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

#endif
