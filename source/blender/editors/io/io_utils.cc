/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"
#include "BKE_screen.hh"

#include "BLI_path_util.h"
#include "BLI_string.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_space_types.h"

#include "ED_fileselect.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "WM_api.hh"

#include "io_utils.hh"

#include <fmt/format.h>

namespace blender::ed::io {
int filesel_drop_import_invoke(bContext *C, wmOperator *op, const wmEvent * /* event */)
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

void filepath_label_draw(bContext *C, wmOperator *op)
{
  if (SpaceFile *sfile = CTX_wm_space_file(C); sfile && sfile->op == op) {
    return;
  }
  /**
   * Check if the operator is running in 3D viewport or the outliner, if so, that means that  the
   * operator is running as a dialog popup.
   */
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
    std::string label = fmt::format(TIP_("Importing {} files."), files_len);
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
