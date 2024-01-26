/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_context.hh"

#include "BLT_translation.h"

#include "BLI_path_util.h"

#include "DNA_space_types.h"

#include "ED_fileselect.hh"

#include "RNA_access.hh"

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
    std::string confirm_text;
    PropertyRNA *files_prop = RNA_struct_find_property(op->ptr, "files");
    const int files_len = files_prop ? RNA_collection_length(op->ptr, "files") : 0;
    if (files_len < 2) {
      char filename[FILE_MAX]{0};
      if (filepath_prop && RNA_property_is_set(op->ptr, filepath_prop)) {
        char filepath[FILE_MAX];
        RNA_string_get(op->ptr, "filepath", filepath);
        BLI_path_split_file_part(filepath, filename, sizeof(filename));
      }
      else if (files_len == 1) {
        PointerRNA file_ptr;
        RNA_property_collection_lookup_int(op->ptr, files_prop, 0, &file_ptr);
        RNA_string_get(op->ptr, "name", filename);
      }
      confirm_text = fmt::format(TIP_("Import {}"), filename);
    }
    else {
      confirm_text = fmt::format(TIP_("Import {} files"), files_len);
    }
    return WM_operator_props_dialog_popup(
        C, op, 350, WM_operatortype_name(op->type, op->ptr).c_str(), confirm_text.c_str());
  }

  WM_event_add_fileselect(C, op);
  return OPERATOR_RUNNING_MODAL;
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
