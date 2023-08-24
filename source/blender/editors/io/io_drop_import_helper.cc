/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_space_types.h"

#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "BKE_context.h"
#include "BKE_main.h"
#include "BKE_preferences.h"
#include "BKE_screen.h"

#include "ED_fileselect.hh"

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "io_drop_import_helper.hh"

static wmOperatorType *get_operator_type_for_extension(const bContext *C, const char *extension)
{
  ScrArea *area = CTX_wm_area(C);
  SpaceType *st = area->type;

  FileDropType *file_drop_type = nullptr;

  LISTBASE_FOREACH (FileDropType *, file_drop_test, &st->file_drop_types) {
    if (file_drop_test->poll_extension &&
        file_drop_test->poll_extension(C, file_drop_test, extension)) {
      file_drop_type = file_drop_test;
    }
  };
  return file_drop_type ? WM_operatortype_find(file_drop_type->op_name, false) : nullptr;
}

static int wm_drop_import_helper_exec(bContext *C, wmOperator *op)
{
  char extension[MAX_NAME];
  RNA_string_get(op->ptr, "extension", extension);

  wmOperatorType *ot = get_operator_type_for_extension(C, extension);

  if (!ot) {
    return OPERATOR_CANCELLED;
  }

  PointerRNA op_props;
  WM_operator_properties_create_ptr(&op_props, ot);

  char filepath[FILE_MAX];
  if (RNA_struct_find_property(&op_props, "filepath")) {
    RNA_string_get(op->ptr, "filepath", filepath);
    RNA_string_set(&op_props, "filepath", filepath);
  }

  if (RNA_struct_find_property(&op_props, "directory")) {
    RNA_string_get(op->ptr, "directory", filepath);
    RNA_string_set(&op_props, "directory", filepath);

    RNA_collection_clear(&op_props, "files");
    int files_len = RNA_collection_length(op->ptr, "files");
    PropertyRNA *prop = RNA_struct_find_property(op->ptr, "files");

    for (int i = 0; i < files_len; i++) {
      PointerRNA fileptr;
      RNA_property_collection_lookup_int(op->ptr, prop, i, &fileptr);
      RNA_string_get(&fileptr, "name", filepath);

      PointerRNA itemptr{};
      RNA_collection_add(&op_props, "files", &itemptr);
      RNA_string_set(&itemptr, "name", filepath);
    }
  }

  WM_operator_name_call_ptr(C, ot, WM_OP_INVOKE_DEFAULT, &op_props, nullptr);
  WM_operator_properties_free(&op_props);
  return OPERATOR_FINISHED;
}

void WM_OT_drop_import_helper(wmOperatorType *ot)
{
  ot->name = "Drop Import Herlper";
  ot->description =
      "Helper operator that handles file drops events and calls operators that can import files "
      "with extension";
  ot->idname = "WM_OT_drop_import_helper";

  ot->exec = wm_drop_import_helper_exec;
  PropertyRNA *prop;

  prop = RNA_def_string(ot->srna, "extension", nullptr, MAX_NAME, "extension", "extension");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
}

void files_drop_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  const auto paths = WM_drag_get_paths(drag);
  RNA_string_set(drop->ptr, "filepath", paths[0].c_str());

  char dir[FILE_MAX], file[FILE_MAX];
  RNA_string_set(drop->ptr, "extension", BLI_path_extension(paths[0].c_str()));

  BLI_path_split_dir_file(paths[0].c_str(), dir, sizeof(dir), file, sizeof(file));
  RNA_string_set(drop->ptr, "directory", dir);

  RNA_collection_clear(drop->ptr, "files");
  for (const auto &path : paths) {
    BLI_path_split_dir_file(path.c_str(), dir, sizeof(dir), file, sizeof(file));

    PointerRNA itemptr{};
    RNA_collection_add(drop->ptr, "files", &itemptr);
    RNA_string_set(&itemptr, "name", file);
  }
}

static bool poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  if (drag->type != WM_DRAG_PATH) {
    return false;
  }
  const auto paths = WM_drag_get_paths(drag);
  const char *extension = BLI_path_extension(paths[0].c_str());

  return extension && get_operator_type_for_extension(C, extension);
}
static char *tooltip(bContext *C, wmDrag *drag, const int /*xy*/[2], wmDropBox * /*drop*/)
{
  const auto paths = WM_drag_get_paths(drag);
  const char *extension = BLI_path_extension(paths[0].c_str());

  wmOperatorType *ot = get_operator_type_for_extension(C, extension);

  if (!ot || (!ot->name || !ot->description)) {
    return nullptr;
  }
  return BLI_strdup(ot->name ? ot->name : ot->description);
}

void ED_dropbox_import_helper()
{
  ListBase *lb = WM_dropboxmap_find("Window", 0, 0);
  WM_dropbox_add(lb, "WM_OT_drop_import_helper", poll, files_drop_copy, nullptr, tooltip);
}
