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

#include "UI_interface.hh"

#include "io_drop_import_helper.hh"

static bool poll_extension(blender::Span<bFileExtension> file_extensions, const char *extension)
{
  for (auto &file_extension : file_extensions) {
    if (STREQ(extension, file_extension.extension)) {
      return true;
    }
  }
  return false;
}

static blender::Vector<FileHandlerType *> poll_file_handlers_for_extension(bContext *C,
                                                                           const char *extension)
{
  blender::Vector<FileHandlerType *> result;
  for (auto *file_handler : BKE_file_handlers()) {
    if (!poll_extension(file_handler->extensions, extension)) {
      continue;
    }
    if (!(file_handler->poll && file_handler->poll(C, file_handler))) {
      continue;
    }
    wmOperatorType *test_operator = WM_operatortype_find(file_handler->import_operator, false);
    if (!test_operator) {
      /* Discard if import operator does not exist. */
      continue;
    }
    result.append(file_handler);
  }
  return result;
}

static PointerRNA copy_file_properties_to_operator_type_pointer(wmOperator *op, wmOperatorType *ot)
{
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
  return op_props;
}

static int wm_drop_import_helper_exec(bContext *C, wmOperator *op)
{
  char extension[MAX_NAME];
  RNA_string_get(op->ptr, "extension", extension);

  blender::Vector<FileHandlerType *> file_handlers = poll_file_handlers_for_extension(C,
                                                                                      extension);

  if (file_handlers.size() == 0) {
    return OPERATOR_CANCELLED;
  }

  wmOperatorType *ot = WM_operatortype_find(file_handlers[0]->import_operator, false);

  PointerRNA op_props = copy_file_properties_to_operator_type_pointer(op, ot);

  WM_operator_name_call_ptr(C, ot, WM_OP_INVOKE_DEFAULT, &op_props, nullptr);
  WM_operator_properties_free(&op_props);
  return OPERATOR_FINISHED;
}

static int wm_drop_import_helper_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  char extension[MAX_NAME];
  RNA_string_get(op->ptr, "extension", extension);

  blender::Vector<FileHandlerType *> file_handlers = poll_file_handlers_for_extension(C,
                                                                                      extension);

  if (file_handlers.size() == 1) {
    return wm_drop_import_helper_exec(C, op);
  }

  uiPopupMenu *pup = UI_popup_menu_begin(C, "", ICON_NONE);
  uiLayout *layout = UI_popup_menu_layout(pup);
  uiLayoutSetOperatorContext(layout, WM_OP_INVOKE_DEFAULT);

  for (auto *file_handler : file_handlers) {
    wmOperatorType *ot = WM_operatortype_find(file_handler->import_operator, false);
    PointerRNA op_props = copy_file_properties_to_operator_type_pointer(op, ot);
    char *import_label = BLI_sprintfN("Import %s", file_handler->label);
    uiItemFullO_ptr(layout,
                    ot,
                    import_label,
                    ICON_NONE,
                    static_cast<IDProperty *>(op_props.data),
                    WM_OP_INVOKE_DEFAULT,
                    UI_ITEM_NONE,
                    nullptr);
    MEM_freeN(import_label);
  }

  UI_popup_menu_end(C, pup);
  return OPERATOR_INTERFACE;
}
void WM_OT_drop_import_helper(wmOperatorType *ot)
{
  ot->name = "Drop Import Herlper";
  ot->description =
      "Helper operator that handles file drops and uses file handler information to call "
      "operators that can import a file with extension.";
  ot->idname = "WM_OT_drop_import_helper";

  ot->exec = wm_drop_import_helper_exec;
  ot->invoke = wm_drop_import_helper_invoke;
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

  return extension && poll_file_handlers_for_extension(C, extension).size() > 0;
}
static char *tooltip(bContext *C, wmDrag *drag, const int /*xy*/[2], wmDropBox * /*drop*/)
{
  const auto paths = WM_drag_get_paths(drag);
  const char *extension = BLI_path_extension(paths[0].c_str());

  blender::Vector<FileHandlerType *> file_handlers = poll_file_handlers_for_extension(C,
                                                                                      extension);
  if (file_handlers.size() == 0) {
    return nullptr;
  }
  if (file_handlers.size() == 1) {
    return BLI_sprintfN("Import %s", file_handlers[0]->label);
  }

  return BLI_strdup("Multiple operators can handle this file(s), drop to pick which to use");
}

void ED_dropbox_import_helper()
{
  ListBase *lb = WM_dropboxmap_find("Window", 0, 0);
  WM_dropbox_add(lb, "WM_OT_drop_import_helper", poll, files_drop_copy, nullptr, tooltip);
}
