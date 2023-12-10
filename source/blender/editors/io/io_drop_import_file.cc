/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_space_types.h"

#include "BLI_blenlib.h"
#include "BLI_utildefines.h"

#include "BLI_path_util.h"

#include "BLT_translation.h"

#include "BKE_context.hh"
#include "BKE_file_handler.hh"
#include "BKE_main.hh"
#include "BKE_preferences.h"

#include "ED_fileselect.hh"

#include <fmt/format.h>

#include "RNA_access.hh"
#include "RNA_define.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "UI_interface.hh"

#include "io_drop_import_file.hh"

/* Retuns the list of file paths stored in #WM_OT_drop_import_file operator properties. */
static blender::Vector<std::string> drop_import_file_paths(const wmOperator *op)
{
  blender::Vector<std::string> result;
  char dir[FILE_MAX], file[FILE_MAX];

  RNA_string_get(op->ptr, "directory", dir);

  PropertyRNA *prop = RNA_struct_find_property(op->ptr, "files");
  int files_len = RNA_property_collection_length(op->ptr, prop);

  for (int i = 0; i < files_len; i++) {
    PointerRNA fileptr;
    RNA_property_collection_lookup_int(op->ptr, prop, i, &fileptr);
    RNA_string_get(&fileptr, "name", file);
    char file_path[FILE_MAX];
    BLI_path_join(file_path, sizeof(file_path), dir, file);
    result.append(file_path);
  }
  return result;
}

static int wm_drop_import_file_exec(bContext *C, wmOperator *op)
{
  auto paths = drop_import_file_paths(op);
  if (paths.size() < 1) {
    return OPERATOR_CANCELLED;
  }

  auto file_handlers = BKE_file_handlers_poll_file_drop(C, paths);
  if (file_handlers.size() == 0) {
    return OPERATOR_CANCELLED;
  }

  PointerRNA file_props = file_handlers[0]->import_operator_file_properties_create_ptr(paths);

  WM_operator_name_call_ptr(
      C, file_handlers[0]->get_import_operator(), WM_OP_INVOKE_DEFAULT, &file_props, nullptr);
  WM_operator_properties_free(&file_props);
  return OPERATOR_FINISHED;
}

static int wm_drop_import_file_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  const auto paths = drop_import_file_paths(op);
  if (paths.size() < 1) {
    return OPERATOR_CANCELLED;
  }

  auto file_handlers = BKE_file_handlers_poll_file_drop(C, paths);
  /** Execute directly if the files are only supported by a file handler. */
  if (file_handlers.size() == 1) {
    return wm_drop_import_file_exec(C, op);
  }

  /** Create a menu with file handler import operators that can support the files and let user
   * decide what to use. */

  uiPopupMenu *pup = UI_popup_menu_begin(C, "", ICON_NONE);
  uiLayout *layout = UI_popup_menu_layout(pup);
  uiLayoutSetOperatorContext(layout, WM_OP_INVOKE_DEFAULT);

  for (auto *file_handler : file_handlers) {
    PointerRNA file_props = file_handler->import_operator_file_properties_create_ptr(paths);
    std::string label = fmt::format(TIP_("Import {}"), file_handler->label);
    uiItemFullO_ptr(layout,
                    file_handler->get_import_operator(),
                    label.c_str(),
                    ICON_NONE,
                    static_cast<IDProperty *>(file_props.data),
                    WM_OP_INVOKE_DEFAULT,
                    UI_ITEM_NONE,
                    nullptr);
  }

  UI_popup_menu_end(C, pup);
  return OPERATOR_INTERFACE;
}

void WM_OT_drop_import_file(wmOperatorType *ot)
{
  ot->name = "Drop to Import File";
  ot->description = "Manages file ";
  ot->idname = "WM_OT_drop_import_file";
  ot->flag = OPTYPE_INTERNAL;
  ot->exec = wm_drop_import_file_exec;
  ot->invoke = wm_drop_import_file_invoke;

  WM_operator_properties_filesel(ot,
                                 FILE_TYPE_FOLDER,
                                 FILE_BLENDER,
                                 FILE_OPENFILE,
                                 WM_FILESEL_FILEPATH | WM_FILESEL_FILES | WM_FILESEL_DIRECTORY |
                                     WM_FILESEL_SHOW_PROPS,
                                 FILE_DEFAULTDISPLAY,
                                 FILE_SORT_DEFAULT);
}

void drop_import_file_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  const auto paths = WM_drag_get_paths(drag);

  char dir[FILE_MAX];
  BLI_path_split_dir_part(paths[0].c_str(), dir, sizeof(dir));
  RNA_string_set(drop->ptr, "directory", dir);

  RNA_collection_clear(drop->ptr, "files");
  for (const auto &path : paths) {
    char file[FILE_MAX];
    BLI_path_split_file_part(path.c_str(), file, sizeof(file));

    PointerRNA itemptr{};
    RNA_collection_add(drop->ptr, "files", &itemptr);
    RNA_string_set(&itemptr, "name", file);
  }
}

static bool drop_import_file_poll(bContext *C, wmDrag *drag, const wmEvent * /*event*/)
{
  if (drag->type != WM_DRAG_PATH) {
    return false;
  }
  const auto paths = WM_drag_get_paths(drag);
  return BKE_file_handlers_poll_file_drop(C, paths).size() > 0;
}

static char *drop_import_file_tooltip(bContext *C,
                                      wmDrag *drag,
                                      const int /*xy*/[2],
                                      wmDropBox * /*drop*/)
{
  const auto paths = WM_drag_get_paths(drag);

  const auto file_handlers = BKE_file_handlers_poll_file_drop(C, paths);
  if (file_handlers.size() == 0) {
    return nullptr;
  }

  if (file_handlers.size() == 1) {
    std::string label = fmt::format(TIP_("Import {}"), file_handlers[0]->label);
    return BLI_strdup(label.c_str());
  }

  return BLI_strdup(TIP_("Multiple file handlers can be used, drop to pick which to use"));
}

void ED_dropbox_drop_import_file()
{
  ListBase *lb = WM_dropboxmap_find("Window", 0, 0);
  WM_dropbox_add(lb,
                 "WM_OT_drop_import_file",
                 drop_import_file_poll,
                 drop_import_file_copy,
                 nullptr,
                 drop_import_file_tooltip);
}
