/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BKE_file_handler.hh"
#include "BKE_screen.hh"
#include "BLT_translation.h"
#include "ED_fileselect.hh"

#include "CLG_log.h"

#include "DNA_space_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.h"

#include "WM_api.hh"
#include "WM_types.hh"
#include "wm_event_system.hh"

#include "UI_interface.hh"

#include "io_drop_import_file.hh"

#include <fmt/format.h>
#include <numeric>
static CLG_LogRef LOG = {"io.drop_import_file"};

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

/**
 * Return a vector of file handlers that support any file path in `paths` and the call to
 * `poll_drop` returns #true. Unlike `BKE_file_handlers_poll_file_drop`, it ensures that file
 * handlers have a valid import operator.
 */
static blender::Vector<blender::bke::FileHandlerType *> drop_import_file_poll_file_handlers(
    const bContext *C, const blender::Span<std::string> paths, const bool quiet = true)
{
  auto file_handlers = blender::bke::file_handlers_poll_file_drop(C, paths);
  file_handlers.remove_if([quiet](const blender::bke::FileHandlerType *file_handler) {
    return WM_operatortype_find(file_handler->import_operator, quiet) == nullptr;
  });
  return file_handlers;
}

/**
 * Sets in PointerRNA `ptr` all paths, returns`true` if pointer supports multiple paths.
 */
static bool file_handler_import_operator_create_ptr(PointerRNA &ptr,
                                                    const blender::Span<std::string> paths)
{

  PropertyRNA *filepath_prop = RNA_struct_find_property_check(ptr, "filepath", PROP_STRING);
  if (filepath_prop) {
    RNA_property_string_set(&ptr, filepath_prop, paths[0].c_str());
  }

  PropertyRNA *directory_prop = RNA_struct_find_property_check(ptr, "directory", PROP_STRING);
  if (directory_prop) {
    char dir[FILE_MAX];
    BLI_path_split_dir_part(paths[0].c_str(), dir, sizeof(dir));
    RNA_property_string_set(&ptr, directory_prop, dir);
  }

  PropertyRNA *files_prop = RNA_struct_find_collection_property_check(
      ptr, "files", &RNA_OperatorFileListElement);
  if (files_prop) {
    RNA_property_collection_clear(&ptr, files_prop);
    for (const auto &index : paths.index_range()) {
      char file[FILE_MAX];
      BLI_path_split_file_part(paths[index].c_str(), file, sizeof(file));

      PointerRNA item_ptr{};
      RNA_property_collection_add(&ptr, files_prop, &item_ptr);
      RNA_string_set(&item_ptr, "name", file);
    }
  }
  const bool has_any_filepath_prop = filepath_prop || directory_prop || files_prop;
  /**
   * The `directory` and `files` properties are both required for handling multiple files, if
   * only one is defined means that the other is missing.
   */
  const bool has_missing_filepath_prop = bool(directory_prop) != bool(files_prop);

  if (!has_any_filepath_prop || has_missing_filepath_prop) {
    const char *message =
        "Expected operator properties filepath or files and directory not found. Refer to "
        "FileHandler documentation for details.";
    CLOG_WARN(&LOG, TIP_(message));
  }
  return directory_prop && files_prop;
}

struct ImportGroup {
  blender::Vector<std::string> extensions;
  blender::Vector<int> count;
  int active_file_handler = 0;
  std::string label;
  blender::Vector<blender::bke::FileHandlerType *> file_handlers;
  wmOperator *op;
};

struct DropImportData {
  blender::Vector<ImportGroup> import_groups;
  blender::Vector<std::string> paths;

  int count = 0;
  ~DropImportData()
  {
    for (auto &import_setting : import_groups) {
      if (!import_setting.op) {
        continue;
      }
      WM_operator_free(import_setting.op);
    }
  }
};

static DropImportData *active_drop_import_data = nullptr;

static void wm_drop_import_file_cancel(bContext * /*C*/, wmOperator *op)
{
  DropImportData *drop_import_data = static_cast<DropImportData *>(op->customdata);
  MEM_delete(drop_import_data);
  op->customdata = nullptr;
  active_drop_import_data = nullptr;
}

static int wm_drop_import_file_exec(bContext * /*C*/, wmOperator *op)
{
  wm_drop_import_file_cancel(nullptr, op);
  return OPERATOR_FINISHED;
}

static void drop_import_file_draw_import_operator(const bContext *C,
                                                  uiLayout *layout,
                                                  wmOperator *op)
{
  /** Hack: temporary hide, same as in space file. */
  const char *hide[] = {"filepath", "files", "directory", "filename"};
  for (int i = 0; i < ARRAY_SIZE(hide); i++) {
    PropertyRNA *prop = RNA_struct_find_property(op->ptr, hide[i]);
    if (prop) {
      RNA_def_property_flag(prop, PROP_HIDDEN);
    }
  }

  uiTemplateOperatorPropertyButs(
      C, layout, op, UI_BUT_LABEL_ALIGN_NONE, UI_TEMPLATE_OP_PROPS_SHOW_EMPTY);

  /** Revert hack. */
  for (int i = 0; i < ARRAY_SIZE(hide); i++) {
    PropertyRNA *prop = RNA_struct_find_property(op->ptr, hide[i]);
    if (prop) {
      RNA_def_property_clear_flag(prop, PROP_HIDDEN);
    }
  }
}

static void drop_import_file_exec_import(bContext &C, wmOperator *op)
{
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);
  for (auto &import_group : drop_import_data.import_groups) {
    blender::Vector<std::string> paths;
    for (auto &path : drop_import_data.paths) {
      const char *extension = BLI_path_extension(path.c_str());
      if (!extension) {
        continue;
      }
      if (!import_group.extensions.contains(extension)) {
        continue;
      }
      paths.append(path);
    }
    while (!paths.is_empty()) {
      const bool all = file_handler_import_operator_create_ptr(*import_group.op->ptr, paths);
      if (all) {
        paths.clear();
      }
      else {
        paths.remove(0);
      }
      WM_operator_name_call_ptr(
          &C, import_group.op->type, WM_OP_EXEC_DEFAULT, import_group.op->ptr, nullptr);
    }
  }
  UI_popup_handlers_remove_all(&C, &CTX_wm_window(&C)->modalhandlers);
}

static void wm_drop_import_file_draw(bContext *C, wmOperator *op)
{
  uiItemL(op->layout, op->type->name, ICON_NONE);
  uiLayout *row = uiLayoutRow(op->layout, false);

  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);

  uiLayout *col = uiLayoutColumn(row, false);
  uiLayout *sub_row1 = uiLayoutRow(col, false);
  uiLayout *box = uiLayoutBox(col);

  uiItemR(sub_row1, op->ptr, "import_group", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
  int group = RNA_enum_get(op->ptr, "import_group");
  std::string extension_count;
  for (int64_t idx = 0; idx < drop_import_data.import_groups[group].extensions.size(); idx++) {
    if (!extension_count.empty()) {
      extension_count += " ";
    }
    extension_count += fmt::format(TIP_("{} files {}/{}"),
                                   drop_import_data.import_groups[group].extensions[idx],
                                   drop_import_data.import_groups[group].count[idx],
                                   drop_import_data.count);
  }

  if (drop_import_data.import_groups[group].file_handlers.size() > 1) {
    uiItemR(box, op->ptr, "file_handler", eUI_Item_Flag(0), "", ICON_NONE);
  }

  drop_import_file_draw_import_operator(C, box, drop_import_data.import_groups[group].op);

  uiItemL(uiLayoutBox(box), extension_count.c_str(), ICON_INFO);

  uiLayout *sub_row = uiLayoutRow(op->layout, true);
  uiBlock *block = uiLayoutGetBlock(sub_row);
  uiBut *but = uiDefBut(block,
                        UI_BTYPE_BUT,
                        0,
                        fmt::format(TIP_("Import {} files"), drop_import_data.count).c_str(),
                        50,
                        50,
                        50,
                        UI_UNIT_Y,
                        nullptr,
                        0,
                        0,
                        0,
                        0,
                        "");
  UI_but_flag_enable(but, UI_BUT_ACTIVE_DEFAULT);
  UI_but_func_set(but, [op](bContext &C) -> void { drop_import_file_exec_import(C, op); });
}

static int wm_drop_import_file_invoke(bContext *C, wmOperator *op, const wmEvent * /*event*/)
{
  auto paths = drop_import_file_paths(op);
  if (paths.is_empty()) {
    return OPERATOR_CANCELLED;
  }
  auto file_handlers = drop_import_file_poll_file_handlers(C, paths, false);
  if (file_handlers.is_empty()) {
    return OPERATOR_CANCELLED;
  }

  DropImportData *drop_import_data = MEM_new<DropImportData>("");
  op->customdata = active_drop_import_data = drop_import_data;
  drop_import_data->paths = paths;

  for (const std::string &path : paths) {
    const char *extension = BLI_path_extension(path.c_str());
    if (!extension) {
      continue;
    }
    auto itr = std::find_if(
        drop_import_data->import_groups.begin(),
        drop_import_data->import_groups.end(),
        [extension](ImportGroup &group) { return group.extensions.contains(extension); });
    if (itr != drop_import_data->import_groups.end()) {
      int64_t index = itr->extensions.first_index_of(extension);
      itr->count[index]++;
      continue;
    }
    ImportGroup group{};
    group.extensions.append(extension);
    group.count.append(1);

    for (blender::bke::FileHandlerType *fh : file_handlers) {
      if (fh->file_extensions.contains(extension)) {
        group.file_handlers.append_non_duplicates(fh);
      }
    }
    if (group.file_handlers.is_empty()) {
      CLOG_WARN(&LOG, "%s skipped.", path.c_str());
      continue;
    }
    /** Join extensions with same file handlers. */
    bool append = true;
    for (auto &&test_group : drop_import_data->import_groups) {
      if (test_group.file_handlers == group.file_handlers) {
        test_group.extensions.extend(group.extensions);
        test_group.count.extend(group.count);
        append = false;
      }
    }
    if (append) {
      drop_import_data->import_groups.append(std::move(group));
    }
  }

  for (auto &&group : drop_import_data->import_groups) {
    drop_import_data->count += std::accumulate(group.count.begin(), group.count.end(), 0);
    group.op = wm_operator_create(
        CTX_wm_manager(C),
        WM_operatortype_find(group.file_handlers[0]->import_operator, false),
        nullptr,
        CTX_wm_reports(C));
    WM_operator_last_properties_init(group.op);
    if (group.file_handlers.size() == 1) {
      group.label = group.file_handlers[0]->label;
      continue;
    }
    for (std::string &extension : group.extensions) {
      if (!group.label.empty()) {
        group.label += " ";
      }
      group.label += extension;
    }
  }
  return WM_operator_ui_popup(C, op, 400);
}

const EnumPropertyItem *RNA_file_hadler_for_drop_itemf(bContext * /*C*/,
                                                       PointerRNA *ptr,
                                                       PropertyRNA * /*prop*/,
                                                       bool *r_free)
{
  int group = RNA_enum_get(ptr, "import_group");

  int totitem = 0;
  int i = 0;
  EnumPropertyItem *item = nullptr;
  for (auto fh : active_drop_import_data->import_groups[group].file_handlers) {
    EnumPropertyItem item_tmp = {0};
    item_tmp.identifier = fh->idname;
    item_tmp.name = fh->label;
    item_tmp.value = i++;
    RNA_enum_item_add(&item, &totitem, &item_tmp);
  }
  RNA_enum_item_end(&item, &totitem);
  *r_free = true;
  return item;
}

const EnumPropertyItem *RNA_import_group_itemf(bContext *C,
                                               PointerRNA * /*ptr*/,
                                               PropertyRNA * /*prop*/,
                                               bool *r_free)
{
  auto fhs = drop_import_file_poll_file_handlers(
      C, active_drop_import_data->paths.as_span(), false);

  int totitem = 0;
  int i = 0;
  EnumPropertyItem *item = nullptr;
  for (auto &import_group : active_drop_import_data->import_groups) {
    EnumPropertyItem item_tmp = {0};
    item_tmp.identifier = import_group.extensions[0].c_str();
    item_tmp.name = import_group.label.c_str();
    item_tmp.value = i++;
    RNA_enum_item_add(&item, &totitem, &item_tmp);
  }
  RNA_enum_item_end(&item, &totitem);
  *r_free = true;
  return item;
}

static void drop_import_file_handler_update(bContext *C, PointerRNA *ptr, PropertyRNA * /*prop*/)
{
  const int group = RNA_enum_get(ptr, "import_group");
  DropImportData &drop_import_data = *active_drop_import_data;
  ImportGroup &import_group = drop_import_data.import_groups[group];
  const int active_file_handler = RNA_enum_get(ptr, "file_handler");

  import_group.active_file_handler = active_file_handler;
  WM_operator_free(import_group.op);

  import_group.op = wm_operator_create(
      CTX_wm_manager(C),
      WM_operatortype_find(import_group.file_handlers[active_file_handler]->import_operator,
                           false),
      nullptr,
      CTX_wm_reports(C));
  WM_operator_last_properties_init(import_group.op);
}

static void drop_import_group_update(bContext * /*C*/, PointerRNA *ptr, PropertyRNA * /*prop*/)
{
  const int group = RNA_enum_get(ptr, "import_group");
  DropImportData &drop_import_data = *active_drop_import_data;
  ImportGroup &import_group = drop_import_data.import_groups[group];
  RNA_enum_set(ptr, "file_handler", import_group.active_file_handler);
}

void WM_OT_drop_import_file(wmOperatorType *ot)
{
  ot->name = "Drop to Import File";
  ot->description = "Operator that allows file handlers to receive file drops";
  ot->idname = "WM_OT_drop_import_file";
  ot->flag = OPTYPE_INTERNAL;
  ot->exec = wm_drop_import_file_exec;
  ot->ui = wm_drop_import_file_draw;
  ot->cancel = wm_drop_import_file_cancel;
  ot->invoke = wm_drop_import_file_invoke;

  PropertyRNA *prop;

  prop = RNA_def_string_dir_path(
      ot->srna, "directory", nullptr, FILE_MAX, "Directory", "Directory of the file");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_collection_runtime(ot->srna, "files", &RNA_OperatorFileListElement, "Files", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna, "file_handler", rna_enum_dummy_NULL_items, 0, "File Handler", "");
  RNA_def_enum_funcs(prop, RNA_file_hadler_for_drop_itemf);
  RNA_def_property_update_runtime_with_context_and_property(prop, drop_import_file_handler_update);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna, "import_group", rna_enum_dummy_NULL_items, 0, "Import Group", "");
  RNA_def_enum_funcs(prop, RNA_import_group_itemf);
  RNA_def_property_update_runtime_with_context_and_property(prop, drop_import_group_update);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

void drop_import_file_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
{
  const auto paths = WM_drag_get_paths(drag);

  char dir[FILE_MAX];
  BLI_path_split_dir_part(paths[0].c_str(), dir, sizeof(dir));
  RNA_string_set(drop->ptr, "directory", dir);

  RNA_collection_clear(drop->ptr, "files");
  for (const int64_t idx : paths.index_range()) {
    char file[FILE_MAX];
    BLI_path_split_file_part(paths[idx].c_str(), file, sizeof(file));
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
  return !drop_import_file_poll_file_handlers(C, paths, true).is_empty();
}

static char *drop_import_file_tooltip(bContext *C,
                                      wmDrag *drag,
                                      const int /*xy*/[2],
                                      wmDropBox * /*drop*/)
{
  const auto paths = WM_drag_get_paths(drag);
  const auto file_handlers = drop_import_file_poll_file_handlers(C, paths, true);
  if (file_handlers.size() == 1) {
    wmOperatorType *ot = WM_operatortype_find(file_handlers[0]->import_operator, false);
    return BLI_strdup(TIP_(ot->name));
  }

  return BLI_strdup(TIP_("Multiple file handlers can be used, drop to pick which to use"));
}

void ED_dropbox_drop_import_file()
{
  ListBase *lb = WM_dropboxmap_find("Window", SPACE_EMPTY, RGN_TYPE_WINDOW);
  WM_dropbox_add(lb,
                 "WM_OT_drop_import_file",
                 drop_import_file_poll,
                 drop_import_file_copy,
                 nullptr,
                 drop_import_file_tooltip);
}
