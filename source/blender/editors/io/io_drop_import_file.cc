/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BLT_translation.h"

#include "BKE_file_handler.hh"
#include "BKE_idprop.h"
#include "BKE_screen.hh"

#include "ED_fileselect.hh"

#include "CLG_log.h"

#include "DNA_space_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.h"
#include "RNA_types.hh"

#include "WM_api.hh"
#include "WM_types.hh"
#include "wm_event_system.hh"

#include "UI_interface.hh"

#include "io_drop_import_file.hh"

#include <fmt/format.h>
#include <numeric>

static CLG_LogRef LOG = {"io.drop_import_file"};

/** Returns the list of file paths stored in #WM_OT_drop_import_file operator properties. */
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
 * `poll_drop` returns #true. Unlike `bke::file_handlers_poll_file_drop`, it ensures that file
 * handlers have a valid import operator.
 */
static blender::Vector<blender::bke::FileHandlerType *> drop_import_file_poll_file_handlers(
    const bContext *C, const blender::Span<std::string> paths, const bool quiet = true)
{
  using namespace blender;
  auto file_handlers = bke::file_handlers_poll_file_drop(C, paths);
  file_handlers.remove_if([quiet](const bke::FileHandlerType *file_handler) {
    return WM_operatortype_find(file_handler->import_operator, quiet) == nullptr;
  });
  return file_handlers;
}

/**
 * Sets in the PointerRNA `ptr` all paths, returns`true` if pointer supports multiple paths.
 */
static bool file_handler_import_operator_paths_set(PointerRNA &ptr,
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
    CLOG_WARN(&LOG, "%s", message);
  }
  return directory_prop && files_prop;
}

/**
 * Set of extensions that are supported by set of file handlers joined in a tab.
 * Only the #active_file_handler would be used to import of files whose extension is
 * listed in the import tab.
 */
struct ImportTab {
  blender::Vector<std::string> extensions;
  blender::Vector<int> count;
  int active_file_handler = 0;
  std::string label;
  blender::Vector<blender::bke::FileHandlerType *> file_handlers;
  wmOperator *op = nullptr;
};

struct DropImportData {
  /** List of import tabs, each tab will execute the #active_file_handler in the tab. */
  blender::Vector<ImportTab> tabs;
  /** List of all file paths given to #WM_OT_drop_import_file. */
  blender::Vector<std::string> paths;
  /** Count of files that can be handled by any file handler. */
  int count = 0;
  std::string file_summary;
  int active_tab = 0;
  ~DropImportData()
  {
    for (auto &tab : tabs) {
      if (!tab.op) {
        continue;
      }
      WM_operator_free(tab.op);
    }
  }
};

static void wm_drop_import_file_cleanup(wmOperator *op)
{
  DropImportData *drop_import_data = static_cast<DropImportData *>(op->customdata);
  MEM_delete(drop_import_data);
  op->customdata = nullptr;
}

static void wm_drop_import_file_cancel(bContext * /*C*/, wmOperator *op)
{
  wm_drop_import_file_cleanup(op);
}

static int wm_drop_import_file_exec(bContext *C, wmOperator *op)
{
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);
  for (auto &tab : drop_import_data.tabs) {
    blender::Vector<std::string> paths;
    for (auto &path : drop_import_data.paths) {
      const char *extension = BLI_path_extension(path.c_str());
      if (!extension) {
        continue;
      }
      if (!tab.extensions.contains(extension)) {
        continue;
      }
      paths.append(path);
    }
    while (!paths.is_empty()) {
      const bool all = file_handler_import_operator_paths_set(*tab.op->ptr, paths);
      if (all) {
        paths.clear();
      }
      else {
        paths.remove(0);
      }
      WM_operator_name_call_ptr(C, tab.op->type, WM_OP_EXEC_DEFAULT, tab.op->ptr, nullptr);
    }
  }
  wm_drop_import_file_cleanup(op);
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

static void wm_drop_import_file_draw(bContext *C, wmOperator *op)
{
  uiLayout *row = uiLayoutRow(op->layout, false);
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);

  uiLayout *col = uiLayoutColumn(row, false);

  /* Tab enum. */
  PropertyRNA *tab_prop = RNA_struct_find_property(op->ptr, "tab");
  uiItemTabsEnumR_prop(uiLayoutRow(col, true), C, op->ptr, tab_prop, nullptr, nullptr, false);
  uiItemS_ex(col, 0.25f);

  auto &tab = drop_import_data.tabs[RNA_enum_get(op->ptr, "tab")];
  if (tab.file_handlers.size() > 1) {
    uiItemR(col, op->ptr, "file_handler", eUI_Item_Flag(0), "", ICON_NONE);
  }

  drop_import_file_draw_import_operator(C, col, tab.op);

  uiItemS_ex(col, 0.25f);
  uiItemL_ex(col, drop_import_data.file_summary.c_str(), ICON_NONE, true, false);
  uiItemS_ex(col, 0.25f);
}

static void wm_drop_import_file_fh_items_set(wmOperator *op)
{
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);
  PropertyRNA *enum_items_prop = RNA_struct_find_collection_property_check(
      *op->ptr, "file_handlers", &RNA_EnumPropertyItem);
  RNA_property_collection_clear(op->ptr, enum_items_prop);
  int i = 0;
  for (const auto *fh : drop_import_data.tabs[drop_import_data.active_tab].file_handlers) {
    PointerRNA enum_item_ptr{};
    RNA_property_collection_add(op->ptr, enum_items_prop, &enum_item_ptr);
    EnumPropertyItem *item = static_cast<EnumPropertyItem *>(enum_item_ptr.data);
    *item = {};
    item->identifier = fh->idname;
    item->name = fh->label;
    item->value = i++;
  }
}

static void wm_drop_import_file_tabs_items_set(wmOperator *op)
{
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);
  PropertyRNA *enum_items_prop = RNA_struct_find_collection_property_check(
      *op->ptr, "tabs", &RNA_EnumPropertyItem);
  RNA_property_collection_clear(op->ptr, enum_items_prop);
  int i = 0;
  for (auto &tab : drop_import_data.tabs) {
    PointerRNA enum_item_ptr{};
    RNA_property_collection_add(op->ptr, enum_items_prop, &enum_item_ptr);
    EnumPropertyItem *item = static_cast<EnumPropertyItem *>(enum_item_ptr.data);
    *item = {};
    item->identifier = tab.extensions[0].c_str();
    item->name = tab.label.c_str();
    item->value = i++;
  }
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

  DropImportData *drop_import_data = MEM_new<DropImportData>(__func__);
  op->customdata = drop_import_data;
  drop_import_data->paths = paths;

  for (const std::string &path : paths) {
    const char *extension = BLI_path_extension(path.c_str());
    if (!extension) {
      continue;
    }
    /* Skip already listed extensions. */
    auto itr = std::find_if(
        drop_import_data->tabs.begin(), drop_import_data->tabs.end(), [extension](ImportTab &tab) {
          return tab.extensions.contains(extension);
        });
    if (itr != drop_import_data->tabs.end()) {
      int64_t index = itr->extensions.first_index_of(extension);
      itr->count[index]++;
      continue;
    }
    ImportTab tab{};
    tab.extensions.append(extension);
    tab.count.append(1);
    /* Check file handlers that support current extension. */
    for (blender::bke::FileHandlerType *fh : file_handlers) {
      if (fh->file_extensions.contains(extension)) {
        tab.file_handlers.append_non_duplicates(fh);
      }
    }
    if (tab.file_handlers.is_empty()) {
      CLOG_WARN(&LOG, "%s skipped.", path.c_str());
      continue;
    }
    bool merged = false;
    /** Merge tabs with same file handlers. */
    for (auto &test_tab : drop_import_data->tabs) {
      if (test_tab.file_handlers == tab.file_handlers) {
        test_tab.extensions.extend(tab.extensions);
        test_tab.count.extend(tab.count);
        merged = true;
      }
    }
    if (!merged) {
      drop_import_data->tabs.append(std::move(tab));
    }
  }
  for (auto &tab : drop_import_data->tabs) {
    drop_import_data->count += std::accumulate(tab.count.begin(), tab.count.end(), 0);

    /* Initialize selected file handler operator. */
    tab.op = wm_operator_create(CTX_wm_manager(C),
                                WM_operatortype_find(tab.file_handlers[0]->import_operator, false),
                                nullptr,
                                CTX_wm_reports(C));
    WM_operator_last_properties_init(tab.op);

    /** If a import tab can be handled only by one file hander the tab will display its label. */
    if (tab.file_handlers.size() == 1) {
      tab.label = tab.file_handlers[0]->label;
      continue;
    }
    for (std::string &extension : tab.extensions) {
      if (!tab.label.empty()) {
        tab.label += " ";
      }
      tab.label += extension;
    }
  }
  /* Load runtime enums. */
  wm_drop_import_file_tabs_items_set(op);
  wm_drop_import_file_fh_items_set(op);

  for (const auto &tab : drop_import_data->tabs) {
    for (const int64_t idx : tab.extensions.index_range()) {
      if (!drop_import_data->file_summary.empty()) {
        drop_import_data->file_summary += ", ";
      }
      drop_import_data->file_summary += fmt::format(
          "{} {} files", tab.count[idx], tab.extensions[idx]);
    }
  }

  return WM_operator_props_dialog_popup(
      C, op, 400, fmt::format(TIP_("Import {} files"), drop_import_data->count).c_str());
}

static const EnumPropertyItem *enum_prop_itemf(PointerRNA *ptr, const char *name)
{
  PropertyRNA *enum_items_prop = RNA_struct_find_collection_property_check(
      *ptr, name, &RNA_EnumPropertyItem);
  int enum_items_len = RNA_property_collection_length(ptr, enum_items_prop);

  EnumPropertyItem *item = nullptr;
  int totitem = 0;

  for (int i = 0; i < enum_items_len; i++) {
    PointerRNA enum_item_ptr;
    RNA_property_collection_lookup_int(ptr, enum_items_prop, i, &enum_item_ptr);
    EnumPropertyItem *item_ptr = static_cast<EnumPropertyItem *>(enum_item_ptr.data);
    RNA_enum_item_add(&item, &totitem, item_ptr);
  }
  RNA_enum_item_end(&item, &totitem);
  return item;
}

static const EnumPropertyItem *RNA_file_hadler_itemf(bContext * /*C*/,
                                                     PointerRNA *ptr,
                                                     PropertyRNA * /*prop*/,
                                                     bool *r_free)
{
  const EnumPropertyItem *item = enum_prop_itemf(ptr, "file_handlers");
  *r_free = true;
  return item;
}

static const EnumPropertyItem *RNA_import_tabs_itemf(bContext * /*C*/,
                                                     PointerRNA *ptr,
                                                     PropertyRNA * /*prop*/,
                                                     bool *r_free)
{
  const EnumPropertyItem *item = enum_prop_itemf(ptr, "tabs");
  *r_free = true;
  return item;
}

static bool wm_drop_import_file_check(bContext *C, wmOperator *op)
{
  const int active_tab = RNA_enum_get(op->ptr, "tab");
  DropImportData &drop_import_data = *static_cast<DropImportData *>(op->customdata);
  if (drop_import_data.active_tab != active_tab) {
    drop_import_data.active_tab = active_tab;
    RNA_enum_set(op->ptr, "file_handler", drop_import_data.tabs[active_tab].active_file_handler);
    wm_drop_import_file_fh_items_set(op);
    return true;
  }
  const int active_file_handler = RNA_enum_get(op->ptr, "file_handler");
  if (active_file_handler != drop_import_data.tabs[active_tab].active_file_handler) {
    ImportTab &tab = drop_import_data.tabs[active_tab];
    tab.active_file_handler = active_file_handler;
    WM_operator_free(tab.op);
    /* Set selected file handler operator in the import tab. */
    tab.op = wm_operator_create(
        CTX_wm_manager(C),
        WM_operatortype_find(tab.file_handlers[active_file_handler]->import_operator, false),
        nullptr,
        CTX_wm_reports(C));
    WM_operator_last_properties_init(tab.op);
  }
  return false;
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
  ot->check = wm_drop_import_file_check;

  PropertyRNA *prop;

  prop = RNA_def_string_dir_path(
      ot->srna, "directory", nullptr, FILE_MAX, "Directory", "Directory of the file");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_collection_runtime(ot->srna, "files", &RNA_OperatorFileListElement, "Files", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_collection_runtime(
      ot->srna, "file_handlers", &RNA_EnumPropertyItem, "File Handlers", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_collection_runtime(ot->srna, "tabs", &RNA_EnumPropertyItem, "Import tabs", "");
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna, "file_handler", rna_enum_dummy_NULL_items, 0, "File Handler", "");
  RNA_def_enum_funcs(prop, RNA_file_hadler_itemf);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);

  prop = RNA_def_enum(ot->srna, "tab", rna_enum_dummy_NULL_items, 0, "Import tab", "");
  RNA_def_enum_funcs(prop, RNA_import_tabs_itemf);
  RNA_def_property_flag(prop, PROP_HIDDEN | PROP_SKIP_SAVE);
}

static void drop_import_file_copy(bContext * /*C*/, wmDrag *drag, wmDropBox *drop)
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
