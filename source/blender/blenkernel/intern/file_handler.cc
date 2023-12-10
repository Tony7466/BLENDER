/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_path_util.h"
#include "BLI_string.h"

#include "WM_api.hh"
#include "WM_types.hh"

#include "BKE_file_handler.hh"

#include "RNA_access.hh"

static blender::RawVector<std::unique_ptr<FileHandlerType>> &file_handlers()
{
  static blender::RawVector<std::unique_ptr<FileHandlerType>> file_handlers;
  return file_handlers;
}

const blender::RawVector<std::unique_ptr<FileHandlerType>> &BKE_file_handlers()
{
  return file_handlers();
}

FileHandlerType *BKE_file_handler_find(const char *name)
{
  auto itr = std::find_if(file_handlers().begin(),
                          file_handlers().end(),
                          [name](const std::unique_ptr<FileHandlerType> &file_handler) {
                            return STREQ(name, file_handler->idname);
                          });
  if (itr != file_handlers().end()) {
    return itr->get();
  }
  return nullptr;
}

void BKE_file_handler_add(std::unique_ptr<FileHandlerType> file_handler)
{
  BLI_assert(BKE_file_handler_find(file_handler->idname) != nullptr);

  /** Load all extensions from the string list into the list. */
  const char char_separator = ';';
  const char *char_begin = file_handler->file_extensions_str;
  const char *char_end = BLI_strchr_or_end(char_begin, char_separator);
  while (char_begin[0]) {
    if (char_end - char_begin > 1) {
      std::string file_extension(char_begin, char_end - char_begin);
      file_handler->file_extensions.append(file_extension);
    }
    char_begin = char_end[0] ? char_end + 1 : char_end;
    char_end = BLI_strchr_or_end(char_begin, char_separator);
  }

  file_handlers().append(std::move(file_handler));
}

void BKE_file_handler_remove(FileHandlerType *file_handler)
{
  file_handlers().remove_if(
      [file_handler](const std::unique_ptr<FileHandlerType> &test_file_handler) {
        return test_file_handler.get() == file_handler;
      });
}

blender::Vector<FileHandlerType *> BKE_file_handlers_poll_file_drop(
    const bContext *C, const blender::Span<std::string> paths)
{

  blender::Vector<std::string> path_extensions;
  for (const std::string &path : paths) {
    const char *extension = BLI_path_extension(path.c_str());
    if (!extension) {
      continue;
    }
    path_extensions.append_non_duplicates(extension);
  }

  blender::Vector<FileHandlerType *> result;

  for (const std::unique_ptr<FileHandlerType> &file_handler_ptr : file_handlers()) {
    FileHandlerType &file_handler = *file_handler_ptr;
    const auto &file_extensions = file_handler.file_extensions;

    const bool support_any_extension = std::any_of(
        file_extensions.begin(),
        file_extensions.end(),
        [&path_extensions](const std::string &test_extension) {
          return path_extensions.contains(test_extension);
        });

    if (!support_any_extension) {
      continue;
    }

    if (!(file_handler.poll_drop && file_handler.poll_drop(C, &file_handler))) {
      continue;
    }

    /* File handles can be registered without a import operator, or the import operator can be
     * removed and the file handler can still be registered, filter too file handles with valid
     * import operators. */
    if (!file_handler.get_import_operator()) {
      continue;
    }

    result.append(&file_handler);
  }
  return result;
}

blender::Vector<int64_t> FileHandlerType::filter_supported_paths(
    const blender::Span<std::string> paths) const
{
  blender::Vector<int64_t> indices;

  for (const int idx : paths.index_range()) {
    const char *extension = BLI_path_extension(paths[idx].c_str());
    if (!extension) {
      continue;
    }
    if (file_extensions.contains(extension)) {
      indices.append(idx);
    }
  }
  return indices;
}

wmOperatorType *FileHandlerType::get_import_operator() const
{
  return WM_operatortype_find(import_operator, false);
}

PointerRNA FileHandlerType::import_operator_file_properties_create_ptr(
    const blender::Span<std::string> paths) const
{
  BLI_assert(get_import_operator());

  PointerRNA props;
  WM_operator_properties_create_ptr(&props, get_import_operator());

  const auto suported_paths = filter_supported_paths(paths);

  if (PropertyRNA *prop = RNA_struct_find_property(&props, "filepath")) {
    RNA_string_set(&props, "filepath", paths[suported_paths[0]].c_str());
  }

  if (PropertyRNA *prop = RNA_struct_find_property(&props, "directory")) {

    char dir[FILE_MAX];
    BLI_path_split_dir_part(paths[0].c_str(), dir, sizeof(dir));
    RNA_string_set(&props, "directory", dir);

    RNA_collection_clear(&props, "files");
    for (const auto &index : suported_paths) {
      char file[FILE_MAX];
      BLI_path_split_file_part(paths[index].c_str(), file, sizeof(file));

      PointerRNA itemptr{};
      RNA_collection_add(&props, "files", &itemptr);
      RNA_string_set(&itemptr, "name", file);
    }
  }
  return props;
}
