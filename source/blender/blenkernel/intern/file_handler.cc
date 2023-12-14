/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BLT_translation.h"

#include "BKE_file_handler.hh"
#include "BKE_report.h"

#include "CLG_log.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "fmt/format.h"

static CLG_LogRef FH_LOG = {"bke.file_handler"};
#define LOG (&FH_LOG)

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
  BLI_assert(BKE_file_handler_find(file_handler->idname) == nullptr);

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
