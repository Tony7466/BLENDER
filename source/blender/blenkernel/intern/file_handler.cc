/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstring>

#include "BKE_file_handler.hh"

#include "BLI_path_util.h"
#include "BLI_string.h"

namespace blender::bke {

static Vector<std::unique_ptr<FileHandlerType>> &file_handlers_vector()
{
  static Vector<std::unique_ptr<FileHandlerType>> file_handlers;
  return file_handlers;
}

Span<std::unique_ptr<FileHandlerType>> file_handlers()
{
  return file_handlers_vector();
}

FileHandlerType *file_handler_find(const StringRef name)
{
  auto itr = std::find_if(file_handlers().begin(),
                          file_handlers().end(),
                          [name](const std::unique_ptr<FileHandlerType> &file_handler) {
                            return name == file_handler->idname;
                          });
  if (itr != file_handlers().end()) {
    return itr->get();
  }
  return nullptr;
}

void file_handler_add(std::unique_ptr<FileHandlerType> file_handler)
{
  BLI_assert(file_handler_find(file_handler->idname) == nullptr);

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

  file_handlers_vector().append(std::move(file_handler));
}

void file_handler_remove(FileHandlerType *file_handler)
{
  file_handlers_vector().remove_if(
      [file_handler](const std::unique_ptr<FileHandlerType> &test_file_handler) {
        return test_file_handler.get() == file_handler;
      });
}

static bool ends_with_case(StringRef str, StringRef find)
{
  if (find.size() > str.size() || find.is_empty()) {
    return false;
  }
  return BLI_strcaseeq(str.end() - find.size(), find.data()) == 1;
}

blender::Vector<FileHandlerType *> file_handlers_poll_file_drop(
    const bContext *C, const blender::Span<std::string> paths)
{

  blender::Vector<FileHandlerType *> result;
  for (const std::unique_ptr<FileHandlerType> &file_handler_ptr : file_handlers()) {
    FileHandlerType &file_handler = *file_handler_ptr;
    const auto &file_extensions = file_handler.file_extensions;
    bool support_any_path = false;
    for (const std::string &path : paths) {
      auto test_path_fn = [&path](const std::string &extension) {
        return ends_with_case(path, extension);
      };
      support_any_path = std::any_of(file_extensions.begin(), file_extensions.end(), test_path_fn);
      if (support_any_path) {
        break;
      }
    }
    if (!support_any_path) {
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
    auto test_path_fn = [&path = paths[idx]](const std::string &extension) {
      return ends_with_case(path, extension);
    };
    if (std::find_if(file_extensions.begin(), file_extensions.end(), test_path_fn) !=
        file_extensions.end())
    {
      indices.append(idx);
    }
  }
  return indices;
}

}  // namespace blender::bke
