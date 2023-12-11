/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_path_util.h"
#include "BLI_string.h"

#include "BLT_translation.h"

#include "BKE_file_handler.hh"
#include "BKE_report.h"

#include "CLG_log.h"

#include "WM_api.hh"
#include "WM_types.hh"

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
    const bContext *C, const blender::Span<std::string> paths, const bool quiet)
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
    if (!file_handler.get_import_operator(quiet)) {
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

wmOperatorType *FileHandlerType::get_import_operator(const bool quiet) const
{
  return WM_operatortype_find(import_operator, quiet);
}

static const char *property_type_str(PropertyType prop_type)
{
  switch (prop_type) {
    case PROP_BOOLEAN: {
      return "BooleanProperty";
    }
    case PROP_INT: {
      return "IntProperty";
    }
    case PROP_FLOAT: {
      return "FloatProperty";
    }
    case PROP_STRING: {
      return "StringProperty";
    }
    case PROP_ENUM: {
      return "EnumProperty";
    }
    case PROP_POINTER: {
      return "PointerProperty";
    }
    case PROP_COLLECTION: {
      return "CollectionProperty";
    }
    default:
      BLI_assert_unreachable();
      return nullptr;
  }
}

/** Finds a property in the operator properties with a specific #name and #property_type */
static PropertyRNA *find_property_type(PointerRNA &props,
                                       const char *name,
                                       const PropertyType property_type)
{
  PropertyRNA *prop = RNA_struct_find_property(&props, name);
  if (!prop) {
    return nullptr;
  }

  const PropertyType prop_type = RNA_property_type(prop);
  if (prop_type == property_type) {
    return prop;
  }

  if (prop_type != property_type) {
    std::string message = fmt::format(TIP_("'{} : {}()' expected, got '{} : {}()'."),
                                      name,
                                      property_type_str(property_type),
                                      name,
                                      property_type_str(prop_type));
    CLOG_WARN(&FH_LOG, message.c_str());
  }
  return nullptr;
}

/**
 * Finds a collection property in the operator properties with a specific #name and #struct_type.
 */
static PropertyRNA *find_collection_property(PointerRNA &props,
                                             const char *name,
                                             const StructRNA *struct_type)
{
  PropertyRNA *prop = RNA_struct_find_property(&props, name);
  if (!prop) {
    return nullptr;
  }

  const PropertyType prop_type = RNA_property_type(prop);
  const StructRNA *prop_struct_type = RNA_property_pointer_type(&props, prop);
  if (prop_type == PROP_COLLECTION && prop_struct_type == struct_type) {
    return prop;
  }

  if (prop_type != PROP_COLLECTION) {
    std::string message = fmt::format(TIP_("'{} : {}(type = {})' expected, got '{} : {}()'."),
                                      name,
                                      property_type_str(PROP_COLLECTION),
                                      RNA_struct_identifier(struct_type),
                                      name,
                                      property_type_str(prop_type));
    CLOG_WARN(&FH_LOG, message.c_str());
  }

  if (prop_struct_type != struct_type) {
    std::string message = fmt::format(
        TIP_("'{} : {}(type = {})' expected, got '{} : {}(type = {})'."),
        name,
        property_type_str(PROP_COLLECTION),
        RNA_struct_identifier(struct_type),
        name,
        property_type_str(PROP_COLLECTION),
        RNA_struct_identifier(prop_struct_type));
    CLOG_WARN(&FH_LOG, message.c_str());
  }

  return nullptr;
}

PointerRNA FileHandlerType::import_operator_create_ptr(
    const blender::Span<std::string> paths) const
{
  BLI_assert(get_import_operator());
  PointerRNA props;
  wmOperatorType *op = get_import_operator();
  WM_operator_properties_create_ptr(&props, op);

  const auto suported_paths = filter_supported_paths(paths);

  PropertyRNA *filepath_prop = find_property_type(props, "filepath", PROP_STRING);
  if (filepath_prop) {
    RNA_property_string_set(&props, filepath_prop, paths[suported_paths[0]].c_str());
  }

  PropertyRNA *directory_prop = find_property_type(props, "directory", PROP_STRING);
  if (directory_prop) {
    char dir[FILE_MAX];
    BLI_path_split_dir_part(paths[0].c_str(), dir, sizeof(dir));
    RNA_property_string_set(&props, directory_prop, dir);
  }

  PropertyRNA *files_prop = find_collection_property(props, "files", &RNA_OperatorFileListElement);
  if (files_prop) {
    RNA_property_collection_clear(&props, files_prop);
    for (const auto &index : suported_paths) {
      char file[FILE_MAX];
      BLI_path_split_file_part(paths[index].c_str(), file, sizeof(file));

      PointerRNA item_ptr{};
      RNA_property_collection_add(&props, files_prop, &item_ptr);
      RNA_string_set(&item_ptr, "name", file);
    }
  }

  if (!filepath_prop && !directory_prop && !files_prop) {
    std::string message = fmt::format(
        TIP_("{}: import operator '{}' requires file path properties:\n"
             "- For single files: 'filepath : StringProperty()'\n"
             "- For multiple files: 'directory : StringProperty()' and"
             "'files : CollectionProperty(type = OperatorFileListElement)'"),
        idname,
        op->idname);
    CLOG_WARN(&FH_LOG, message.c_str());
  }

  if (directory_prop && !files_prop) {
    std::string message = fmt::format(
        TIP_("{}: import operator '{}' have 'directory : StringProperty()' but misses 'files : "
             "CollectionProperty(type = OperatorFileListElement)'."),
        idname,
        op->idname);
    CLOG_WARN(&FH_LOG, message.c_str());
  }

  if (!directory_prop && files_prop) {
    std::string message = fmt::format(
        TIP_("{}: Import operator '{}' have 'files : CollectionProperty(type = "
             "OperatorFileListElement)' but misses 'directory : StringProperty()'."),
        idname,
        op->idname);
    CLOG_WARN(&FH_LOG, message.c_str());
  }

  return props;
}

#undef LOG
