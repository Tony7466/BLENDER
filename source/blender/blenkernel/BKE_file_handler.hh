/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "DNA_windowmanager_types.h"

#include "RNA_types.hh"

#define FH_MAX_FILE_EXTENSIONS_STR 512

struct FileHandlerType {
  /** Unique name. */
  char idname[OP_MAX_TYPENAME];
  /** For UI text. */
  char label[OP_MAX_TYPENAME];
  /** Import operator name. */
  char import_operator[OP_MAX_TYPENAME];
  /** Formatted string of file extensions supported by the file handler, each extension should
   * start with a `.` and be separated by `;`. For Example: `".blend;.ble"`. */
  char file_extensions_str[FH_MAX_FILE_EXTENSIONS_STR];

  /** Check if file handler can be used on file drop. */
  bool (*poll_drop)(const struct bContext *C, FileHandlerType *file_handle_type);

  /** List of file extensions supported by the file handler. */
  blender::Vector<std::string> file_extensions;

  /** RNA integration. */
  ExtensionRNA rna_ext;

  /**
   * Return a vector of indices in #paths of file paths supported by the file handler.
   */
  blender::Vector<int64_t> filter_supported_paths(const blender::Span<std::string> paths) const;

  /** Get the operator set as #import_operator, if not exists returns #nullptr. */
  wmOperatorType *get_import_operator(const bool quiet = false) const;

  /**
   * Creates a RNA pointer for the #import_operator and sets on it all supported file paths from
   * #paths.
   */
  PointerRNA import_operator_create_ptr(const blender::Span<std::string> paths) const;
};

/**
 * Adds a new `file_handler` to the `file_handlers` list, also loads all the file extensions from
 * the formatted `FileHandlerType.file_extensions_str` string to `FileHandlerType.file_extensions`
 * list.
 *
 * The new  `file_handler` is expected to have a unique `FileHandlerType.idname`.
 */
void BKE_file_handler_add(std::unique_ptr<FileHandlerType> file_handler);

/** Returns a `file_handler` that have a specific `idname`, otherwise return `nullptr`. */
FileHandlerType *BKE_file_handler_find(const char *idname);

/** Removes and frees a specific `file_handler` from the `file_handlers` list, the `file_handler`
 * pointer will be not longer valid for use. */
void BKE_file_handler_remove(FileHandlerType *file_handler);

/** Return a reference of the #RawVector with all `file_handlers` registered. */
const blender::RawVector<std::unique_ptr<FileHandlerType>> &BKE_file_handlers();

/**
 * Return a vector of file handlers that support any file path in #paths, has a valid
 * #import_operator assigned and the call to #poll_drop returns #true.
 */
blender::Vector<FileHandlerType *> BKE_file_handlers_poll_file_drop(
    const bContext *C, const blender::Span<std::string> paths, const bool quiet = false);
