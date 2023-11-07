/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "DNA_windowmanager_types.h"

#include "RNA_types.hh"

struct FileHandlerType {

  char idname[OP_MAX_TYPENAME]; /* Unique name. */

  char label[OP_MAX_TYPENAME]; /* For UI text. */

  char import_operator[OP_MAX_TYPENAME]; /* Import operator name. */

  /* String list of file extensions supported by the file handler. */
  char file_extensions_str[OP_MAX_TYPENAME];

  /* Check if file handler can be used on file drop. */
  bool (*poll_drop)(const struct bContext *C, FileHandlerType *file_handle_type);

  /* List of file extensions supported by the file handler. */
  blender::Vector<std::string> file_extensions;

  /* RNA integration */
  ExtensionRNA rna_ext;
};

void BKE_file_handler_add(std::unique_ptr<FileHandlerType> file_handler);

FileHandlerType *BKE_file_handler_find(const char *idname);

void BKE_file_handler_remove(FileHandlerType *file_handler);

const blender::Vector<FileHandlerType *> BKE_file_handlers();
