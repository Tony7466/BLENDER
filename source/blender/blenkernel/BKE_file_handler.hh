/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_vector.hh"

#include "DNA_windowmanager_types.h"

#include "RNA_types.hh"

struct bFileExtension {
  char extension[64];
};

struct FileHandlerType {

  char idname[OP_MAX_TYPENAME]; /* Unique name. */

  char label[OP_MAX_TYPENAME]; /* For UI text. */

  char import_operator[OP_MAX_TYPENAME]; /* Import operator name. */

  /* String list of file extensions supported by the file handler. */
  char file_extensions_str[OP_MAX_TYPENAME];

  /* Check if file handler can be used. */
  bool (*poll_drop)(const struct bContext *C, FileHandlerType *file_handle_type);

  /* List of file extensions supported by the file handler. */
  blender::Vector<bFileExtension> file_extensions;

  /* RNA integration */
  ExtensionRNA rna_ext;
};

void BKE_file_handlers_init();

void BKE_file_handlers_free();

blender::Span<FileHandlerType *> BKE_file_handlers();

void BKE_file_handler_add(FileHandlerType *file_handler);

void BKE_file_handler_remove(FileHandlerType *file_handler);
