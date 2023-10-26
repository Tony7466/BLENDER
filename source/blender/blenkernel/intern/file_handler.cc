/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_file_handler.hh"

static blender::Vector<FileHandlerType *> *file_handlers = nullptr;

void BKE_file_handlers_init()
{
  file_handlers = MEM_new<blender::Vector<FileHandlerType *>>(__func__);
}

void BKE_file_handlers_free()
{
  for (auto *file_handler : *file_handlers) {
    if (file_handler->rna_ext.free) {
      file_handler->rna_ext.free(file_handler->rna_ext.data);
    }
    MEM_delete(file_handler);
  }
  MEM_delete(file_handlers);
}

blender::Span<FileHandlerType *> BKE_file_handlers()
{
  return file_handlers->as_span();
}

void BKE_file_handler_add(FileHandlerType *file_handler)
{
  file_handlers->append(file_handler);
}

void BKE_file_handler_remove(FileHandlerType *file_handler)
{
  file_handlers->remove(file_handlers->first_index_of(file_handler));
  MEM_delete(file_handler);
}
