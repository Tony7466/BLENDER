/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup intern_memutil
 */

#include "MEM_alloc_string_storage.hh"

namespace intern::memutil {

void alloc_string_storage_init()
{
  internal::ensure_storage_container();
}

namespace internal {

AllocStringStorageContainer &ensure_storage_container()
{
  static AllocStringStorageContainer storage;
  return storage;
}

}  // namespace internal

}  // namespace intern::memutil
