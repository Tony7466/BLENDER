/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <memory>

#include "BLI_any.hh"
#include "BLI_function_ref.hh"
#include "BLI_generic_key.hh"
#include "BLI_memory_counter_fwd.hh"
#include "BLI_utildefines.h"

namespace blender::disk_read_cache {

class ReadValue {
 public:
  std::unique_ptr<GenericKey> key;

  virtual ~ReadValue() = default;

  virtual void count_memory(MemoryCounter &memory) const = 0;
};

std::shared_ptr<const ReadValue> read_base(const GenericKey &key,
                                           FunctionRef<std::unique_ptr<ReadValue>()> read_fn);

template<typename T>
inline std::shared_ptr<const T> read(const GenericKey &key,
                                     FunctionRef<std::unique_ptr<T>()> read_fn)
{
  return std::static_pointer_cast<const T>(read_base(key, read_fn));
}

}  // namespace blender::disk_read_cache
