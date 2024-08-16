/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_function_ref.hh"
#include "BLI_generic_key.hh"
#include "BLI_memory_counter_fwd.hh"

namespace blender::memory_cache {

class CachedValue {
 public:
  virtual ~CachedValue() = default;

  virtual void count_memory(MemoryCounter &memory) const = 0;
};

std::shared_ptr<CachedValue> get_base(const GenericKey &key,
                                      FunctionRef<std::unique_ptr<CachedValue>()> compute_fn);

template<typename T>
inline std::shared_ptr<const T> get(const GenericKey &key,
                                    FunctionRef<std::unique_ptr<T>()> compute_fn)
{
  return std::static_pointer_cast<const T>(get_base(key, compute_fn));
}

void free_to_fit(int64_t capacity);

}  // namespace blender::memory_cache
