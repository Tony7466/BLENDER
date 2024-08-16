/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#include <functional>
#include <mutex>

#include "BLI_generic_key.hh"
#include "BLI_memory_counter.hh"
#include "BLI_set.hh"

namespace blender::memory_cache {

using memory_counter::MemoryBySharedData;

class MemoryCache {
 private:
  struct Value {
    std::unique_ptr<GenericKey> key;
    // OwnedMemory memory;
    int64_t last_use = 0;
    std::function<void()> free_fn;
  };

  std::mutex mutex_;
  int64_t capacity_in_bytes_ = 0;
  int64_t current_bytes_ = 0;
  MemoryBySharedData memory_by_shared_data_;
  int64_t logical_time_ = 0;

  Map<std::reference_wrapper<const GenericKey>, Value> cache_;

 public:
  MemoryCache(const int64_t capacity_in_bytes = 1024 * 1024 * 1024);

  void add(std::unique_ptr<GenericKey> key,
           FunctionRef<void(MemoryCounter &memory)> count_memory_fn,
           std::function<void()> free_fn);

  void remove(const GenericKey &key);

  void touch(const GenericKey &key);

 private:
  void free_if_necessary();

  void add_memory_of(const memory_counter::SharedDataInfo &memory);
  void subtract_memory_of(const memory_counter::SharedDataInfo &memory);
};

MemoryCache &global_cache();

}  // namespace blender::memory_cache
