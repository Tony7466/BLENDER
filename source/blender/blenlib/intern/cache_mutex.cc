/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_cache_mutex.hh"
#include "BLI_lazy_threading.hh"
#include "BLI_task.hh"

#include <iostream>

#ifdef WITH_TBB
#  include <tbb/task_arena.h>
#  include <tbb/task_group.h>
#endif

namespace blender {

struct CacheMutexArena {
  std::atomic<bool> task_pushed = false;
  tbb::task_arena arena;
  tbb::task_group group;
};

CacheMutex::CacheMutex() = default;
CacheMutex::~CacheMutex() = default;

void CacheMutex::ensure(const FunctionRef<void()> compute_cache)
{
  if (cache_valid_.load(std::memory_order_acquire)) {
    return;
  }
  mutex_.lock();
  /* Double checked lock. */
  if (cache_valid_.load(std::memory_order_relaxed)) {
    mutex_.unlock();
    return;
  }

  if (!arena_) {
    arena_ = std::make_unique<CacheMutexArena>();
  }

  lazy_threading::send_hint();
  lazy_threading::ReceiverIsolation isolation;
  arena_->arena.execute([&]() {
    mutex_.unlock();
    bool expected = false;
    const bool is_primary = arena_->task_pushed.compare_exchange_strong(expected, true);
    if (is_primary) {
      arena_->group.run_and_wait(compute_cache);
      cache_valid_.store(true, std::memory_order_release);
    }
    else {
      while (!cache_valid_.load(std::memory_order_acquire)) {
        arena_->group.wait();
      }
    }
  });
}

void CacheMutex::tag_dirty()
{
  cache_valid_.store(false);
  if (arena_) {
    arena_->task_pushed.store(false);
  }
}

}  // namespace blender
