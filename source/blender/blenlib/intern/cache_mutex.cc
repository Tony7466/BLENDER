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

void CacheMutex::ensure(const FunctionRef<void()> compute_cache, const bool is_expensive)
{
  if (cache_valid_.load(std::memory_order_acquire)) {
    return;
  }

  /* If the computation is expensive, a more complex evaluation strategy is used which allows other
   * threads waiting for this cache to join its computation. This avoids blocking threads
   * unnecessarily. */
  if (is_expensive) {
    {
      std::scoped_lock lock{mutex_};
      /* Double checked lock. */
      if (cache_valid_.load(std::memory_order_relaxed)) {
        mutex_.unlock();
        return;
      }
      /* Ensure the arena exists. */
      if (!arena_) {
        arena_ = std::make_unique<CacheMutexArena>();
      }
    }

    /* The code below is performing a potentially expensive operation. Allow any task schedulers to
     * move work to other threads. This hint can't be sent from within the isolated region. */
    lazy_threading::send_hint();
    lazy_threading::ReceiverIsolation isolation;

    /* Use a task arena to make sure that the current thread does not steal completely unrelated
     * tasks from other threads. While it should be save, it results in less predictable timings
     * and does not guarantee better overall performance.
     *
     * Task isolation using #BLI_task_isolate is not necessary, because the mutex is not locked
     * right now.
     */
    arena_->arena.execute([&]() {
      /* Check if the current thread is the first thread starting the computation, or another
       * thread. */
      bool expected = false;
      const bool is_primary = arena_->task_pushed.compare_exchange_strong(expected, true);

      if (is_primary) {
        /* The first thread enqueues the work to compute the cache. */
        arena_->group.run_and_wait(compute_cache);
        cache_valid_.store(true, std::memory_order_release);
      }
      else {
        /* Threads that come later have to wait until the cache is computed. */
        while (!cache_valid_.load(std::memory_order_acquire)) {
          /* While waiting, they can also join the ongoing cache computation if it is multi
           * threaded. */
          arena_->group.wait();
        }
      }
    });
  }
  else {
    std::scoped_lock lock{mutex_};
    /* Double checked lock. */
    if (cache_valid_.load(std::memory_order_relaxed)) {
      return;
    }
    /* Use task isolation because a mutex is locked and the cache computation might use
     * multi-threading. */
    threading::isolate_task(compute_cache);

    cache_valid_.store(true, std::memory_order_release);
  }
}

void CacheMutex::tag_dirty()
{
  cache_valid_.store(false);
  if (arena_) {
    arena_->task_pushed.store(false);
  }
}

}  // namespace blender
