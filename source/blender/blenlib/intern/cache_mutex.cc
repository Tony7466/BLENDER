/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_cache_mutex.hh"
#include "BLI_task.hh"

#ifdef WITH_TBB
#  include <tbb/task_group.h>
#endif

namespace blender {

CacheMutex::CacheMutex() = default;
CacheMutex::~CacheMutex() = default;

void CacheMutex::ensure(const FunctionRef<void()> compute_cache, const bool is_expensive)
{
  if (cache_valid_.load(std::memory_order_acquire)) {
    /* Fast case when the cache is computed already. */
    return;
  }
  mutex_.lock();
  /* Double checked lock. */
  if (cache_valid_.load(std::memory_order_relaxed)) {
    mutex_.unlock();
    return;
  }
  if (is_computing_in_group_) {
    /* When another thread is already computing the cache, call `wait` on the task group instead.
     * This allows the current thread to steal work from somewhere else instead of being idle until
     * the cache computation is done. */
    mutex_.unlock();
    while (!cache_valid_.load(std::memory_order_acquire)) {
      task_group_->wait();
    }
    return;
  }
  /* If the cache computation is expensive, we want to make sure that other threads waiting for the
   * cache can continue to do some work instead of being idle until the cache is ready. */
  if (is_expensive) {
    if (!task_group_) {
      task_group_ = std::make_unique<tbb::task_group>();
    }
    is_computing_in_group_ = true;
    mutex_.unlock();

    /* Run the actual computation when the mutex is not locked. This is necessary, so that other
     * threads can lock the mutex in the mean-time. Task isolation is not necessary because the
     * mutex is not locked. */
    task_group_->run_and_wait(compute_cache);

    std::scoped_lock lock{mutex_};
    is_computing_in_group_ = false;
  }
  else {
    /* Use task isolation because a mutex is locked and the cache computation might use
     * multi-threading. */
    threading::isolate_task(compute_cache);
    cache_valid_.store(true, std::memory_order_release);
    mutex_.unlock();
  }
}

}  // namespace blender
