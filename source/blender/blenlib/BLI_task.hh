/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef WITH_TBB
/* Quiet top level deprecation message, unrelated to API usage here. */
#  if defined(WIN32) && !defined(NOMINMAX)
/* TBB includes Windows.h which will define min/max macros causing issues
 * when we try to use std::min and std::max later on. */
#    define NOMINMAX
#    define TBB_MIN_MAX_CLEANUP
#  endif
#  include <tbb/blocked_range.h>
#  include <tbb/parallel_for.h>
#  include <tbb/parallel_for_each.h>
#  include <tbb/parallel_invoke.h>
#  include <tbb/parallel_reduce.h>
#  include <tbb/task_arena.h>
#  ifdef WIN32
/* We cannot keep this defined, since other parts of the code deal with this on their own, leading
 * to multiple define warnings unless we un-define this, however we can only undefine this if we
 * were the ones that made the definition earlier. */
#    ifdef TBB_MIN_MAX_CLEANUP
#      undef NOMINMAX
#    endif
#  endif
#endif

#include "BLI_function_ref.hh"
#include "BLI_index_range.hh"
#include "BLI_lazy_threading.hh"
#include "BLI_span.hh"
#include "BLI_utildefines.h"

namespace blender {

/**
 * Wrapper type around an integer to differentiate it from other parameters in a function call.
 */
struct GrainSize {
  int64_t value;

  explicit constexpr GrainSize(const int64_t grain_size) : value(grain_size) {}
};

}  // namespace blender

namespace blender::threading {

template<typename Range, typename Function>
inline void parallel_for_each(Range &&range, const Function &function)
{
#ifdef WITH_TBB
  tbb::parallel_for_each(range, function);
#else
  for (auto &&value : range) {
    function(value);
  }
#endif
}

/**
 * Specifies how large the individual tasks are relative to each other. It's common that all tasks
 * have a very similar size in which case one can just ignore this. However, sometimes tasks have
 * very different sizes and it makes sense for the scheduler to group fewer big tasks and many
 * small tasks together.
 */
class TaskSizeHints {
 public:
  enum class Type {
    /** All tasks have the same size. */
    Static,
    /** All tasks can have different sizes and one has to look up the sizes one by one. */
    IndividualLookup,
    /**
     * All tasks can have different sizes but one can efficiently determine the size of a
     * consecutive range of tasks.
     */
    AccumulatedLookup,
  };

  Type type;
  std::optional<int64_t> size;
  std::optional<int64_t> full_size;

 protected:
  TaskSizeHints(const Type type) : type(type) {}

 public:
  TaskSizeHints(const int64_t size) : type(Type::Static), size(size) {}

  bool use_single_thread(const IndexRange range, const int64_t threshold) const
  {
    switch (this->type) {
      case Type::Static: {
        return *this->size * range.size() <= threshold;
      }
      case Type::IndividualLookup: {
        if (full_size.has_value()) {
          if (*full_size <= threshold) {
            return true;
          }
        }
        return false;
      }
      case Type::AccumulatedLookup: {
        return this->lookup_accumulated_size(range) <= threshold;
      }
    }
    BLI_assert_unreachable();
    return true;
  }

  /**
   * Get the individual size of all tasks in the range. This must only be used if the type is
   * #Type::IndividualLookup.
   */
  virtual void lookup_individual_sizes(IndexRange /*range*/, MutableSpan<int64_t> r_sizes) const
  {
    BLI_assert_unreachable();
    r_sizes.fill(1);
  }

  /**
   * Get the accumulated size of a range of tasks. This must only be used if the type is
   * #Type::AccumulatedLookup.
   */
  virtual int64_t lookup_accumulated_size(IndexRange range) const
  {
    BLI_assert_unreachable();
    return range.size();
  }
};

namespace detail {

template<typename Fn> class TaskSizeHints_IndividualLookup : public TaskSizeHints {
 private:
  Fn fn_;

 public:
  TaskSizeHints_IndividualLookup(Fn fn, const std::optional<int64_t> full_size)
      : TaskSizeHints(Type::IndividualLookup), fn_(std::move(fn))
  {
    this->full_size = full_size;
  }

  void lookup_individual_sizes(const IndexRange range, MutableSpan<int64_t> r_sizes) const override
  {
    fn_(range, r_sizes);
  }
};

template<typename Fn> class TaskSizeHints_AccumulatedLookup : public TaskSizeHints {
 private:
  Fn fn_;

 public:
  TaskSizeHints_AccumulatedLookup(Fn fn)
      : TaskSizeHints(Type::AccumulatedLookup), fn_(std::move(fn))
  {
  }

  int64_t lookup_accumulated_size(const IndexRange range) const override
  {
    return fn_(range);
  }
};

}  // namespace detail

/**
 * Specify how large the task at each index is with a callback. This is especially useful if the
 * size of each individual task can be very different. Specifying the size allows the scheduler to
 * distribute the work across threads more equally.
 *
 * \param fn: A function that returns the size for a single task: `(int64_t index) -> int64_t`.
 * \param full_size: The (approximate) accumulated size of all tasks. This is optional and should
 *   only be passed in if it is trivially accessible already.
 */
template<typename Fn>
inline auto individual_task_sizes(Fn &&fn, const std::optional<int64_t> full_size = std::nullopt)
{
  auto array_fn = [fn = std::forward<Fn>(fn)](const IndexRange range,
                                              MutableSpan<int64_t> r_sizes) {
    for (const int64_t i : range.index_range()) {
      r_sizes[i] = fn(range[i]);
    }
  };
  return detail::TaskSizeHints_IndividualLookup<decltype(array_fn)>(std::move(array_fn),
                                                                    full_size);
}

/**
 * Very similar to #individual_task_sizes, but should be used if one can very efficiently compute
 * the accumulated task size (in O(1) time). This is often the case when e.g. working with
 * #OffsetIndices.
 *
 * \param fn: A function that returns the accumulated size for a range of tasks:
 * `(IndexRange indices) -> int64_t`.
 */
template<typename Fn> inline auto accumulated_task_sizes(Fn &&fn)
{
  return detail::TaskSizeHints_AccumulatedLookup<decltype(fn)>(std::forward<Fn>(fn));
}

namespace detail {
void parallel_for_impl(IndexRange range,
                       int64_t grain_size,
                       FunctionRef<void(IndexRange)> function,
                       const TaskSizeHints &size_hints);
void memory_bandwidth_bound_task_impl(FunctionRef<void()> function);
}  // namespace detail

/**
 * Executes the given function for sub-ranges of the given range, potentialy in parallel.
 * This is the main primitive for parallelizing code.
 *
 * \param range: The indices that should be iterated over in parallel.
 * \param grain_size: The approximate amount of work that should be scheduled at once.
 *   For example of the range is [0 - 1000] and the grain size is 200, then the function will be
 *   called 5 times with [0 - 200], [201 - 400], ... (approximately). The `size_hints` parameter
 *   can be used to adjust how the work is split up if the tasks have different sizes.
 * \param function: A callback that actually does the work in parallel. It should have one
 *   #IndexRange parameter.
 * \param size_hints: Can be used to specify the size of the tasks *relative to* each other and the
 *   grain size. If all tasks have approximately the same size, this can be ignored. Otherwise, one
 *   can use `threading::individual_task_sizes(...)` or `threading::accumulated_task_sizes(...)`.
 *   If the grain size is e.g. 200 and each task has the size 100, then only two tasks will be
 *   scheduled at once.
 */
template<typename Function>
inline void parallel_for(const IndexRange range,
                         const int64_t grain_size,
                         const Function &function,
                         const TaskSizeHints &size_hints = TaskSizeHints(1))
{
  if (range.is_empty()) {
    return;
  }
  /* Invoking tbb for small workloads has a large overhead. */
  if (size_hints.use_single_thread(range, grain_size)) {
    function(range);
    return;
  }
  detail::parallel_for_impl(range, grain_size, function, size_hints);
}

/**
 * Move the sub-range boundaries down to the next aligned index. The "global" begin and end
 * remain fixed though.
 */
inline IndexRange align_sub_range(const IndexRange unaligned_range,
                                  const int64_t alignment,
                                  const IndexRange global_range)
{
  const int64_t global_begin = global_range.start();
  const int64_t global_end = global_range.one_after_last();
  const int64_t alignment_mask = ~(alignment - 1);

  const int64_t unaligned_begin = unaligned_range.start();
  const int64_t unaligned_end = unaligned_range.one_after_last();
  const int64_t aligned_begin = std::max(global_begin, unaligned_begin & alignment_mask);
  const int64_t aligned_end = unaligned_end == global_end ?
                                  unaligned_end :
                                  std::max(global_begin, unaligned_end & alignment_mask);
  const IndexRange aligned_range = IndexRange::from_begin_end(aligned_begin, aligned_end);
  return aligned_range;
}

/**
 * Same as #parallel_for but tries to make the sub-range sizes multiples of the given alignment.
 * This can improve performance when the range is processed using vectorized and/or unrolled loops,
 * because the fallback loop that processes remaining values is used less often. A disadvantage of
 * using this instead of #parallel_for is that the size differences between sub-ranges can be
 * larger, which means that work is distributed less evenly.
 */
template<typename Function>
inline void parallel_for_aligned(const IndexRange range,
                                 const int64_t grain_size,
                                 const int64_t alignment,
                                 const Function &function)
{
  parallel_for(range, grain_size, [&](const IndexRange unaligned_range) {
    const IndexRange aligned_range = align_sub_range(unaligned_range, alignment, range);
    function(aligned_range);
  });
}

template<typename Value, typename Function, typename Reduction>
inline Value parallel_reduce(IndexRange range,
                             int64_t grain_size,
                             const Value &identity,
                             const Function &function,
                             const Reduction &reduction)
{
#ifdef WITH_TBB
  if (range.size() >= grain_size) {
    lazy_threading::send_hint();
    return tbb::parallel_reduce(
        tbb::blocked_range<int64_t>(range.first(), range.one_after_last(), grain_size),
        identity,
        [&](const tbb::blocked_range<int64_t> &subrange, const Value &ident) {
          return function(IndexRange(subrange.begin(), subrange.size()), ident);
        },
        reduction);
  }
#else
  UNUSED_VARS(grain_size, reduction);
#endif
  return function(range, identity);
}

template<typename Value, typename Function, typename Reduction>
inline Value parallel_reduce_aligned(const IndexRange range,
                                     const int64_t grain_size,
                                     const int64_t alignment,
                                     const Value &identity,
                                     const Function &function,
                                     const Reduction &reduction)
{
  parallel_reduce(
      range,
      grain_size,
      identity,
      [&](const IndexRange unaligned_range, const Value &ident) {
        const IndexRange aligned_range = align_sub_range(unaligned_range, alignment, range);
        function(aligned_range, ident);
      },
      reduction);
}

/**
 * Execute all of the provided functions. The functions might be executed in parallel or in serial
 * or some combination of both.
 */
template<typename... Functions> inline void parallel_invoke(Functions &&...functions)
{
#ifdef WITH_TBB
  tbb::parallel_invoke(std::forward<Functions>(functions)...);
#else
  (functions(), ...);
#endif
}

/**
 * Same #parallel_invoke, but allows disabling threading dynamically. This is useful because when
 * the individual functions do very little work, there is a lot of overhead from starting parallel
 * tasks.
 */
template<typename... Functions>
inline void parallel_invoke(const bool use_threading, Functions &&...functions)
{
  if (use_threading) {
    lazy_threading::send_hint();
    parallel_invoke(std::forward<Functions>(functions)...);
  }
  else {
    (functions(), ...);
  }
}

/** See #BLI_task_isolate for a description of what isolating a task means. */
template<typename Function> inline void isolate_task(const Function &function)
{
#ifdef WITH_TBB
  lazy_threading::ReceiverIsolation isolation;
  tbb::this_task_arena::isolate(function);
#else
  function();
#endif
}

/**
 * Should surround parallel code that is highly bandwidth intensive, e.g. it just fills a buffer
 * with no or just few additional operations. If the buffers are large, it's beneficial to limit
 * the number of threads doing the work because that just creates more overhead on the hardware
 * level and doesn't provide a notable performance benefit beyond a certain point.
 */
template<typename Function>
inline void memory_bandwidth_bound_task(const int64_t approximate_bytes_touched,
                                        const Function &function)
{
  /* Don't limit threading when all touched memory can stay in the CPU cache, because there a much
   * higher memory bandwidth is available compared to accessing RAM. This value is supposed to be
   * on the order of the L3 cache size. Accessing that value is not quite straight forward and even
   * if it was, it's not clear if using the exact cache size would be beneficial because there is
   * often more stuff going on on the CPU at the same time. */
  if (approximate_bytes_touched <= 8 * 1024 * 1024) {
    function();
    return;
  }
  detail::memory_bandwidth_bound_task_impl(function);
}

}  // namespace blender::threading
