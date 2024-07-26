/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup intern_mem
 */

#pragma once

namespace mem_guarded::internal {

enum class AllocationType {
  /** Allocation is handled through 'C type' alloc/free calls. */
  ALLOC_FREE,
  /** Allocation is handled through 'C++ type' new/delete calls. */
  NEW_DELETE,
};

/** Internal implementation of #MEM_freeN, exposed because #MEM_delete needs access to it. */
extern void (*mem_freeN_ex)(void *vmemh, AllocationType allocation_type);

/** Internal implementation of #MEM_mallocN_aligned, exposed because #MEM_new needs access to it.
 */
extern void *(*mem_mallocN_aligned_ex)(size_t len,
                                       size_t alignment,
                                       const char *str,
                                       AllocationType allocation_type);

/**
 * Store a std::any into the #Local thread-local data also used for memory usage tracking.
 *
 * Typically, this `any` should contain a `shared_ptr` to the actual data, to ensure that the data
 * itself is not duplicated, and that the the memory usage system does become an owner of it.
 *
 * That way, the memleak data does not get destructed before the memory usage data is, which
 * happens after the execution and destruction of the memleak detector.
 */
void add_memleak_data(std::any &data);

}  // namespace mem_guarded::internal
