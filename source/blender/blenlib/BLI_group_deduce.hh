/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_offset_indices.hh"
#include "BLI_span.hh"

/** \file
 * \ingroup bli
 *
 * Implementations of an algorithm to deduce all the groups.
 * Group is a set of the same values. The values can be randomly distributed in taked span.
 * Deduced group is a set of an indices to all the values in taked span.
 *
 * This file contains base implementation of algorithm and helpful tools to deal with the groups.
 * Index or an identifier of groups both is a ways to define the group value key.
 * Index is the main one, but identifier can be used too by utils.
 *
 * All the implementations are multithreaded. Result can be not stable to avoid redundant actions.
 *
 * The main statistical property of groups is fragmentation. This is a simplified version of the
 * average group size grade for algorithm.
 */

namespace blender::index_mask {
class IndexMask;
}

namespace blender::grouped_indices {

/**
 * Deduce the fragmentation threshold for a given span of indices. A constant cost is assumed.
 */
bool is_fragmented(const Span<int> group_indices, int total_groups);

/**
 * Turn the span of random values to the span of the group value indices.
 * \return Total of the groups.
 */
int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices, bool stable = true);

/**
 * \param r_counts_to_offsets: The result a groups sizes and offsets going to be computed and
 * filled into. All the source \a group_indices indices must be bounded in to this span. \param
 * r_indices: Have to be the same size as \a group_indices. Result is indices to \a group_indices
 * span. \param fragmented: Usually *grouped_indices::is_fragmented* should used be only. \param
 * stable: Stability of the order of indices in groups. \return For any value from \a group_indices
 * can be sampled a span from. This span have to contains index of this value.
 */
GroupedSpan<int> from_indices(const Span<int> group_indices,
                              MutableSpan<int> r_counts_to_offsets,
                              MutableSpan<int> r_indices,
                              bool fragmented,
                              bool stable = true);

/**
 * This is similar to #from_indices but automatically convert \a groups_ids to the group indices by
 * using \a universe. \param r_offsets and r_indices: Will be automatically reinitialized and
 * filled in as in #from_indices.
 */
GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  const index_mask::IndexMask &universe,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices,
                                  bool stable = true);

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices,
                                  bool stable = true);

}  // namespace blender::grouped_indices