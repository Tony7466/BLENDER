/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_offset_indices.hh"
#include "BLI_span.hh"

namespace blender::grouped_indices {

class IndexMask;

bool is_fragmented(const Span<int> group_indices, int total_groups);

int identifiers_to_indices(MutableSpan<int> r_identifiers_to_indices, bool stable = true);

GroupedSpan<int> from_indices(const Span<int> group_indices,
                              MutableSpan<int> r_counts_to_offsets,
                              MutableSpan<int> r_indices,
                              bool fragmented,
                              bool stable = true);

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  const IndexMask &universe,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices,
                                  bool stable = true);

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices,
                                  bool stable = true);

}  // namespace blender::grouped_indices