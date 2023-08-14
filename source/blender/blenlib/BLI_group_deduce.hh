/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_offset_indices.hh"
#include "BLI_span.hh"

namespace blender::grouped_indices {

bool is_fragmented(const Span<int> group_indices, const int total_groups);

GroupedSpan<int> from_indices(const Span<int> group_indices,
                              const bool fragmented,
                              MutableSpan<int> r_counts_to_offsets,
                              MutableSpan<int> r_indices);

GroupedSpan<int> from_identifiers(const Span<int> groups_ids,
                                  Array<int> &r_offsets,
                                  Array<int> &r_indices);

}  // namespace blender::grouped_indices