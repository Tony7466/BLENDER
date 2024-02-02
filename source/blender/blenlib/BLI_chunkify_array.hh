/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_offset_indices.hh"
#include "BLI_vector.hh"

namespace blender {

struct ChunkifyArrayParams {
  int64_t approximate_elements_per_chunk = 10;
  /* Only allows powers of two for now. */
  int64_t element_size = 8;
};

/**
 * Split the data into chunks. The chunk boundaries are based on the content. That means that even
 * if the data is modified or elements are added and removed, the chunks remain relatively stable
 * (i.e. most chunks stay the same). This stability makes the chunks suitable for data
 * deduplication/compression.
 */
OffsetIndices<int64_t> chunkify_array(const void *data,
                                      int64_t bytes_num,
                                      const ChunkifyArrayParams &params,
                                      Vector<int64_t> &r_offsets);

}  // namespace blender
