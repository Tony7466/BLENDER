/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "BLI_chunkify_array.hh"
#include "BLI_hash_mm3.h"
#include "BLI_rand.hh"

#include "testing/testing.h"

namespace blender::tests {

TEST(chunkify_array, Test)
{
  Vector<int> data;
  const int amount = 100000;

  RandomNumberGenerator rng(123);

  for ([[maybe_unused]] const int i : IndexRange(amount)) {
    // data.append(234);
    // data.append(i);
    data.append(rng.get_int32());
  }

  data.insert(40000, 12);

  data[50000] = 0;

  ChunkifyArrayParams params;
  params.element_size = sizeof(data[0]);
  params.approximate_elements_per_chunk = 1000;

  Vector<int64_t> offsets_vec;
  OffsetIndices<int64_t> offsets = chunkify_array(
      data.data(), data.as_span().size_in_bytes(), params, offsets_vec);

  for (const int i : offsets.index_range()) {
    const IndexRange chunk_range = offsets[i];
    const Span<std::byte> chunk = data.as_span().cast<std::byte>().slice(chunk_range);
    const uint32_t hash = BLI_hash_mm3(
        reinterpret_cast<const uint8_t *>(chunk.data()), chunk.size(), 436325);
    std::cout << chunk_range.size() << " \tHash: " << hash << "\n";
  }
}

}  // namespace blender::tests
