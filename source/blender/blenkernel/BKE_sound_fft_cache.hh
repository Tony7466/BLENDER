/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_array.hh"
#include "BLI_concurrent_map.hh"
#include "BLI_function_ref.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_sound_types.h"

void BKE_sound_fft_cache_new(struct bSound *sound);
void BKE_sound_fft_cache_delete(struct bSound *sound);

namespace blender::bke::sound::fft_cache {

struct FFTParameter {
  int aligned_sample_index;
  std::optional<int> channel;
  int fft_size;
  int window;

  BLI_STRUCT_EQUALITY_OPERATORS_4(FFTParameter, aligned_sample_index, fft_size, window, channel);

  uint64_t hash() const;
};

struct FFTResult {
  Array<float> bins;
};

using CacheMap = ConcurrentMap<FFTParameter, std::shared_ptr<FFTResult>>;

class FFTCache {
 public:
  FFTCache();

  std::shared_ptr<FFTResult> get_or_compute(const FFTParameter &parameter,
                                            const FunctionRef<std::shared_ptr<FFTResult>()> fn);

 private:
  CacheMap map_;
};

struct FFTCacheRuntime {
  std::shared_ptr<FFTCache> cache;
};

}  // namespace blender::bke::sound::fft_cache
