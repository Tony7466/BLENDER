/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <mutex>

#include "BLI_array.hh"
#include "BLI_function_ref.hh"
#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_sound_types.h"

void BKE_sound_fft_cache_new(struct bSound *sound, int total_samples);
void BKE_sound_fft_cache_delete(struct bSound *sound);

namespace blender::bke::sound::fft_cache {

struct FFTParameter {
  int aligned_sample_index;
  int fft_size;
  int window;
  std::optional<int> channel;

  BLI_STRUCT_EQUALITY_OPERATORS_4(FFTParameter, aligned_sample_index, fft_size, window, channel);

  uint64_t hash() const;
};

struct FFTResult {
  Array<float> bins;
};

class FFTCache {
 public:
  FFTCache(int total_samples);

  FFTResult *try_get_or_compute(const FFTParameter &parameter,
                                const FunctionRef<std::unique_ptr<FFTResult>()> fn);

  std::mutex mutex_;

 private:
  Map<FFTParameter, std::unique_ptr<FFTResult>> map_;
  const int total_samples_;
};

struct FFTCacheRuntime {
  std::shared_ptr<FFTCache> cache;
};

}  // namespace blender::bke::sound::fft_cache
