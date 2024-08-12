/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <shared_mutex>
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
  std::optional<int> channel;
  int fft_size;
  int window;

  BLI_STRUCT_EQUALITY_OPERATORS_4(FFTParameter, aligned_sample_index, fft_size, window, channel);

  friend bool operator<(const FFTParameter &lhs, const FFTParameter &rhs)
  {
    int lhs_channel = lhs.channel.value_or(-1);
    int rhs_channel = rhs.channel.value_or(-1);
    return std::tie(lhs.aligned_sample_index, lhs_channel, lhs.fft_size, lhs.window) <
           std::tie(rhs.aligned_sample_index, rhs_channel, rhs.fft_size, rhs.window);
  }

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
