/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <shared_mutex>

#include "BLI_array.hh"
#include "BLI_map.hh"
#include "BLI_struct_equality_utils.hh"

#include "DNA_sound_types.h"

void BKE_sound_fft_cache_new(struct bSound *sound);
void BKE_sound_fft_cache_delete(struct bSound *sound);

namespace blender::bke::sound::fft_cache {

struct Parameter {
  bool downmix;
  int fft_size;
  int window;
  int channel;

  BLI_STRUCT_EQUALITY_OPERATORS_4(Parameter, downmix, fft_size, window, channel);

  uint64_t hash() const;
};

struct FFTCache {
  std::shared_mutex mutex;
  Map<Parameter, Array<float>> map;
};

struct FFTCacheRuntime {
  std::shared_ptr<FFTCache> cache;
};

}  // namespace blender::bke::sound::fft_cache
