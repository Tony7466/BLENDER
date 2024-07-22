/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_sound_fft_cache.hh"

using blender::bke::sound::fft_cache::FFTCacheRuntime;
using blender::bke::sound::fft_cache::FFTCache;

void BKE_sound_fft_cache_new(bSound *sound)
{
  sound->fft_cache = MEM_new<FFTCacheRuntime>(__func__);
  sound->fft_cache->cache = std::make_shared<FFTCache>();
}

void BKE_sound_fft_cache_delete(bSound *sound)
{
  MEM_delete(sound->fft_cache);
}

namespace blender::bke::sound::fft_cache {

uint64_t Parameter::hash() const
{
  return get_default_hash(downmix, fft_size, window, channel);
}

}  // namespace blender::bke::sound::fft_cache
