/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_sound_fft_cache.hh"
#include "BLI_math_base.hh"

using blender::bke::sound::fft_cache::FFTCache;
using blender::bke::sound::fft_cache::FFTCacheRuntime;

void BKE_sound_fft_cache_new(bSound *sound, int length)
{
  sound->fft_cache = MEM_new<FFTCacheRuntime>(__func__);
  sound->fft_cache->cache = std::make_shared<FFTCache>(length * sound->samplerate);
}

void BKE_sound_fft_cache_delete(bSound *sound)
{
  MEM_delete(sound->fft_cache);
}

namespace blender::bke::sound::fft_cache {

uint64_t FFTParameter::hash() const
{
  const uint64_t h4 = std::hash<std::optional<int>>{}(channel);
  return get_default_hash(aligned_sample_index, fft_size, window) ^ (h4 * 3632623);
}

FFTCache::FFTCache(int total_samples) : total_samples_(total_samples) {}

FFTResult *FFTCache::try_get_or_compute(const FFTParameter &parameter,
                                        const FunctionRef<std::unique_ptr<FFTResult>()> fn)
{
  // TODO: do not lock every time we read
  std::lock_guard lock{mutex_};
  FFTResult *result = map_.lookup_or_add_cb(parameter, fn).get();

  return result;
}

}  // namespace blender::bke::sound::fft_cache
