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

void BKE_sound_fft_cache_new(bSound *sound, int total_samples)
{
  sound->fft_cache = MEM_new<FFTCacheRuntime>(__func__);
  sound->fft_cache->cache = std::make_shared<FFTCache>(total_samples);
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

FFTCache::FFTCache(int total_samples) : total_samples_(total_samples) {}

bool FFTCache::contains(const Parameter &parameter) const
{
  return map_.contains(parameter);
}

Span<float> FFTCache::get(const Parameter &parameter, int sample_index) const
{
  const Array<float> &bins = map_.lookup(parameter);

  const int aligned_sample_index = (sample_index / parameter.fft_size) * parameter.fft_size;
  const int bin_index = math::clamp(
      aligned_sample_index / 2, 0, int(bins.size()) - parameter.fft_size / 2);

  return Span<float>(bins.begin() + bin_index, parameter.fft_size / 2);
}

MutableSpan<float> FFTCache::upsert(const Parameter &parameter, int sample_index)
{
  const int total_bins = math::ceil(total_samples_ / parameter.fft_size) *
                         (parameter.fft_size / 2);

  Array<float> &bins = map_.lookup_or_add(std::move(parameter), Array<float>(total_bins, -1.0f));

  const int aligned_sample_index = (sample_index / parameter.fft_size) * parameter.fft_size;
  const int bin_index = math::clamp(
      aligned_sample_index / 2, 0, total_bins - parameter.fft_size / 2);

  return MutableSpan<float>(bins.begin() + bin_index, parameter.fft_size / 2);
}

}  // namespace blender::bke::sound::fft_cache
