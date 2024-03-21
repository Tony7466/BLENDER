/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "util/hash.h"

CCL_NAMESPACE_BEGIN

/**
 * Splits samples into two different classes, A and B, which can be
 * compared for variance estimation.
 */
ccl_device_inline bool sample_is_class_A(int pattern, int sample)
{
#if 0
  if (!(pattern == SAMPLING_PATTERN_TABULATED_SOBOL || pattern == SAMPLING_PATTERN_SOBOL_BURLEY)) {
    /* Fallback: assign samples randomly.
     * This is guaranteed to work "okay" for any sampler, but isn't good.
     * (NOTE: the seed constant is just a random number to guard against
     * possible interactions with other uses of the hash. There's nothing
     * special about it.)
     */
    return hash_hp_seeded_uint(sample, 0xa771f873) & 1;
  }
#else
  (void)pattern;
#endif

  /* This follows the approach from section 10.2.1 of "Progressive
   * Multi-Jittered Sample Sequences" by Christensen et al., but
   * implemented with efficient bit-fiddling.
   *
   * This approach also turns out to work equally well with Owen
   * scrambled and shuffled Sobol (see https://developer.blender.org/D15746#429471).
   */
  return popcount(uint(sample) & 0xaaaaaaaa) & 1;
}

CCL_NAMESPACE_END
