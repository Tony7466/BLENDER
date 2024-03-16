/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "integrator/adaptive_sampling.h"
#include "util/math.h"

CCL_NAMESPACE_BEGIN

// Mark small, frequently called functions as inline for potential performance gains
inline bool need_filter_impl(int sample, bool use, int min_samples, int adaptive_step) {
    if (!use) {
        return false;
    }

    if (sample <= min_samples) {
        return false;
    }

    return (sample & (adaptive_step - 1)) == (adaptive_step - 1);
}

AdaptiveSampling::AdaptiveSampling() {}

int AdaptiveSampling::align_samples(int start_sample, int num_samples) const
{
    // Check if adaptive sampling is disabled
    if (!use) {
        return num_samples;
    }

    // Calculate the 0-based sample index at which the first filtering will happen
    const int first_filter_sample = (min_samples + 1) | (adaptive_step - 1);

    // Allow as many samples as possible until the first filter sample
    if (start_sample + num_samples <= first_filter_sample) {
        return num_samples;
    }

    // Find the sample index at which the next filtering will happen
    const int next_filter_sample = max(first_filter_sample, start_sample | (adaptive_step - 1));

    // Calculate the number of samples until the next filter
    const int num_samples_until_filter = next_filter_sample - start_sample + 1;

    // Return the minimum of num_samples and num_samples_until_filter
    return min(num_samples_until_filter, num_samples);
}

bool AdaptiveSampling::need_filter(int sample) const
{
    // Reuse the need_filter implementation for better performance
    return need_filter_impl(sample, use, min_samples, adaptive_step);
}

CCL_NAMESPACE_END
