/* SPDX-License-Identifier: Apache-2.0 */
/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation */

#pragma once

#include "kernel/sample/util.h"
#include "util/hash.h"

CCL_NAMESPACE_BEGIN

// Precomputed Sobol sequence table
const int MAX_SAMPLES = 1000000;
float precomputed_samples[MAX_SAMPLES][NUM_TAB_SOBOL_DIMENSIONS];

// Function to precompute Sobol samples
void precompute_sobol_samples() {
    // Implement Sobol sequence generation and store in precomputed_samples array
    // This can be done offline or during initialization
}

// Function to generate 1D sample using precomputed table
ccl_device_inline float tabulated_sobol_sample_1D(uint sample_index) {
    return precomputed_samples[sample_index % MAX_SAMPLES][0];
}

// Function to generate 2D sample using precomputed table
ccl_device_inline float2 tabulated_sobol_sample_2D(uint sample_index) {
    float2 sample;
    sample.x = precomputed_samples[sample_index % MAX_SAMPLES][0];
    sample.y = precomputed_samples[sample_index % MAX_SAMPLES][1];
    return sample;
}

// Function to generate a 1D sample using tabulated Sobol sequence
ccl_device_inline float tabulated_sobol_sample_1D(KernelGlobals kg,
                                                  uint sample,
                                                  const uint rng_hash,
                                                  const uint dimension)
{
    uint seed = kernel_data.integrator.scrambling_distance < 1.0f ?
                kernel_data.integrator.seed : rng_hash;

    const uint index = tabulated_sobol_shuffled_sample_index(kg, sample, dimension, seed);
    float x = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS);

    // Do limited Cranley-Patterson rotation when using scrambling distance.
    if (kernel_data.integrator.scrambling_distance < 1.0f) {
        const float jitter_x = hash_wang_seeded_float(dimension, rng_hash) *
                               kernel_data.integrator.scrambling_distance;
        x += jitter_x - floorf(x + jitter_x);
    }

    return x;
}

// Function to generate a 2D sample using tabulated Sobol sequence
ccl_device_inline float2 tabulated_sobol_sample_2D(KernelGlobals kg,
                                                   uint sample,
                                                   const uint rng_hash,
                                                   const uint dimension)
{
    uint seed = kernel_data.integrator.scrambling_distance < 1.0f ?
                kernel_data.integrator.seed : rng_hash;

    const uint index = tabulated_sobol_shuffled_sample_index(kg, sample, dimension, seed);
    float x = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS);
    float y = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 1);

    // Do limited Cranley-Patterson rotation when using scrambling distance.
    if (kernel_data.integrator.scrambling_distance < 1.0f) {
        const float jitter_x = hash_wang_seeded_float(dimension, rng_hash) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_y = hash_wang_seeded_float(dimension, rng_hash ^ 0xca0e1151) *
                               kernel_data.integrator.scrambling_distance;
        x += jitter_x - floorf(x + jitter_x);
        y += jitter_y - floorf(y + jitter_y);
    }

    return make_float2(x, y);
}

// Parallelized function to generate samples
void generate_samples_parallel(uint start_index, uint end_index, float* samples) {
    #pragma omp parallel for
    for (uint i = start_index; i < end_index; ++i) {
        samples[i] = tabulated_sobol_sample_1D(i);
    }
}

// Function to compute the shuffled sample index for Sobol sequence
ccl_device_inline uint tabulated_sobol_shuffled_sample_index(KernelGlobals kg,
                                                             uint sample,
                                                             uint dimension,
                                                             uint seed)
{
    const uint sample_count = kernel_data.integrator.tabulated_sobol_sequence_size;
    const uint pattern_i = hash_shuffle_uint(dimension, NUM_TAB_SOBOL_PATTERNS, seed);
    const uint sample_mask = sample_count - 1;
    const uint sample_shuffled = nested_uniform_scramble(sample,
                                                         hash_wang_seeded_uint(dimension, seed));
    return ((pattern_i * sample_count) + sample_shuffled) & sample_mask;
}

// Function to generate a 3D sample using tabulated Sobol sequence
ccl_device_inline float3 tabulated_sobol_sample_3D(KernelGlobals kg,
                                                   uint sample,
                                                   const uint rng_hash,
                                                   const uint dimension)
{
    uint seed = kernel_data.integrator.scrambling_distance < 1.0f ?
                kernel_data.integrator.seed : rng_hash;

    const uint index = tabulated_sobol_shuffled_sample_index(kg, sample, dimension, seed);
    float x = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS);
    float y = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 1);
    float z = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 2);

    // Do limited Cranley-Patterson rotation when using scrambling distance.
    if (kernel_data.integrator.scrambling_distance < 1.0f) {
        const float jitter_x = hash_wang_seeded_float(dimension, rng_hash) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_y = hash_wang_seeded_float(dimension, rng_hash ^ 0xca0e1151) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_z = hash_wang_seeded_float(dimension, rng_hash ^ 0xbf604c5a) *
                               kernel_data.integrator.scrambling_distance;
        x += jitter_x - floorf(x + jitter_x);
        y += jitter_y - floorf(y + jitter_y);
        z += jitter_z - floorf(z + jitter_z);
    }

    return make_float3(x, y, z);
}

// Function to generate a 4D sample using tabulated Sobol sequence
ccl_device_inline float4 tabulated_sobol_sample_4D(KernelGlobals kg,
                                                   uint sample,
                                                   const uint rng_hash,
                                                   const uint dimension)
{
    uint seed = kernel_data.integrator.scrambling_distance < 1.0f ?
                kernel_data.integrator.seed : rng_hash;

    const uint index = tabulated_sobol_shuffled_sample_index(kg, sample, dimension, seed);
    float x = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS);
    float y = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 1);
    float z = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 2);
    float w = kernel_data_fetch(sample_pattern_lut, index * NUM_TAB_SOBOL_DIMENSIONS + 3);

    // Do limited Cranley-Patterson rotation when using scrambling distance.
    if (kernel_data.integrator.scrambling_distance < 1.0f) {
        const float jitter_x = hash_wang_seeded_float(dimension, rng_hash) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_y = hash_wang_seeded_float(dimension, rng_hash ^ 0xca0e1151) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_z = hash_wang_seeded_float(dimension, rng_hash ^ 0xbf604c5a) *
                               kernel_data.integrator.scrambling_distance;
        const float jitter_w = hash_wang_seeded_float(dimension, rng_hash ^ 0x99634d1d) *
                               kernel_data.integrator.scrambling_distance;
        x += jitter_x - floorf(x + jitter_x);
        y += jitter_y - floorf(y + jitter_y);
        z += jitter_z - floorf(z + jitter_z);
        w += jitter_w - floorf(w + jitter_w);
    }

    return make_float4(x, y, z, w);
}

CCL_NAMESPACE_END
