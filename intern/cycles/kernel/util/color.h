/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/integrator/state_util.h"
#include "kernel/util/lookup_table.h"
#include "util/color.h"
#include "util/math.h"

CCL_NAMESPACE_BEGIN

ccl_device float3 xyz_to_rgb(KernelGlobals kg, float3 xyz)
{
  return make_float3(dot(float4_to_float3(kernel_data.film.xyz_to_r), xyz),
                     dot(float4_to_float3(kernel_data.film.xyz_to_g), xyz),
                     dot(float4_to_float3(kernel_data.film.xyz_to_b), xyz));
}

ccl_device float3 xyz_to_rgb_clamped(KernelGlobals kg, float3 xyz)
{
  return max(xyz_to_rgb(kg, xyz), zero_float3());
}

ccl_device float3 rec709_to_rgb(KernelGlobals kg, float3 rec709)
{
  return (kernel_data.film.is_rec709) ?
             rec709 :
             make_float3(dot(float4_to_float3(kernel_data.film.rec709_to_r), rec709),
                         dot(float4_to_float3(kernel_data.film.rec709_to_g), rec709),
                         dot(float4_to_float3(kernel_data.film.rec709_to_b), rec709));
}

ccl_device float linear_rgb_to_gray(KernelGlobals kg, float3 c)
{
  return dot(c, float4_to_float3(kernel_data.film.rgb_to_y));
}

ccl_device float3 find_position_in_lookup_unit_step(
    ccl_constant float lookup[][3], float position_to_find, int start, int end, int step)
{
  if (UNLIKELY(position_to_find <= start)) {
    return load_float3(lookup[0]);
  }
  if (UNLIKELY(position_to_find >= end)) {
    int i = (end - start) / step;
    return load_float3(lookup[i]);
  }

  float lookup_pos = (position_to_find - start) / (float)step;
  int lower_bound = floor_to_int(lookup_pos);
  int upper_bound = min(lower_bound + 1, (end - start) / step);
  float progress = lookup_pos - int(lookup_pos);
  return mix(load_float3(lookup[lower_bound]), load_float3(lookup[upper_bound]), progress);
}

ccl_device float find_position_in_lookup_unit_step(
    ccl_constant float lookup[], float position_to_find, int start, int end, int step)
{
  if (UNLIKELY(position_to_find <= start)) {
    return lookup[0];
  }
  if (UNLIKELY(position_to_find >= end)) {
    int i = (end - start) / step;
    return lookup[i];
  }

  float lookup_pos = (position_to_find - start) / (float)step;
  int lower_bound = floor_to_int(lookup_pos);
  int upper_bound = min(lower_bound + 1, (end - start) / step);
  float progress = lookup_pos - int(lookup_pos);
  return mix(lookup[lower_bound], lookup[upper_bound], progress);
}

template<typename ConstIntegratorGenericState>
ccl_device_inline Spectrum
rgb_to_spectrum(KernelGlobals kg, ConstIntegratorGenericState state, int32_t path_flag, float3 rgb)
{
#ifndef __SPECTRAL_RENDERING__
  return rgb;
#else
  const Spectrum wavelengths = integrator_state_wavelengths(state, path_flag);

  Spectrum intensities;
  FOREACH_SPECTRUM_CHANNEL (i) {
    /* Find position in the lookup of wavelength. */
    float3 magnitudes = find_position_in_lookup_unit_step(
        rec709_wavelength_lookup, GET_SPECTRUM_CHANNEL(wavelengths, i), 360, 830, 1);
    /* Multiply the lookups by the RGB factors. */
    float3 contributions = magnitudes * rgb;
    /* Add the three components. */
    GET_SPECTRUM_CHANNEL(intensities, i) = reduce_add(contributions);
  }

  return intensities;
#endif
}

#ifndef __KERNEL_METAL__
ccl_device_inline Spectrum rgb_to_spectrum(KernelGlobals kg,
                                           std::nullptr_t state,
                                           int32_t path_flag,
                                           float3 rgb)
{
  return zero_spectrum();
}
#endif

ccl_device float3 wavelength_to_xyz(KernelGlobals kg, float wavelength)
{
  int table_offset = kernel_data.cam.camera_response_function_offset;

  float position = mix(0.0f,
                       WAVELENGTH_CDF_TABLE_SIZE - 1.0f,
                       inverse_lerp(MIN_WAVELENGTH, MAX_WAVELENGTH, wavelength));

  int lower_bound = floor_to_int(position);
  int upper_bound = min(lower_bound + 1, WAVELENGTH_CDF_TABLE_SIZE - 1);
  float progress = position - lower_bound;

  float3 lower_value = make_float3(
      kernel_data_fetch(lookup_table, table_offset + 3 * lower_bound + 0),
      kernel_data_fetch(lookup_table, table_offset + 3 * lower_bound + 1),
      kernel_data_fetch(lookup_table, table_offset + 3 * lower_bound + 2));
  float3 upper_value = make_float3(
      kernel_data_fetch(lookup_table, table_offset + 3 * upper_bound + 0),
      kernel_data_fetch(lookup_table, table_offset + 3 * upper_bound + 1),
      kernel_data_fetch(lookup_table, table_offset + 3 * upper_bound + 2));

  return mix(lower_value, upper_value, progress);
}

template<typename ConstIntegratorGenericState>
ccl_device_inline float3
spectrum_to_rgb(KernelGlobals kg, ConstIntegratorGenericState state, int32_t path_flag, Spectrum s)
{
#ifndef __SPECTRAL_RENDERING__
  return s;
#else
  const Spectrum wavelengths = integrator_state_wavelengths(state, path_flag);

  float3 xyz_sum = zero_float3();
  FOREACH_SPECTRUM_CHANNEL (i) {
    const float wavelength = GET_SPECTRUM_CHANNEL(wavelengths, i);

    const float wavelength_importance = lookup_table_read(
        kg,
        inverse_lerp(MIN_WAVELENGTH, MAX_WAVELENGTH, wavelength),
        kernel_data.cam.wavelength_importance_table_offset,
        WAVELENGTH_CDF_TABLE_SIZE);

    xyz_sum += wavelength_to_xyz(kg, wavelength) * GET_SPECTRUM_CHANNEL(s, i) /
               wavelength_importance;
  }

  xyz_sum *= 3.0f / SPECTRUM_CHANNELS;

  return xyz_to_rgb(kg, xyz_sum);
#endif
}

#ifndef __KERNEL_METAL__
ccl_device_inline float3 spectrum_to_rgb(KernelGlobals kg,
                                         std::nullptr_t state,
                                         int32_t path_flag,
                                         Spectrum s)
{
  return zero_float3();
}
#endif

template<typename ConstIntegratorGenericState>
ccl_device float spectrum_to_gray(KernelGlobals kg,
                                  ConstIntegratorGenericState state,
                                  int32_t path_flag,
                                  Spectrum s)
{
  return linear_rgb_to_gray(kg, spectrum_to_rgb(kg, state, path_flag, s));
}

CCL_NAMESPACE_END
