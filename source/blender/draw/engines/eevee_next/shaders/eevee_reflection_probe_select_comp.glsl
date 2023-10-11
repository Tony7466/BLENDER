/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * Select spherical probes to be considered during render and copy irradiance data from the
 * irradiance cache from each spherical probe location except for the world probe.
 */

#pragma BLENDER_REQUIRE(eevee_lightprobe_eval_lib.glsl)

void main()
{
  int idx = int(gl_GlobalInvocationID.x);
  if (idx >= reflection_probe_count) {
    return;
  }

  SphericalHarmonicL1 sh;
  if (idx == 0) {
    sh = lightprobe_irradiance_world();
  }
  else {
    /* NOTE: The vector-matrix multiplication swapped on purpose to cancel the matrix transpose. */
    vec3 probe_center = vec4(0.0, 0.0, 0.0, -1.0) *
                        reflection_probe_buf[idx].world_to_probe_transposed;
    sh = lightprobe_irradiance_sample(probe_center);
  }

  reflection_probe_buf[idx].low_freq_light = reflection_probes_extract_low_freq(sh);
}
