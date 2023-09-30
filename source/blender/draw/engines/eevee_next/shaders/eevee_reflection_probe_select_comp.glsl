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

  SphericalHarmonicL1 sh = lightprobe_irradiance_sample(reflection_probe_buf[idx].pos);

  reflection_probe_buf[idx].irradiance = reflection_probes_spherical_harmonic_encode(sh);
}
