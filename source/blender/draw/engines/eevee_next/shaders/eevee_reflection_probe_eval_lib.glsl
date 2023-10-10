/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma BLENDER_REQUIRE(gpu_shader_math_base_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_bxdf_sampling_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_reflection_probe_lib.glsl)

#ifdef REFLECTION_PROBE
int reflection_probes_select(vec3 P, float random_probe)
{
  for (int index = 1; index < REFLECTION_PROBES_MAX; index++) {
    ReflectionProbeData probe_data = reflection_probe_buf[index];
    /* ReflectionProbeData doesn't contain any gap, exit at first item that is invalid. */
    if (probe_data.atlas_coord.layer != -1) {
      /* We hit the end of the array. Return last valid index. */
      return index - 1;
    }
    float gradient = length(vec4(P, 1.0) * probe_data.world_to_probe_transposed);
    float score = saturate(gradient * probe_data.influence_scale + probe_data.influence_bias);
    if (score < random_probe) {
      return index;
    }
  }
  /* This should never happen (world probe is always last). */
  return 0;
}
#endif /* REFLECTION_PROBE */
