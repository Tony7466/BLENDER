/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void node_bsdf_conductor(vec4 color,
                         vec4 tint,
                         float roughness,
                         float anisotropy,
                         float rotation,
                         vec3 N,
                         vec3 T,
                         float weight,
                         const float do_multiscatter,
                         out Closure result)
{
  N = safe_normalize(N);
  vec3 V = coordinate_incoming(g_data.P);
  float NV = dot(N, V);

  ClosureReflection reflection_data;
  reflection_data.N = N;
  reflection_data.roughness = roughness;
  vec3 F0 = color.rgb;
  vec3 F82 = min(tint.rgb, vec3(1.0));
  vec3 metallic_brdf;
  brdf_f82_tint_lut(F0, F82, NV, roughness, do_multiscatter != 0.0, metallic_brdf);
  reflection_data.color = metallic_brdf;
  reflection_data.weight = weight;

  result = closure_eval(reflection_data);
}
