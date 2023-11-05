/* SPDX-FileCopyrightText: 2019-2022 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

void node_bsdf_glass(vec4 color,
                     vec4 specular_tint,
                     float roughness,
                     float ior,
                     vec3 N,
                     float weight,
                     const float do_multiscatter,
                     out Closure result)
{
  color = saturate(color);
  roughness = saturate(roughness);
  ior = max(ior, 1e-5f);

  N = safe_normalize(N);
  vec3 V = coordinate_incoming(g_data.P);
  float NV = dot(N, V);

  vec3 F0 = vec3(F0_from_ior(ior)) * specular_tint.rgb;
  vec3 F90 = vec3(1.0);
  vec3 reflectance, transmittance;
  bsdf_lut(
      F0, F90, color.rgb, NV, roughness, ior, do_multiscatter != 0.0, reflectance, transmittance);

  ClosureReflection reflection_data;
  reflection_data.weight = weight;
  reflection_data.color = reflectance;
  reflection_data.N = N;
  reflection_data.roughness = roughness;

  ClosureRefraction refraction_data;
  refraction_data.weight = weight;
  refraction_data.color = transmittance;
  refraction_data.N = N;
  refraction_data.roughness = roughness;
  refraction_data.ior = ior;

  result = closure_eval(reflection_data, refraction_data);
}
