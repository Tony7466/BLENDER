/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

vec4 fresnel_conductor(float cosi, const vec3 eta, const vec3 k)
{
  const vec4 cosiv4 = vec4(cosi);
  const vec4 etav4 = vec4(eta, 1.0);
  const vec4 kv4 = vec4(k, 1.0);

  const vec4 cosi2 = vec4(cosi * cosi);
  const vec4 one = vec4(1.0);
  const vec4 tmp_f = (etav4 * etav4) + (kv4 * kv4);

  const vec4 tmp = tmp_f * cosi2;
  const vec4 Rparl2 = (tmp - (2.0 * etav4 * cosiv4) + one) / (tmp + (2.0 * etav4 * cosiv4) + one);
  const vec4 Rperp2 = (tmp_f - (2.0 * etav4 * cosiv4) + cosi2) /
                      (tmp_f + (2.0 * etav4 * cosiv4) + cosi2);
  return (Rparl2 + Rperp2) * 0.5;
}

void node_bsdf_conductor(vec4 base_color,
                         vec4 edge_tint,
                         vec3 eta,
                         vec3 k,
                         float roughness,
                         float anisotropy,
                         float rotation,
                         vec3 N,
                         vec3 T,
                         float weight,
                         const float do_multiscatter,
                         const float use_complex_ior,
                         out Closure result)
{
  if (use_complex_ior != 0.0) {
    base_color = fresnel_conductor(1.0, eta, k);
    edge_tint = fresnel_conductor(1.0 / 7.0, eta, k);
  }

  /* Clamp to match Cycles */
  base_color = saturate(base_color);
  edge_tint = saturate(edge_tint);
  roughness = saturate(roughness);
  /* Not used by EEVEE */
  /* anisotropy = saturate(anisotropy); */

  N = safe_normalize(N);
  vec3 V = coordinate_incoming(g_data.P);
  float NV = dot(N, V);

  ClosureReflection reflection_data;
  reflection_data.N = N;
  reflection_data.roughness = roughness;
  vec3 F0 = base_color.rgb;
  vec3 F82 = edge_tint.rgb;
  vec3 metallic_brdf;
  brdf_f82_tint_lut(F0, F82, NV, roughness, do_multiscatter != 0.0, metallic_brdf);
  reflection_data.color = metallic_brdf;
  reflection_data.weight = weight;

  result = closure_eval(reflection_data);
}
