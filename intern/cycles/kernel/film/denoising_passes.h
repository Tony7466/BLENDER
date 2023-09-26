/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/geom/geom.h"

#include "kernel/film/write.h"

CCL_NAMESPACE_BEGIN

#ifdef __DENOISING_FEATURES__
ccl_device_forceinline void film_write_denoising_features_surface(KernelGlobals kg,
                                                                  IntegratorState state,
                                                                  ccl_private const ShaderData *sd,
                                                                  ccl_global float *ccl_restrict
                                                                      render_buffer)
{
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  if (!(path_flag & PATH_RAY_DENOISING_FEATURES)) {
    return;
  }

  /* Skip implicitly transparent surfaces. */
  if (sd->flag & SD_HAS_ONLY_VOLUME) {
    return;
  }

  /* Don't write denoising passes for paths that were split off for shadow catchers
   * to avoid double-counting. */
  if (path_flag & PATH_RAY_SHADOW_CATCHER_PASS) {
    return;
  }

  ccl_global float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer);

  if (kernel_data.film.pass_denoising_depth != PASS_UNUSED) {
    const Spectrum denoising_feature_throughput = INTEGRATOR_STATE(
        state, path, denoising_feature_throughput);
    const float depth = sd->ray_length - INTEGRATOR_STATE(state, ray, tmin);
    const float denoising_depth = ensure_finite(average(denoising_feature_throughput) * depth);
    film_write_pass_float(buffer + kernel_data.film.pass_denoising_depth, denoising_depth);
  }

  float3 normal = zero_float3();
  Spectrum diffuse_albedo = zero_spectrum();
  float sum_weight = 0.0f;

  for (int i = 0; i < sd->num_closure; i++) {
    ccl_private const ShaderClosure *sc = &sd->closure[i];

    if (!CLOSURE_IS_BSDF_OR_BSSRDF(sc->type)) {
      continue;
    }

    /* All closures contribute to the normal feature, but only diffuse-like ones to the albedo. */
    /* If far-field hair, use fiber tangent as feature instead of normal. */
    normal += (sc->type == CLOSURE_BSDF_HAIR_HUANG_ID ? safe_normalize(sd->dPdu) : sc->N) *
              sc->sample_weight;
    sum_weight += sc->sample_weight;

    Spectrum closure_albedo = bsdf_albedo(kg, sd, sc, true, true);
    diffuse_albedo += closure_albedo;
  }

  if (sum_weight != 0.0f) {
    normal /= sum_weight;
  }

  if (kernel_data.film.pass_denoising_normal != PASS_UNUSED) {
    /* Transform normal into camera space. */
    const Transform worldtocamera = kernel_data.cam.worldtocamera;
    normal = transform_direction(&worldtocamera, normal);

    const float3 denoising_normal = ensure_finite(normal);
    film_write_pass_float3(buffer + kernel_data.film.pass_denoising_normal, denoising_normal);
  }

  if (kernel_data.film.pass_denoising_albedo != PASS_UNUSED) {
    const Spectrum denoising_feature_throughput = INTEGRATOR_STATE(
        state, path, denoising_feature_throughput);
    const Spectrum denoising_albedo = ensure_finite(denoising_feature_throughput * diffuse_albedo);
    film_write_pass_spectrum(buffer + kernel_data.film.pass_denoising_albedo, denoising_albedo);
  }

  INTEGRATOR_STATE_WRITE(state, path, flag) &= ~PATH_RAY_DENOISING_FEATURES;
}

ccl_device_forceinline void film_write_denoising_features_volume(KernelGlobals kg,
                                                                 IntegratorState state,
                                                                 const Spectrum albedo,
                                                                 const bool scatter,
                                                                 ccl_global float *ccl_restrict
                                                                     render_buffer)
{
  ccl_global float *buffer = film_pass_pixel_render_buffer(kg, state, render_buffer);
  const Spectrum denoising_feature_throughput = INTEGRATOR_STATE(
      state, path, denoising_feature_throughput);

  if (scatter && kernel_data.film.pass_denoising_normal != PASS_UNUSED) {
    /* Assume scatter is sufficiently diffuse to stop writing denoising features. */
    INTEGRATOR_STATE_WRITE(state, path, flag) &= ~PATH_RAY_DENOISING_FEATURES;

    /* Write view direction as normal. */
    const float3 denoising_normal = make_float3(0.0f, 0.0f, -1.0f);
    film_write_pass_float3(buffer + kernel_data.film.pass_denoising_normal, denoising_normal);
  }

  if (kernel_data.film.pass_denoising_albedo != PASS_UNUSED) {
    /* Write albedo. */
    const Spectrum denoising_albedo = ensure_finite(denoising_feature_throughput * albedo);
    film_write_pass_spectrum(buffer + kernel_data.film.pass_denoising_albedo, denoising_albedo);
  }
}
#endif /* __DENOISING_FEATURES__ */

CCL_NAMESPACE_END
