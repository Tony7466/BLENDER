/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/integrator/guiding.h"
#include "kernel/integrator/path_state.h"
#include "kernel/integrator/state_flow.h"
#include "kernel/integrator/surface_shader.h"
#include "kernel/integrator/volume_shader.h"
#include "kernel/integrator/volume_stack.h"

CCL_NAMESPACE_BEGIN

ccl_device_inline bool shadow_intersections_has_remaining(const uint num_hits)
{
  return num_hits >= INTEGRATOR_SHADOW_ISECT_SIZE;
}

#ifdef __TRANSPARENT_SHADOWS__
ccl_device_inline Spectrum integrate_transparent_surface_shadow(KernelGlobals kg,
                                                                IntegratorShadowState state,
                                                                const int hit)
{
  PROFILING_INIT(kg, PROFILING_SHADE_SHADOW_SURFACE);

  /* TODO: does aliasing like this break automatic SoA in CUDA?
   * Should we instead store closures separate from ShaderData?
   *
   * TODO: is it better to declare this outside the loop or keep it local
   * so the compiler can see there is no dependency between iterations? */
  ShaderDataTinyStorage shadow_sd_storage;
  ccl_private ShaderData *shadow_sd = AS_SHADER_DATA(&shadow_sd_storage);

  /* Setup shader data at surface. */
  Intersection isect ccl_optional_struct_init;
  integrator_state_read_shadow_isect(state, &isect, hit);

  Ray ray ccl_optional_struct_init;
  integrator_state_read_shadow_ray(state, &ray);

  shader_setup_from_ray(kg, shadow_sd, &ray, &isect);

  /* Evaluate shader. */
  if (!(shadow_sd->flag & SD_HAS_ONLY_VOLUME)) {
    surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW>(
        kg, state, shadow_sd, NULL, PATH_RAY_SHADOW);
  }

#  ifdef __VOLUME__
  /* Exit/enter volume. */
  shadow_volume_stack_enter_exit(kg, state, shadow_sd);
#  endif

  /* Compute transparency from closures. */
  return surface_shader_transparency(kg, shadow_sd);
}

#  ifdef __VOLUME__

/* Evaluate shader to get extinction coefficient at P. */
ccl_device_inline bool shadow_volume_shader_sample(KernelGlobals kg,
                                                   IntegratorShadowState state,
                                                   ccl_private ShaderData *ccl_restrict sd,
                                                   ccl_private Spectrum *ccl_restrict extinction)
{
  VOLUME_READ_LAMBDA(integrator_state_read_shadow_volume_stack(state, i))
  volume_shader_eval<true>(kg, state, sd, PATH_RAY_SHADOW, volume_read_lambda_pass);

  if (!(sd->flag & SD_EXTINCTION)) {
    return false;
  }

  const float density = object_volume_density(kg, sd->object);
  *extinction = sd->closure_transparent_extinction * density;
  return true;
}

/* Volume Shadows
 *
 * These functions are used to attenuate shadow rays to lights. Both absorption
 * and scattering will block light, represented by the extinction coefficient. */

#    if 0
/* homogeneous volume: assume shader evaluation at the starts gives
 * the extinction coefficient for the entire line segment */
ccl_device void volume_shadow_homogeneous(KernelGlobals kg, IntegratorState state,
                                          ccl_private Ray *ccl_restrict ray,
                                          ccl_private ShaderData *ccl_restrict sd,
                                          ccl_global Spectrum *ccl_restrict throughput)
{
  Spectrum sigma_t = zero_spectrum();

  if (shadow_volume_shader_sample(kg, state, sd, &sigma_t)) {
    *throughput *= volume_color_transmittance(sigma_t, ray->tmax - ray->tmin);
  }
}
#    endif

/* heterogeneous volume: integrate stepping through the volume until we
 * reach the end, get absorbed entirely, or run out of iterations */
ccl_device void volume_shadow_heterogeneous(KernelGlobals kg,
                                            IntegratorShadowState state,
                                            ccl_private Ray *ccl_restrict ray,
                                            ccl_private ShaderData *ccl_restrict sd,
                                            ccl_private Spectrum *ccl_restrict throughput,
                                            const float object_step_size)
{
  /* Load random number state. */
  RNGState rng_state;
  shadow_path_state_rng_load(state, &rng_state);

  Spectrum tp = *throughput;

  /* Prepare for stepping.
   * For shadows we do not offset all segments, since the starting point is
   * already a random distance inside the volume. It also appears to create
   * banding artifacts for unknown reasons. */
  int max_steps;
  float step_size, step_shade_offset, unused;
  volume_step_init(kg,
                   &rng_state,
                   object_step_size,
                   ray->tmin,
                   ray->tmax,
                   &step_size,
                   &step_shade_offset,
                   &unused,
                   &max_steps);
  const float steps_offset = 1.0f;

  /* compute extinction at the start */
  float t = ray->tmin;

  Spectrum sum = zero_spectrum();

  for (int i = 0; i < max_steps; i++) {
    /* advance to new position */
    float new_t = min(ray->tmax, ray->tmin + (i + steps_offset) * step_size);
    float dt = new_t - t;

    float3 new_P = ray->P + ray->D * (t + dt * step_shade_offset);
    Spectrum sigma_t = zero_spectrum();

    /* compute attenuation over segment */
    sd->P = new_P;
    if (shadow_volume_shader_sample(kg, state, sd, &sigma_t)) {
      /* Compute `expf()` only for every Nth step, to save some calculations
       * because `exp(a)*exp(b) = exp(a+b)`, also do a quick #VOLUME_THROUGHPUT_EPSILON
       * check then. */
      sum += (-sigma_t * dt);
      if ((i & 0x07) == 0) { /* TODO: Other interval? */
        tp = *throughput * exp(sum);

        /* stop if nearly all light is blocked */
        if (reduce_max(tp) < VOLUME_THROUGHPUT_EPSILON) {
          break;
        }
      }
    }

    /* stop if at the end of the volume */
    t = new_t;
    if (t == ray->tmax) {
      /* Update throughput in case we haven't done it above */
      tp = *throughput * exp(sum);
      break;
    }
  }

  *throughput = tp;
}

ccl_device_inline void integrate_transparent_volume_shadow(KernelGlobals kg,
                                                           IntegratorShadowState state,
                                                           const int hit,
                                                           const int num_recorded_hits,
                                                           ccl_private Spectrum *ccl_restrict
                                                               throughput)
{
  PROFILING_INIT(kg, PROFILING_SHADE_SHADOW_VOLUME);

  /* TODO: deduplicate with surface, or does it not matter for memory usage? */
  ShaderDataTinyStorage shadow_sd_storage;
  ccl_private ShaderData *shadow_sd = AS_SHADER_DATA(&shadow_sd_storage);

  /* Setup shader data. */
  Ray ray ccl_optional_struct_init;
  integrator_state_read_shadow_ray(state, &ray);
  ray.self.object = OBJECT_NONE;
  ray.self.prim = PRIM_NONE;
  ray.self.light_object = OBJECT_NONE;
  ray.self.light_prim = PRIM_NONE;
  ray.self.light = LAMP_NONE;
  /* Modify ray position and length to match current segment. */
  ray.tmin = (hit == 0) ? ray.tmin : INTEGRATOR_STATE_ARRAY(state, shadow_isect, hit - 1, t);
  ray.tmax = (hit < num_recorded_hits) ? INTEGRATOR_STATE_ARRAY(state, shadow_isect, hit, t) :
                                         ray.tmax;

  shader_setup_from_volume(kg, shadow_sd, &ray);

  VOLUME_READ_LAMBDA(integrator_state_read_shadow_volume_stack(state, i));
  const float step_size = volume_stack_step_size(kg, volume_read_lambda_pass);

  volume_shadow_heterogeneous(kg, state, &ray, shadow_sd, throughput, step_size);
}

#  endif

ccl_device_inline bool integrate_transparent_shadow(KernelGlobals kg,
                                                    IntegratorShadowState state,
                                                    const uint num_hits)
{
  /* Accumulate shadow for transparent surfaces. */
  const uint num_recorded_hits = min(num_hits, INTEGRATOR_SHADOW_ISECT_SIZE);

  for (uint hit = 0; hit < num_recorded_hits + 1; hit++) {
    /* Volume shaders. */
    if (hit < num_recorded_hits || !shadow_intersections_has_remaining(num_hits)) {
#  ifdef __VOLUME__
      if (!integrator_state_shadow_volume_stack_is_empty(kg, state)) {
        Spectrum throughput = INTEGRATOR_STATE(state, shadow_path, throughput);
        integrate_transparent_volume_shadow(kg, state, hit, num_recorded_hits, &throughput);
        if (is_zero(throughput)) {
          return true;
        }

        INTEGRATOR_STATE_WRITE(state, shadow_path, throughput) = throughput;
      }
#  endif
    }

    /* Surface shaders. */
    if (hit < num_recorded_hits) {
      const Spectrum shadow = integrate_transparent_surface_shadow(kg, state, hit);
      const Spectrum throughput = INTEGRATOR_STATE(state, shadow_path, throughput) * shadow;
      if (is_zero(throughput)) {
        return true;
      }

      INTEGRATOR_STATE_WRITE(state, shadow_path, throughput) = throughput;
      INTEGRATOR_STATE_WRITE(state, shadow_path, transparent_bounce) += 1;
      INTEGRATOR_STATE_WRITE(state, shadow_path, rng_offset) += PRNG_BOUNCE_NUM;
    }

    /* Note we do not need to check max_transparent_bounce here, the number
     * of intersections is already limited and made opaque in the
     * INTERSECT_SHADOW kernel. */
  }

  if (shadow_intersections_has_remaining(num_hits)) {
    /* There are more hits that we could not recorded due to memory usage,
     * adjust ray to intersect again from the last hit. */
    const float last_hit_t = INTEGRATOR_STATE_ARRAY(state, shadow_isect, num_recorded_hits - 1, t);
    INTEGRATOR_STATE_WRITE(state, shadow_ray, tmin) = intersection_t_offset(last_hit_t);
  }

  return false;
}
#endif /* __TRANSPARENT_SHADOWS__ */

ccl_device_noinline void integrator_shade_shadow(KernelGlobals kg,
                                                 IntegratorShadowState state,
                                                 ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_SHADE_SHADOW_SETUP);
  const uint num_hits = INTEGRATOR_STATE(state, shadow_path, num_hits);

#ifdef __TRANSPARENT_SHADOWS__
  /* Evaluate transparent shadows. */
  const bool opaque = integrate_transparent_shadow(kg, state, num_hits);
  if (opaque) {
    integrator_shadow_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_SHADOW);
    return;
  }
#endif

  if (shadow_intersections_has_remaining(num_hits)) {
    /* More intersections to find, continue shadow ray. */
    integrator_shadow_path_next(kg,
                                state,
                                DEVICE_KERNEL_INTEGRATOR_SHADE_SHADOW,
                                DEVICE_KERNEL_INTEGRATOR_INTERSECT_SHADOW);
    return;
  }
  else {
    guiding_record_direct_light(kg, state);
    film_write_direct_light(kg, state, render_buffer);
    integrator_shadow_path_terminate(kg, state, DEVICE_KERNEL_INTEGRATOR_SHADE_SHADOW);
    return;
  }
}

CCL_NAMESPACE_END
