/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#pragma once

#include "kernel/integrator/path_state.h"
#include "kernel/integrator/surface_shader.h"

#include "kernel/film/data_passes.h"
#include "kernel/film/denoising_passes.h"
#include "kernel/film/light_passes.h"

#include "kernel/light/sample.h"

#include "kernel/integrator/mnee.h"

#include "kernel/integrator/guiding.h"
#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/shadow_linking.h"
#include "kernel/integrator/subsurface.h"
#include "kernel/integrator/volume_stack.h"

CCL_NAMESPACE_BEGIN

ccl_device_forceinline void integrate_surface_shader_setup(KernelGlobals kg,
                                                           ConstIntegratorState state,
                                                           ccl_private ShaderData *sd)
{
  Intersection isect ccl_optional_struct_init;
  integrator_state_read_isect(state, &isect);

  Ray ray ccl_optional_struct_init;
  integrator_state_read_ray(state, &ray);

  shader_setup_from_ray(kg, sd, &ray, &isect);
}

ccl_device_forceinline float3 integrate_surface_ray_offset(KernelGlobals kg,
                                                           const ccl_private ShaderData *sd,
                                                           const float3 ray_P,
                                                           const float3 ray_D)
{
  /* No ray offset needed for other primitive types. */
  if (!(sd->type & PRIMITIVE_TRIANGLE)) {
    return ray_P;
  }

  /* Self intersection tests already account for the case where a ray hits the
   * same primitive. However precision issues can still cause neighboring
   * triangles to be hit. Here we test if the ray-triangle intersection with
   * the same primitive would miss, implying that a neighboring triangle would
   * be hit instead.
   *
   * This relies on triangle intersection to be watertight, and the object inverse
   * object transform to match the one used by ray intersection exactly.
   *
   * Potential improvements:
   * - It appears this happens when either barycentric coordinates are small,
   *   or dot(sd->Ng, ray_D)  is small. Detect such cases and skip test?
   * - Instead of ray offset, can we tweak P to lie within the triangle?
   */

#ifndef __METALRT__
  /* MetalRT and Cycles triangle tests aren't numerically identical, meaning this method
   * isn't robust for MetalRT. In this case, just applying the ray offset uniformly gives
   * identical looking results.
   */

  float3 verts[3];
  if (sd->type == PRIMITIVE_TRIANGLE) {
    triangle_vertices(kg, sd->prim, verts);
  }
  else {
    kernel_assert(sd->type == PRIMITIVE_MOTION_TRIANGLE);
    motion_triangle_vertices(kg, sd->object, sd->prim, sd->time, verts);
  }

  float3 local_ray_P = ray_P;
  float3 local_ray_D = ray_D;

  if (!(sd->object_flag & SD_OBJECT_TRANSFORM_APPLIED)) {
    const Transform itfm = object_get_inverse_transform(kg, sd);
    local_ray_P = transform_point(&itfm, local_ray_P);
    local_ray_D = transform_direction(&itfm, local_ray_D);
  }

  if (ray_triangle_intersect_self(local_ray_P, local_ray_D, verts)) {
    return ray_P;
  }
  else
#endif
  {
    return ray_offset(ray_P, sd->Ng);
  }
}

ccl_device_forceinline bool integrate_surface_holdout(KernelGlobals kg,
                                                      ConstIntegratorState state,
                                                      ccl_private ShaderData *sd,
                                                      ccl_global float *ccl_restrict render_buffer)
{
  /* Write holdout transparency to render buffer and stop if fully holdout. */
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  if (((sd->flag & SD_HOLDOUT) || (sd->object_flag & SD_OBJECT_HOLDOUT_MASK)) &&
      (path_flag & PATH_RAY_TRANSPARENT_BACKGROUND))
  {
    const Spectrum holdout_weight = surface_shader_apply_holdout(kg, sd);
    const Spectrum throughput = INTEGRATOR_STATE(state, path, throughput);
    const float transparent = average(holdout_weight * throughput);
    film_write_holdout(kg, state, path_flag, transparent, render_buffer);
    if (isequal(holdout_weight, one_spectrum())) {
      return false;
    }
  }

  return true;
}

ccl_device_forceinline void integrate_surface_emission(KernelGlobals kg,
                                                       IntegratorState state,
                                                       ccl_private const ShaderData *sd,
                                                       ccl_global float *ccl_restrict
                                                           render_buffer)
{
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

#ifdef __LIGHT_LINKING__
  if (!light_link_object_match(kg, light_link_receiver_forward(kg, state), sd->object) &&
      !(path_flag & PATH_RAY_CAMERA))
  {
    return;
  }
#endif

#ifdef __SHADOW_LINKING__
  /* Indirect emission of shadow-linked emissive surfaces is done via shadow rays to dedicated
   * light sources. */
  if (kernel_data.kernel_features & KERNEL_FEATURE_SHADOW_LINKING) {
    if (!(path_flag & PATH_RAY_CAMERA) &&
        kernel_data_fetch(objects, sd->object).shadow_set_membership != LIGHT_LINK_MASK_ALL)
    {
      return;
    }
  }
#endif

  /* Evaluate emissive closure. */
  Spectrum L = surface_shader_emission(sd);

  const float mis_weight = light_sample_mis_weight_forward_surface(kg, state, path_flag, sd);

  guiding_record_surface_emission(kg, state, L, mis_weight);
  film_write_surface_emission(
      kg, state, L, mis_weight, render_buffer, object_lightgroup(kg, sd->object));
}

/* TODO(weizhen): move these radiance functions to somewhere more appropriate. */
ccl_device_forceinline bool light_is_direct_illumination(IntegratorState state)
{
  return !(INTEGRATOR_STATE(state, path, flag) & PATH_RAY_ANY_PASS);
}

/* Evaluate the integrant of the rendering equation in area measure (BSDF * L * cos_NO * V * G) and
 * write to `radiance`.
 * Return the pdf of sampling the bsdf. */
ccl_device_forceinline float radiance_eval(KernelGlobals kg,
                                           IntegratorState state,
                                           ccl_private ShaderData *sd,
                                           ccl_private LightSample *ls,
                                           ccl_private BsdfEval *radiance,
                                           const bool check_visibility = false)
{
  if (!(ls->pdf > 0.0f)) {
    bsdf_eval_init(radiance, zero_spectrum());
    return 0.0f;
  }

  ShaderDataCausticsStorage emission_sd_storage;
  ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);

  PROFILING_INIT(kg, PROFILING_RESTIR_LIGHT_EVAL);
  const Spectrum light_eval = light_sample_shader_eval(
      kg, state, emission_sd, ls, sd->time, sd, check_visibility);
  PROFILING_EVENT(PROFILING_RESTIR_BSDF_EVAL);
  const float bsdf_pdf = surface_shader_bsdf_eval(kg, state, sd, ls->D, radiance, ls->shader);
  const float geometry_term = ls->jacobian_solid_angle_to_area();
  bsdf_eval_mul(radiance, light_eval * geometry_term);

  return bsdf_pdf;
}

ccl_device int integrate_surface_ray_portal(KernelGlobals kg,
                                            IntegratorState state,
                                            ccl_private ShaderData *sd,
                                            ccl_private const ShaderClosure *sc)
{
  ccl_private const RayPortalClosure *pc = (ccl_private const RayPortalClosure *)sc;

  float sum_sample_weight = 0.0f;
  for (int i = 0; i < sd->num_closure; i++) {
    ccl_private const ShaderClosure *sc = &sd->closure[i];

    if (CLOSURE_IS_BSDF_OR_BSSRDF(sc->type)) {
      sum_sample_weight += sc->sample_weight;
    }
  }
  if (sum_sample_weight <= 0.0f) {
    return LABEL_NONE;
  }

  if (len_squared(sd->P - pc->P) > 1e-9f) {
    /* if the ray origin is changed, unset the current object,
     * so we can potentially hit the same polygon again */
    INTEGRATOR_STATE_WRITE(state, isect, object) = OBJECT_NONE;
    INTEGRATOR_STATE_WRITE(state, ray, P) = pc->P;
  }
  else {
    INTEGRATOR_STATE_WRITE(state, ray, P) = integrate_surface_ray_offset(kg, sd, pc->P, pc->D);
  }
  INTEGRATOR_STATE_WRITE(state, ray, D) = pc->D;
  INTEGRATOR_STATE_WRITE(state, ray, tmin) = 0.0f;
  INTEGRATOR_STATE_WRITE(state, ray, tmax) = FLT_MAX;
#ifdef __RAY_DIFFERENTIALS__
  INTEGRATOR_STATE_WRITE(state, ray, dP) = differential_make_compact(sd->dP);
#endif

  const float pick_pdf = pc->sample_weight / sum_sample_weight;
  INTEGRATOR_STATE_WRITE(state, path, throughput) *= pc->weight / pick_pdf;

  int label = LABEL_TRANSMIT | LABEL_RAY_PORTAL;
  path_state_next(kg, state, label, sd->flag);

  return label;
}

/* Branch off a shadow path and initialize common part of it.
 * THe common is between the surface shading and configuration of a special shadow ray for the
 * shadow linking. */
ccl_device_inline IntegratorShadowState
integrate_direct_light_shadow_init_surface(KernelGlobals kg,
                                           IntegratorState state,
                                           ccl_private const Ray *ccl_restrict ray,
                                           const Spectrum bsdf_spectrum,
                                           const int light_group,
                                           const int mnee_vertex_count)
{

  /* Branch off shadow kernel. */
  IntegratorShadowState shadow_state = integrator_shadow_path_init(
      kg, state, DEVICE_KERNEL_INTEGRATOR_INTERSECT_SHADOW, false);

#ifdef __VOLUME__
  /* Copy volume stack and enter/exit volume. */
  integrator_state_copy_volume_stack_to_shadow(kg, shadow_state, state);
#endif

  /* Write shadow ray and associated state to global memory. */
  integrator_state_write_shadow_ray(shadow_state, ray);
  integrator_state_write_shadow_ray_self(kg, shadow_state, ray);

  /* Copy state from main path to shadow path. */
  const Spectrum unlit_throughput = INTEGRATOR_STATE(state, path, throughput);
  const Spectrum throughput = unlit_throughput * bsdf_spectrum;

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, render_pixel_index) = INTEGRATOR_STATE(
      state, path, render_pixel_index);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_offset) = INTEGRATOR_STATE(
      state, path, rng_offset);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_pixel) = INTEGRATOR_STATE(
      state, path, rng_pixel);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, sample) = INTEGRATOR_STATE(
      state, path, sample);

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transparent_bounce) = INTEGRATOR_STATE(
      state, path, transparent_bounce);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, glossy_bounce) = INTEGRATOR_STATE(
      state, path, glossy_bounce);

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, throughput) = throughput;

#ifdef __MNEE__
  if (mnee_vertex_count > 0) {
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transmission_bounce) =
        INTEGRATOR_STATE(state, path, transmission_bounce) + mnee_vertex_count - 1;
    INTEGRATOR_STATE_WRITE(shadow_state,
                           shadow_path,
                           diffuse_bounce) = INTEGRATOR_STATE(state, path, diffuse_bounce) + 1;
    INTEGRATOR_STATE_WRITE(shadow_state,
                           shadow_path,
                           bounce) = INTEGRATOR_STATE(state, path, bounce) + mnee_vertex_count;
  }
  else
#endif
  {
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transmission_bounce) = INTEGRATOR_STATE(
        state, path, transmission_bounce);
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, diffuse_bounce) = INTEGRATOR_STATE(
        state, path, diffuse_bounce);
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, bounce) = INTEGRATOR_STATE(
        state, path, bounce);
  }

  uint32_t shadow_flag = INTEGRATOR_STATE(state, path, flag);
  if (light_is_direct_illumination(state)) {
    shadow_flag |= PATH_RAY_SURFACE_PASS;
  }
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, flag) = shadow_flag;

  /* Write Light-group, +1 as light-group is int but we need to encode into a uint8_t. */
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, lightgroup) = light_group + 1;

#ifdef __PATH_GUIDING__
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, unlit_throughput) = unlit_throughput;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, path_segment) = INTEGRATOR_STATE(
      state, guiding, path_segment);
  INTEGRATOR_STATE(shadow_state, shadow_path, guiding_mis_weight) = 0.0f;
#endif

  return shadow_state;
}

template<bool is_direct_light>
ccl_device_inline void integrate_shadow_write_pass_diffuse_glossy(
    KernelGlobals kg,
    ConstIntegratorState state,
    IntegratorShadowState shadow_state,
    const ccl_private BsdfEval *radiance)
{
  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_PASSES) {
    PackedSpectrum pass_diffuse_weight;
    PackedSpectrum pass_glossy_weight;

    if (is_direct_light) {
      /* Direct light, use BSDFs at this bounce. */
      pass_diffuse_weight = PackedSpectrum(bsdf_eval_pass_diffuse_weight(radiance));
      pass_glossy_weight = PackedSpectrum(bsdf_eval_pass_glossy_weight(radiance));
    }
    else {
      /* Indirect bounce, use weights from earlier surface or volume bounce. */
      pass_diffuse_weight = INTEGRATOR_STATE(state, path, pass_diffuse_weight);
      pass_glossy_weight = INTEGRATOR_STATE(state, path, pass_glossy_weight);
    }

    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_diffuse_weight) = pass_diffuse_weight;
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, pass_glossy_weight) = pass_glossy_weight;
  }
}

ccl_device_inline void integrate_write_pass_diffuse_glossy(KernelGlobals kg,
                                                           IntegratorState state,
                                                           const ccl_private BsdfEval *radiance)
{
  if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_PASSES) {
    if (INTEGRATOR_STATE(state, path, bounce) == 0) {
      INTEGRATOR_STATE_WRITE(state, path, pass_diffuse_weight) = bsdf_eval_pass_diffuse_weight(
          radiance);
      INTEGRATOR_STATE_WRITE(state, path, pass_glossy_weight) = bsdf_eval_pass_glossy_weight(
          radiance);
    }
  }
}

template<bool is_direct_light>
ccl_device_inline void integrate_direct_light_create_shadow_path(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private const RNGState *rng_state,
    ccl_private const ShaderData *sd,
    ccl_private const LightSample *ls,
    ccl_private BsdfEval *radiance,
    const int mnee_vertex_count)
{
  /* Path termination. */
  const float terminate = path_state_rng_light_termination(kg, rng_state);
  if (light_sample_terminate(kg, radiance, terminate)) {
    return;
  }

  Ray ray ccl_optional_struct_init;
  /* Create shadow ray. */
  light_sample_to_surface_shadow_ray(kg, sd, ls, &ray);

  if (ray.self.object != OBJECT_NONE) {
    ray.P = integrate_surface_ray_offset(kg, sd, ray.P, ray.D);
  }

  /* Branch off shadow kernel. */
  IntegratorShadowState shadow_state = integrate_direct_light_shadow_init_surface(
      kg, state, &ray, bsdf_eval_sum(radiance), ls->group, mnee_vertex_count);

  const bool is_transmission = dot(ls->D, sd->N) < 0.0f;
  if (is_transmission) {
#ifdef __VOLUME__
    shadow_volume_stack_enter_exit(kg, shadow_state, sd);
#endif
  }

  integrate_shadow_write_pass_diffuse_glossy<is_direct_light>(kg, state, shadow_state, radiance);
}

/* TODO(weizhen): split function to direct/indirect illumination and rename this to NEE. */
/* Path tracing: sample point on light and evaluate light shader, then
 * queue shadow ray to be traced. */
template<uint node_feature_mask>
#if defined(__KERNEL_GPU__)
ccl_device_forceinline
#else
/* MSVC has very long compilation time (x20) if we force inline this function */
ccl_device
#endif
    void
    integrate_surface_direct_light(KernelGlobals kg,
                                   IntegratorState state,
                                   ccl_private ShaderData *sd,
                                   ccl_private const RNGState *rng_state,
                                   ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_INITIAL_RESAMPLING);
  /* Test if there is a light or BSDF that needs direct light. */
  if (!(kernel_data.integrator.use_direct_light && (sd->flag & SD_BSDF_HAS_EVAL))) {
    return;
  }

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);
  const uint bounce = INTEGRATOR_STATE(state, path, bounce);

  /* Use Resampled Importance Sampling for direct illumination based on Talbot, Justin F.
   * Importance resampling for global illumination. Brigham Young University, 2005. */
  /* TODO(weizhen): add MNEE back? */
  /* TODO(weizhen): the flag `use_spatial_resampling` can be replaced with `use_restir`. */
  const bool is_direct_light = light_is_direct_illumination(state);
  const bool use_ris = is_direct_light && (kernel_data.integrator.use_initial_resampling ||
                                           kernel_data.integrator.use_spatial_resampling);

  /* Sample position on a light. */
  Reservoir reservoir(kg, use_ris);
  const int max_samples = reservoir.max_samples;
  int samples_seen = 0;

  for (int i = 0; i < reservoir.num_light_samples; i++) {
    PROFILING_EVENT(PROFILING_RESTIR_LIGHT_RESAMPLING);
    LightSample ls ccl_optional_struct_init;
    BsdfEval radiance ccl_optional_struct_init;

    /* TODO(weizhen): does this result in correlated samples if different sample numbers are used
     * when using RIS? */
    const float3 rand_light = path_branched_rng_3D(
        kg, rng_state, i, reservoir.num_light_samples, PRNG_LIGHT);
    const float rand_pick = path_branched_rng_1D(
        kg, rng_state, samples_seen++, max_samples, PRNG_PICK);

    /* TODO(weizhen): use higher-level nodes in light tree, or use light tile as in the ReSTIR PT
     * paper. */
    if (!light_sample_from_position(kg,
                                    rand_light,
                                    sd->time,
                                    sd->P,
                                    sd->N,
                                    light_link_receiver_nee(kg, sd),
                                    sd->flag,
                                    bounce,
                                    path_flag,
                                    &ls))
    {
      continue;
    }

    kernel_assert(ls.pdf != 0.0f);

    const bool is_transmission = dot(ls.D, sd->N) < 0.0f;

    if (ls.prim != PRIM_NONE && ls.prim == sd->prim && ls.object == sd->object) {
      /* Skip self intersection if light direction lies in the same hemisphere as the geometric
       * normal. */
      if (dot(ls.D, is_transmission ? -sd->Ng : sd->Ng) > 0.0f) {
        continue;
      }
    }

    /* Evaluate light shader. */
    const bool check_visibility = use_ris && kernel_data.integrator.restir_unbiased;
    const float bsdf_pdf = radiance_eval(kg, state, sd, &ls, &radiance, check_visibility);

    PROFILING_EVENT(PROFILING_RESTIR_RESERVOIR);
    reservoir.add_light_sample(ls, radiance, bsdf_pdf, rand_pick);
  }

  /* If `use_ris`, draw BSDF samples in #integrate_surface_bsdf_bssrdf_bounce(). */
  for (int i = 0; i < reservoir.num_bsdf_samples * use_ris; i++) {
    PROFILING_EVENT(PROFILING_RESTIR_BSDF_RESAMPLING);
    kernel_assert(bounce == 0);

    LightSample ls ccl_optional_struct_init;
    BsdfEval radiance ccl_optional_struct_init;
    float3 bsdf_wo ccl_optional_struct_init;
    float bsdf_pdf = 0.0f;

    float2 bsdf_sampled_roughness = make_float2(1.0f, 1.0f);
    float bsdf_eta = 1.0f;

    float3 rand_bsdf = path_branched_rng_3D(
        kg, rng_state, i, reservoir.num_bsdf_samples, PRNG_SURFACE_BSDF);
    const float rand_pick = path_branched_rng_1D(
        kg, rng_state, samples_seen++, max_samples, PRNG_PICK);

    ccl_private const ShaderClosure *sc = surface_shader_bsdf_bssrdf_pick(sd, &rand_bsdf);

    /* TODO(weizhen): what to do with subsurface? */
    surface_shader_bsdf_sample_closure(kg,
                                       sd,
                                       sc,
                                       path_flag,
                                       rand_bsdf,
                                       &radiance,
                                       &bsdf_wo,
                                       &bsdf_pdf,
                                       &bsdf_sampled_roughness,
                                       &bsdf_eta);

    if (bsdf_pdf == 0.0f || bsdf_eval_is_zero(&radiance)) {
      continue;
    }

    /* Set up ray. */
    Ray ray ccl_optional_struct_init;
    ray.D = normalize(bsdf_wo);
    ray.P = integrate_surface_ray_offset(kg, sd, sd->P, ray.D);
    ray.tmin = 0.0f;
    ray.tmax = FLT_MAX;
    ray.time = INTEGRATOR_STATE(state, ray, time);

    /* Scene Intersection. */
    Intersection isect ccl_optional_struct_init;
    isect.object = OBJECT_NONE;
    isect.prim = PRIM_NONE;
    ray.self.object = sd->object;
    ray.self.prim = sd->prim;
    ray.self.light_object = OBJECT_NONE;
    ray.self.light_prim = PRIM_NONE;
    ray.self.light = LAMP_NONE;

    /* Potential intersection of emissive triangles. */
    /* TODO(weizhen): is it worth to set up the next `path_flag`? */
    const uint32_t next_path_flag = PATH_RAY_ALL_VISIBILITY & ~PATH_RAY_CAMERA;

    bool hit = scene_intersect(kg, &ray, next_path_flag, &isect);

    if (!hit) {
      isect.prim = PRIM_NONE;
    }

    /* Potential intersection of spot, point, and area lights. */
    /* TODO(weizhen): delay shadow ray tracing, or delay this whole BSDF sampling? */
    /* FIXME(weizhen): this has artefacts when `num_hits > 1` due to transparent lights. */
    hit = lights_intersect(
              kg, state, &ray, &isect, sd->prim, sd->object, sd->type, next_path_flag) ||
          hit;

    Spectrum L = zero_spectrum();
    /* TODO(weizhen): volume? */
    if (hit) {
      /* TODO(weizhen): is this the flag for `sc`? */
      const uint32_t transmission_flag = (sd->flag & SD_BSDF_HAS_TRANSMISSION) ?
                                             PATH_RAY_MIS_HAD_TRANSMISSION :
                                             ~PATH_RAY_MIS_HAD_TRANSMISSION;

      if (isect.type & PRIMITIVE_LAMP) {
        if (!light_sample_from_intersection(
                kg, &isect, ray.P, ray.D, sd->N, transmission_flag, &ls))
        {
          continue;
        }
        ShaderDataTinyStorage emission_sd_storage;
        ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage);

        L = light_sample_shader_eval(kg, state, emission_sd, &ls, sd->time);

        /* TODO(weizhen): maybe merge with `light_sample_mis_weight_forward_lamp` */
        /* Light selection pdf. */
#ifdef __LIGHT_TREE__
        if (kernel_data.integrator.use_light_tree) {
          ls.pdf *= light_tree_pdf<false>(kg,
                                          ray.P,
                                          sd->N,
                                          0.0f,
                                          transmission_flag,
                                          0,
                                          kernel_data_fetch(light_to_tree, ls.lamp),
                                          light_link_receiver_forward(kg, state));
        }
        else
#endif
        {
          ls.pdf *= light_distribution_pdf_lamp(kg);
        }
      }
      else {
        /* TODO(weizhen): this is too costly. */
        ShaderData emission_sd;
        shader_setup_from_ray(kg, &emission_sd, &ray, &isect);

        /* TODO(weizhen):  can't use `nullptr` with aov. */
        surface_shader_eval<node_feature_mask>(
            kg, state, &emission_sd, nullptr, transmission_flag);

        /* Emissive triangle. */
        if (emission_sd.flag & SD_EMISSION) {
          /* Evaluate emissive closure. */
          L = surface_shader_emission(&emission_sd);

          triangle_light_sample_from_intersection(
              kg, &isect, &emission_sd, ray.P, ray.D, ray.time, &ls);

          /* TODO(weizhen): PATH_RAY_MIS_SKIP?  */
          const bool has_mis = (emission_sd.type & PRIMITIVE_TRIANGLE) &&
                               (emission_sd.flag &
                                ((emission_sd.flag & SD_BACKFACING) ? SD_MIS_BACK : SD_MIS_FRONT));

          if (has_mis) {
            ls.pdf = triangle_light_pdf(kg, &emission_sd, isect.t);

            /* TODO(weizhen): maybe merge with `light_sample_mis_weight_forward_surface` */
            /* Light selection pdf. */
#ifdef __LIGHT_TREE__
            if (kernel_data.integrator.use_light_tree) {
              uint lookup_offset = kernel_data_fetch(object_lookup_offset, emission_sd.object);
              uint prim_offset = kernel_data_fetch(object_prim_offset, emission_sd.object);
              uint triangle = kernel_data_fetch(triangle_to_tree,
                                                emission_sd.prim - prim_offset + lookup_offset);

              ls.pdf *= light_tree_pdf<false>(kg,
                                              sd->P,
                                              sd->N,
                                              0.0f,
                                              transmission_flag,
                                              emission_sd.object,
                                              triangle,
                                              light_link_receiver_forward(kg, state));
            }
#endif
          }
          else {
            ls.pdf = 0.0f;
          }
        }
      }
    }
    else {
      /* ShaderDataTinyStorage emission_sd_storage; */
      /* ccl_private ShaderData *emission_sd = AS_SHADER_DATA(&emission_sd_storage); */

      /* /\* Distant light. *\/ */
      /* for (int lamp = 0; lamp < kernel_data.integrator.num_lights; lamp++) { */
      /*   if (distant_light_sample_from_intersection(kg, ray.D, lamp, &ls)) { */
      /*     /\* TODO(weizhen): how to do distant light pdf? *\/ */
      /*     L += light_sample_shader_eval(kg, state, emission_sd, &ls, sd->time); */
      /*   } */
      /* } */

      /* /\* Background light. *\/ */
      /* const int background_shader = kernel_data.background.surface_shader; */

      /* /\* Use fast constant background color if available. *\/ */
      /* Spectrum background_eval = zero_spectrum(); */
      /* if (surface_shader_constant_emission(kg, background_shader, &background_eval)) { */
      /*   L = background_eval; */
      /* } */

      /* /\* Evaluate background shader. *\/ */
      /* shader_setup_from_background(kg, emission_sd, ray.P, ray.D, sd->time); */
      /* TODO(weizhen): can't use `nullptr` with aov. */
      /* surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_BACKGROUND>( */
      /*     kg, state, emission_sd, nullptr, PATH_RAY_EMISSION); */
      /* L = surface_shader_background(emission_sd); */
      /* /\* TODO(weizhen): how to do background light pdf? *\/ */
    }

    if (is_zero(L)) {
      continue;
    }
    const float geometry_term = ls.jacobian_solid_angle_to_area();
    bsdf_eval_mul(&radiance, L * geometry_term);
    PROFILING_EVENT(PROFILING_RESTIR_RESERVOIR);
    reservoir.add_bsdf_sample(ls, radiance, bsdf_pdf, rand_pick);
  }

  PROFILING_EVENT(PROFILING_RESTIR_INITIAL_RESAMPLING);

  kernel_assert(samples_seen <= max_samples);

  if (use_ris) {
    /* Write to reservoir and trace shadow ray later. */
    PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
    reservoir.finalize();
    film_write_pass_reservoir(kg, state, &reservoir, render_buffer);
  }
  else if (reservoir.finalize()) {
    bsdf_eval_mul(&reservoir.radiance, reservoir.total_weight);

    int mnee_vertex_count = 0;
    integrate_direct_light_create_shadow_path<false>(
        kg, state, rng_state, sd, &reservoir.ls, &reservoir.radiance, mnee_vertex_count);
  }
}

/* Path tracing: bounce off or through surface with new direction. */
ccl_device_forceinline int integrate_surface_bsdf_bssrdf_bounce(
    KernelGlobals kg,
    IntegratorState state,
    ccl_private ShaderData *sd,
    ccl_private const RNGState *rng_state)
{
  /* Sample BSDF or BSSRDF. */
  if (!(sd->flag & (SD_BSDF | SD_BSSRDF))) {
    return LABEL_NONE;
  }

  float3 rand_bsdf = path_state_rng_3D(kg, rng_state, PRNG_SURFACE_BSDF);
  ccl_private const ShaderClosure *sc = surface_shader_bsdf_bssrdf_pick(sd, &rand_bsdf);

#ifdef __SUBSURFACE__
  /* BSSRDF closure, we schedule subsurface intersection kernel. */
  if (CLOSURE_IS_BSSRDF(sc->type)) {
    return subsurface_bounce(kg, state, sd, sc);
  }
#endif
  if (CLOSURE_IS_RAY_PORTAL(sc->type)) {
    return integrate_surface_ray_portal(kg, state, sd, sc);
  }

  /* BSDF closure, sample direction. */
  float bsdf_pdf = 0.0f, unguided_bsdf_pdf = 0.0f;
  BsdfEval bsdf_eval ccl_optional_struct_init;
  float3 bsdf_wo ccl_optional_struct_init;
  int label;

  float2 bsdf_sampled_roughness = make_float2(1.0f, 1.0f);
  float bsdf_eta = 1.0f;
  float mis_pdf = 1.0f;

#if defined(__PATH_GUIDING__) && PATH_GUIDING_LEVEL >= 4
  if (kernel_data.integrator.use_surface_guiding) {
    label = surface_shader_bsdf_guided_sample_closure(kg,
                                                      state,
                                                      sd,
                                                      sc,
                                                      rand_bsdf,
                                                      &bsdf_eval,
                                                      &bsdf_wo,
                                                      &bsdf_pdf,
                                                      &mis_pdf,
                                                      &unguided_bsdf_pdf,
                                                      &bsdf_sampled_roughness,
                                                      &bsdf_eta,
                                                      rng_state);

    if (bsdf_pdf == 0.0f || bsdf_eval_is_zero(&bsdf_eval)) {
      return LABEL_NONE;
    }

    INTEGRATOR_STATE_WRITE(state, path, unguided_throughput) *= bsdf_pdf / unguided_bsdf_pdf;
  }
  else
#endif
  {
    label = surface_shader_bsdf_sample_closure(kg,
                                               sd,
                                               sc,
                                               INTEGRATOR_STATE(state, path, flag),
                                               rand_bsdf,
                                               &bsdf_eval,
                                               &bsdf_wo,
                                               &bsdf_pdf,
                                               &bsdf_sampled_roughness,
                                               &bsdf_eta);

    if (bsdf_pdf == 0.0f || bsdf_eval_is_zero(&bsdf_eval)) {
      return LABEL_NONE;
    }
    mis_pdf = bsdf_pdf;
    unguided_bsdf_pdf = bsdf_pdf;
  }

  if (label & LABEL_TRANSPARENT) {
    /* Only need to modify start distance for transparent. */
    INTEGRATOR_STATE_WRITE(state, ray, tmin) = intersection_t_offset(sd->ray_length);
  }
  else {
    /* Setup ray with changed origin and direction. */
    const float3 D = normalize(bsdf_wo);
    INTEGRATOR_STATE_WRITE(state, ray, P) = integrate_surface_ray_offset(kg, sd, sd->P, D);
    INTEGRATOR_STATE_WRITE(state, ray, D) = D;
    INTEGRATOR_STATE_WRITE(state, ray, tmin) = 0.0f;
    INTEGRATOR_STATE_WRITE(state, ray, tmax) = FLT_MAX;
#ifdef __RAY_DIFFERENTIALS__
    INTEGRATOR_STATE_WRITE(state, ray, dP) = differential_make_compact(sd->dP);
#endif
  }

  /* Update throughput. */
  const Spectrum bsdf_weight = bsdf_eval_sum(&bsdf_eval) / bsdf_pdf;
  INTEGRATOR_STATE_WRITE(state, path, throughput) *= bsdf_weight;

  integrate_write_pass_diffuse_glossy(kg, state, &bsdf_eval);

  /* Update path state */
  if (!(label & LABEL_TRANSPARENT)) {
    INTEGRATOR_STATE_WRITE(state, path, mis_ray_pdf) = mis_pdf;
    INTEGRATOR_STATE_WRITE(state, path, mis_origin_n) = sd->N;
    INTEGRATOR_STATE_WRITE(state, path, min_ray_pdf) = fminf(
        unguided_bsdf_pdf, INTEGRATOR_STATE(state, path, min_ray_pdf));

#ifdef __LIGHT_LINKING__
    if (kernel_data.kernel_features & KERNEL_FEATURE_LIGHT_LINKING) {
      INTEGRATOR_STATE_WRITE(state, path, mis_ray_object) = sd->object;
    }
#endif
  }

  path_state_next(kg, state, label, sd->flag);

  guiding_record_surface_bounce(kg,
                                state,
                                sd,
                                bsdf_weight,
                                bsdf_pdf,
                                sd->N,
                                normalize(bsdf_wo),
                                bsdf_sampled_roughness,
                                bsdf_eta);

  return label;
}

#ifdef __VOLUME__
ccl_device_forceinline int integrate_surface_volume_only_bounce(IntegratorState state,
                                                                ccl_private ShaderData *sd)
{
  if (!path_state_volume_next(state)) {
    return LABEL_NONE;
  }

  /* Only modify start distance. */
  INTEGRATOR_STATE_WRITE(state, ray, tmin) = intersection_t_offset(sd->ray_length);

  return LABEL_TRANSMIT | LABEL_TRANSPARENT;
}
#endif

ccl_device_forceinline bool integrate_surface_terminate(IntegratorState state,
                                                        const uint32_t path_flag)
{
  const float continuation_probability = (path_flag & PATH_RAY_TERMINATE_ON_NEXT_SURFACE) ?
                                             0.0f :
                                             INTEGRATOR_STATE(
                                                 state, path, continuation_probability);
  if (continuation_probability == 0.0f) {
    return true;
  }
  else if (continuation_probability != 1.0f) {
    INTEGRATOR_STATE_WRITE(state, path, throughput) /= continuation_probability;
  }

  return false;
}

#if defined(__AO__)
ccl_device_forceinline void integrate_surface_ao(KernelGlobals kg,
                                                 IntegratorState state,
                                                 ccl_private const ShaderData *ccl_restrict sd,
                                                 ccl_private const RNGState *ccl_restrict
                                                     rng_state)
{
  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  if (!(kernel_data.kernel_features & KERNEL_FEATURE_AO_ADDITIVE) &&
      !(path_flag & PATH_RAY_CAMERA))
  {
    return;
  }

  /* Skip AO for paths that were split off for shadow catchers to avoid double-counting. */
  if (path_flag & PATH_RAY_SHADOW_CATCHER_PASS) {
    return;
  }

  const float2 rand_bsdf = path_state_rng_2D(kg, rng_state, PRNG_SURFACE_BSDF);

  float3 ao_N;
  const Spectrum ao_weight = surface_shader_ao(
      kg, sd, kernel_data.integrator.ao_additive_factor, &ao_N);

  float3 ao_D;
  float ao_pdf;
  sample_cos_hemisphere(ao_N, rand_bsdf, &ao_D, &ao_pdf);

  bool skip_self = true;

  Ray ray ccl_optional_struct_init;
  ray.P = shadow_ray_offset(kg, sd, ao_D, &skip_self);
  ray.D = ao_D;
  if (skip_self) {
    ray.P = integrate_surface_ray_offset(kg, sd, ray.P, ray.D);
  }
  ray.tmin = 0.0f;
  ray.tmax = kernel_data.integrator.ao_bounces_distance;
  ray.time = sd->time;
  ray.self.object = (skip_self) ? sd->object : OBJECT_NONE;
  ray.self.prim = (skip_self) ? sd->prim : PRIM_NONE;
  ray.self.light_object = OBJECT_NONE;
  ray.self.light_prim = PRIM_NONE;
  ray.self.light = LAMP_NONE;
  ray.dP = differential_zero_compact();
  ray.dD = differential_zero_compact();

  /* Branch off shadow kernel. */
  IntegratorShadowState shadow_state = integrator_shadow_path_init(
      kg, state, DEVICE_KERNEL_INTEGRATOR_INTERSECT_SHADOW, true);

#  ifdef __VOLUME__
  /* Copy volume stack and enter/exit volume. */
  integrator_state_copy_volume_stack_to_shadow(kg, shadow_state, state);
#  endif

  /* Write shadow ray and associated state to global memory. */
  integrator_state_write_shadow_ray(shadow_state, &ray);
  integrator_state_write_shadow_ray_self(kg, shadow_state, &ray);

  /* Copy state from main path to shadow path. */
  const uint16_t bounce = INTEGRATOR_STATE(state, path, bounce);
  const uint16_t transparent_bounce = INTEGRATOR_STATE(state, path, transparent_bounce);
  uint32_t shadow_flag = INTEGRATOR_STATE(state, path, flag) | PATH_RAY_SHADOW_FOR_AO;
  const Spectrum throughput = INTEGRATOR_STATE(state, path, throughput) *
                              surface_shader_alpha(kg, sd);

  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, render_pixel_index) = INTEGRATOR_STATE(
      state, path, render_pixel_index);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_offset) = INTEGRATOR_STATE(
      state, path, rng_offset);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, rng_pixel) = INTEGRATOR_STATE(
      state, path, rng_pixel);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, sample) = INTEGRATOR_STATE(
      state, path, sample);
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, flag) = shadow_flag;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, bounce) = bounce;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, transparent_bounce) = transparent_bounce;
  INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, throughput) = throughput;

  if (kernel_data.kernel_features & KERNEL_FEATURE_AO_ADDITIVE) {
    INTEGRATOR_STATE_WRITE(shadow_state, shadow_path, unshadowed_throughput) = ao_weight;
  }
}
#endif /* defined(__AO__) */

template<uint node_feature_mask>
ccl_device int integrate_surface(KernelGlobals kg,
                                 IntegratorState state,
                                 ccl_global float *ccl_restrict render_buffer)

{
  PROFILING_INIT_FOR_SHADER(kg, PROFILING_SHADE_SURFACE_SETUP);

  /* Setup shader data. */
  ShaderData sd;
  integrate_surface_shader_setup(kg, state, &sd);
  PROFILING_SHADER(sd.object, sd.shader);

  int continue_path_label = 0;

  const uint32_t path_flag = INTEGRATOR_STATE(state, path, flag);

  /* Skip most work for volume bounding surface. */
#ifdef __VOLUME__
  if (!(sd.flag & SD_HAS_ONLY_VOLUME)) {
#endif
    guiding_record_surface_segment(kg, state, &sd);

#ifdef __SUBSURFACE__
    /* Can skip shader evaluation for BSSRDF exit point without bump mapping. */
    if (!(path_flag & PATH_RAY_SUBSURFACE) || ((sd.flag & SD_HAS_BSSRDF_BUMP)))
#endif
    {
      /* Evaluate shader. */
      PROFILING_EVENT(PROFILING_SHADE_SURFACE_EVAL);
      surface_shader_eval<node_feature_mask>(kg, state, &sd, render_buffer, path_flag);

      /* Initialize additional RNG for BSDFs. */
      if (sd.flag & SD_BSDF_NEEDS_LCG) {
        sd.lcg_state = lcg_state_init(INTEGRATOR_STATE(state, path, rng_pixel),
                                      INTEGRATOR_STATE(state, path, rng_offset),
                                      INTEGRATOR_STATE(state, path, sample),
                                      0xb4bc3953);
      }
    }

#ifdef __SUBSURFACE__
    if (path_flag & PATH_RAY_SUBSURFACE) {
      /* When coming from inside subsurface scattering, setup a diffuse
       * closure to perform lighting at the exit point. */
      subsurface_shader_data_setup(kg, state, &sd, path_flag);
      INTEGRATOR_STATE_WRITE(state, path, flag) &= ~PATH_RAY_SUBSURFACE;
    }
    else
#endif
    {
      /* Filter closures. */
      surface_shader_prepare_closures(kg, state, &sd, path_flag);

      /* Evaluate holdout. */
      if (!integrate_surface_holdout(kg, state, &sd, render_buffer)) {
        return LABEL_NONE;
      }

      /* Write emission. */
      if (sd.flag & SD_EMISSION) {
        /* TODO(weizhen): `min_bounce` is only for debugging, revert after finishing the branch. */
        if (INTEGRATOR_STATE(state, path, bounce) >= kernel_data.integrator.min_bounce) {
          integrate_surface_emission(kg, state, &sd, render_buffer);
        }
      }

      /* Perform path termination. Most paths have already been terminated in
       * the intersect_closest kernel, this is just for emission and for dividing
       * throughput by the probability at the right moment.
       *
       * Also ensure we don't do it twice for SSS at both the entry and exit point. */
      if (integrate_surface_terminate(state, path_flag)) {
        return LABEL_NONE;
      }

      /* Write render passes. */
#ifdef __PASSES__
      PROFILING_EVENT(PROFILING_SHADE_SURFACE_PASSES);
      film_write_data_passes(kg, state, &sd, render_buffer);
#endif

#ifdef __DENOISING_FEATURES__
      film_write_denoising_features_surface(kg, state, &sd, render_buffer);
#endif
    }

    /* Load random number state. */
    RNGState rng_state;
    path_state_rng_load(state, &rng_state);

#if defined(__PATH_GUIDING__) && PATH_GUIDING_LEVEL >= 4
    surface_shader_prepare_guiding(kg, state, &sd, &rng_state);
    guiding_write_debug_passes(kg, state, &sd, render_buffer);
#endif
    /* Direct light. */
    PROFILING_EVENT(PROFILING_SHADE_SURFACE_DIRECT_LIGHT);
    if (INTEGRATOR_STATE(state, path, bounce) + 1 >= kernel_data.integrator.min_bounce) {
      /* TODO(weizhen): `min_bounce` is only for debugging, revert after finishing the branch. */
      integrate_surface_direct_light<node_feature_mask>(kg, state, &sd, &rng_state, render_buffer);
    }

#if defined(__AO__)
    /* Ambient occlusion pass. */
    if (kernel_data.kernel_features & KERNEL_FEATURE_AO) {
      PROFILING_EVENT(PROFILING_SHADE_SURFACE_AO);
      integrate_surface_ao(kg, state, &sd, &rng_state);
    }
#endif

    PROFILING_EVENT(PROFILING_SHADE_SURFACE_INDIRECT_LIGHT);
    continue_path_label = integrate_surface_bsdf_bssrdf_bounce(kg, state, &sd, &rng_state);
#ifdef __VOLUME__
  }
  else {
    if (integrate_surface_terminate(state, path_flag)) {
      return LABEL_NONE;
    }

    PROFILING_EVENT(PROFILING_SHADE_SURFACE_INDIRECT_LIGHT);
    continue_path_label = integrate_surface_volume_only_bounce(state, &sd);
  }

  if (continue_path_label & LABEL_TRANSMIT) {
    /* Enter/Exit volume. */
    volume_stack_enter_exit(kg, state, &sd);
  }
#endif

  return continue_path_label;
}

template<DeviceKernel current_kernel>
ccl_device_forceinline void integrator_shade_surface_next_kernel(KernelGlobals kg,
                                                                 IntegratorState state)
{
  if (INTEGRATOR_STATE(state, path, flag) & PATH_RAY_SUBSURFACE) {
    integrator_path_next(kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_INTERSECT_SUBSURFACE);
  }
  else {
    kernel_assert(INTEGRATOR_STATE(state, ray, tmax) != 0.0f);
    integrator_path_next(kg, state, current_kernel, DEVICE_KERNEL_INTEGRATOR_INTERSECT_CLOSEST);
  }
}

template<uint node_feature_mask = KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE,
         DeviceKernel current_kernel = DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE>
ccl_device_forceinline void integrator_shade_surface(KernelGlobals kg,
                                                     IntegratorState state,
                                                     ccl_global float *ccl_restrict render_buffer)
{
  const int continue_path_label = integrate_surface<node_feature_mask>(kg, state, render_buffer);
  if (continue_path_label == LABEL_NONE) {
    integrator_path_terminate(kg, state, current_kernel);
    return;
  }

#ifdef __SHADOW_LINKING__
  /* No need to cast shadow linking rays at a transparent bounce: the lights will be accumulated
   * via the main path in this case. */
  if ((continue_path_label & LABEL_TRANSPARENT) == 0) {
    if (shadow_linking_schedule_intersection_kernel<current_kernel>(kg, state)) {
      return;
    }
  }
#endif

  integrator_shade_surface_next_kernel<current_kernel>(kg, state);
}

ccl_device_forceinline void integrator_shade_surface_raytrace(
    KernelGlobals kg, IntegratorState state, ccl_global float *ccl_restrict render_buffer)
{
  integrator_shade_surface<KERNEL_FEATURE_NODE_MASK_SURFACE,
                           DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_RAYTRACE>(
      kg, state, render_buffer);
}

ccl_device_forceinline void integrator_shade_surface_mnee(
    KernelGlobals kg, IntegratorState state, ccl_global float *ccl_restrict render_buffer)
{
  integrator_shade_surface<(KERNEL_FEATURE_NODE_MASK_SURFACE & ~KERNEL_FEATURE_NODE_RAYTRACE) |
                               KERNEL_FEATURE_MNEE,
                           DEVICE_KERNEL_INTEGRATOR_SHADE_SURFACE_MNEE>(kg, state, render_buffer);
}

CCL_NAMESPACE_END
