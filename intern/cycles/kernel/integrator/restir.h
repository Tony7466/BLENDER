#ifndef RESTIR_H_
#define RESTIR_H_

#include "kernel/integrator/reservoir.h"
#include "kernel/integrator/state.h"
#include "kernel/util/profiling.h"

CCL_NAMESPACE_BEGIN

struct SpatialResampling {
  ccl_global const KernelWorkTile *tile;
  int radius;

  SpatialResampling(ccl_global const KernelWorkTile *ccl_restrict tile, const int radius)
  {
    this->tile = tile;
    this->radius = radius;
  }

  uint get_render_pixel_index(const int x, const int y) const
  {
    return (uint)tile->offset + x + y * tile->stride;
  }

  /* Get neighbors in a square. */
  uint3 get_neighbor(const float2 rand) const
  {
    const float2 offset = floor(rand * radius + 0.5f);

    const int x = tile->x + (int)offset.x;
    const int y = tile->y + (int)offset.y;

    if (x < tile->min_x || x > tile->max_x || y < tile->min_y || y > tile->max_y) {
      /* TODO(weizhen): this is wasting samples, but can we do better? For example overscan? */
      /* Out of bound. */
      return make_uint3(x, y, -1);
    }

    return make_uint3(x, y, get_render_pixel_index(x, y));
  }

  static bool is_valid_neighbor(const ccl_private ShaderData *sd,
                                const ccl_private ShaderData *neighbor_sd)
  {
    /* TODO(weizhen): find a good criterion, for example the distance to the normal plane of sd. */
    return (sd->object == neighbor_sd->object) && (sd->type == neighbor_sd->type);
  }
};

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private Reservoir *reservoir,
                                                          const ccl_global float *buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  int i = 0;
  /* TODO(weizhen): this works for diffuse surfaces. For specular, probably `sd->wi` is needed
   * instead. */
  /* TODO(weizhen): compress the data. */
  reservoir->ls.object = (int)buffer[i++];
  if (reservoir->ls.object == OBJECT_NONE) {
    /* Analytic light. */
    reservoir->ls.lamp = (int)buffer[i++];
  }
  else {
    /* Mesh light. */
    reservoir->ls.prim = (int)buffer[i++];
  }

  reservoir->ls.u = buffer[i++];
  reservoir->ls.v = buffer[i++];

  reservoir->total_weight = buffer[i++];
}

/* TODO(weizhen): better to pack in another pass. */
ccl_device_inline void integrator_restir_unpack_shader(ccl_private ShaderData *sd,
                                                       ccl_private uint32_t *path_flag,
                                                       const ccl_global float *buffer)
{
  int i = 0;

  *path_flag = (uint32_t)buffer[i++];

  /* TODO(weizhen): compress the data. */
  sd->u = buffer[i++];
  sd->v = buffer[i++];
  sd->type = (uint)buffer[i++];
  sd->object = (int)buffer[i++];
  sd->prim = (int)buffer[i++];
  /* TODO(weizhen): only Huang Hair BSDF needs LCG now. Can we find a way to remove the usage? */
  /* TODO(weizhen): check if we can compute `lcg_state` after we support curve. */
  sd->lcg_state = (uint)buffer[i];

  /* (TODO): do I need to write to this? */
  // sd->closure_transparent_extinction;
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       ccl_private SpatialReservoir *reservoir,
                                                       const uint32_t render_pixel_index,
                                                       const ccl_global float *ccl_restrict
                                                           render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
  ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                   kernel_data.film.pass_surface_data;
  integrator_restir_unpack_shader(&reservoir->sd, &reservoir->path_flag, buffer);
}

ccl_device_inline void integrator_restir_unpack_shader(KernelGlobals kg,
                                                       IntegratorState state,
                                                       ccl_private SpatialReservoir *reservoir,
                                                       ccl_global float *ccl_restrict
                                                           render_buffer)
{
  const uint32_t render_pixel_index = INTEGRATOR_STATE(state, path, render_pixel_index);
  integrator_restir_unpack_shader(kg, reservoir, render_pixel_index, render_buffer);
}

ccl_device_inline void integrator_restir_unpack_reservoir(KernelGlobals kg,
                                                          ccl_private SpatialReservoir *reservoir,
                                                          const uint32_t render_pixel_index,
                                                          const ccl_global float *ccl_restrict
                                                              render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_RESERVOIR_PASSES);
  if (kernel_data.film.pass_restir_reservoir != PASS_UNUSED) {
    const uint64_t render_buffer_offset = render_pixel_index * kernel_data.film.pass_stride;
    ccl_global const float *buffer = render_buffer + render_buffer_offset +
                                     kernel_data.film.pass_restir_reservoir;
    integrator_restir_unpack_reservoir(kg, &reservoir->reservoir, buffer);
  }
  integrator_restir_unpack_shader(kg, reservoir, render_pixel_index, render_buffer);
}

ccl_device_inline void ray_setup(KernelGlobals kg,
                                 ConstIntegratorState state,
                                 ccl_private Ray *ccl_restrict ray,
                                 const uint3 render_pixel_index)
{
  if (render_pixel_index.z == -1) {
    integrator_state_read_ray(state, ray);
  }
  else {
    /* Setup neighboring ray from the camera. */
    const int x = render_pixel_index.x;
    const int y = render_pixel_index.y;

    /* TODO(weizhen): for adaptive sampling maybe the sample is not the same? */
    const int sample = INTEGRATOR_STATE(state, path, sample);
    const uint rng_pixel = path_rng_pixel_init(kg, sample, x, y);
    integrate_camera_sample(kg, sample, x, y, rng_pixel, ray);
  }
}

ccl_device_forceinline void shader_data_setup_from_restir(KernelGlobals kg,
                                                          ConstIntegratorState state,
                                                          ccl_private SpatialReservoir *reservoir,
                                                          ccl_global float *ccl_restrict
                                                              render_buffer)
{
  Ray ray;
  ray_setup(kg, state, &ray, reservoir->pixel_index);

  PROFILING_INIT(kg, PROFILING_RESTIR_SURFACE_DATA_SETUP);
  shader_setup_from_restir(kg, &ray, &reservoir->sd);

  PROFILING_EVENT(PROFILING_RESTIR_SHADER_SETUP);
  /* TODO(weizhen): what features are needed here? Is this the right state? */
  surface_shader_eval<KERNEL_FEATURE_NODE_MASK_SURFACE_SHADOW>(
      kg, state, &reservoir->sd, render_buffer, reservoir->path_flag);
  /* TODO(weizhen): do we call `surface_shader_prepare_closures()` here? */
}

ccl_device_inline bool restir_get_neighbor(KernelGlobals kg,
                                           const ccl_private SpatialResampling *resampling,
                                           ccl_private SpatialReservoir *current,
                                           ccl_private SpatialReservoir *neighbor,
                                           const int id,
                                           const float3 rand,
                                           ccl_global float *ccl_restrict render_buffer)
{
  /* Always put the current sample in the reservoir. */
  const float2 rand_index = (id == 0) ? zero_float2() : float3_to_float2(rand) * 2.0f - 1.0f;
  const uint3 pixel_index = resampling->get_neighbor(rand_index);

  if (pixel_index.z == -1) {
    return false;
  }

  neighbor->pixel_index = pixel_index;
  integrator_restir_unpack_reservoir(kg, neighbor, pixel_index.z, render_buffer);

  return resampling->is_valid_neighbor(&current->sd, &neighbor->sd);
}

ccl_device_inline bool spatial_sample_streaming(KernelGlobals kg,
                                                IntegratorState state,
                                                const ccl_private SpatialResampling *resampling,
                                                ccl_private SpatialReservoir *current,
                                                const ccl_private RNGState *rng_state,
                                                const int samples,
                                                const bool visibility,
                                                ccl_global float *ccl_restrict render_buffer)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_SPATIAL_RESAMPLING);

  for (int i = 0; i < samples; i++) {
    PROFILING_EVENT(PROFILING_RESTIR_SPATIAL_RESAMPLING);
    const float3 rand = path_branched_rng_3D(kg, rng_state, i, samples, PRNG_SPATIAL_RESAMPLING);

    SpatialReservoir neighbor;
    if (!restir_get_neighbor(kg, resampling, current, &neighbor, i, rand, render_buffer)) {
      continue;
    }

    if (neighbor.is_empty()) {
      continue;
    }

    /* Evaluate neighbor sample from the current shading point. */
    if (!light_sample_from_uv(kg, &current->sd, current->path_flag, &neighbor.reservoir.ls)) {
      continue;
    }
    radiance_eval(
        kg, state, &current->sd, &neighbor.reservoir.ls, &neighbor.reservoir.radiance, visibility);
    PROFILING_EVENT(PROFILING_RESTIR_RESERVOIR);
    if (sample_copy_direction(kg, neighbor.reservoir)) {
      /* Jacobian for non-identity shift. */
      neighbor.reservoir.total_weight *= neighbor.reservoir.ls.jacobian_solid_angle_to_area();
    }
    current->add_reservoir(neighbor, rand.z);
  }

  PROFILING_EVENT(PROFILING_RESTIR_SPATIAL_RESAMPLING);
  if (!current->reservoir.finalize()) {
    return false;
  }

  LightSample picked_ls = current->reservoir.ls;

  /* Loop over neighborhood again to determine valid samples. Skip the current pixel, because if
   * the reservoir is not empty it must be valid. */
  int valid_neighbors = 1;
  for (int i = 1; i < samples; i++) {
    const float3 rand = path_branched_rng_3D(kg, rng_state, i, samples, PRNG_SPATIAL_RESAMPLING);

    SpatialReservoir neighbor;
    if (!restir_get_neighbor(kg, resampling, current, &neighbor, i, rand, render_buffer)) {
      continue;
    }

    shader_data_setup_from_restir(kg, state, &neighbor, render_buffer);

    /* Evaluate picked sample from neighboring shading points. */
    if (!light_sample_from_uv(kg, &neighbor.sd, neighbor.path_flag, &picked_ls)) {
      continue;
    }
    radiance_eval(kg, state, &neighbor.sd, &picked_ls, &neighbor.reservoir.radiance, visibility);
    valid_neighbors += !bsdf_eval_is_zero(&neighbor.reservoir.radiance);
  }

  current->reservoir.total_weight /= valid_neighbors;

  return true;
}

ccl_device bool integrator_restir(KernelGlobals kg,
                                  IntegratorState state,
                                  ccl_global const KernelWorkTile *ccl_restrict tile,
                                  ccl_global float *ccl_restrict render_buffer,
                                  const int x,
                                  const int y,
                                  const int scheduled_sample)
{
  PROFILING_INIT(kg, PROFILING_RESTIR_SPATIAL_RESAMPLING);
  SpatialReservoir current(kg);
  integrator_restir_unpack_shader(kg, state, &current, render_buffer);

  if (!(current.sd.type)) {
    /* No interesction at the current shading point. */
    /* TODO(weizhen): revisit this condition when we support background and distant lights. */
    return false;
  }

  shader_data_setup_from_restir(kg, state, &current, render_buffer);

  /* Load random number state. */
  RNGState rng_state;
  path_state_rng_load(state, &rng_state);

  /* Plus one to account for the current pixel. */
  const int samples = kernel_data.integrator.restir_spatial_samples + 1;

  const int radius = kernel_data.integrator.restir_spatial_radius;
  SpatialResampling spatial_resampling(tile, radius);

  const bool visibility = kernel_data.integrator.restir_spatial_visibility;

  /* TODO(weizhen): add options for pairwiseMIS and biasedMIS. The current MIS weight is not good
   * for point light with soft falloff and area light with small spread. */
  /* Uniformly sample neighboring reservoirs within a radius. There is probability to pick the same
   * reservoir twice, but the chance should be low if the radius is big enough and low descrepancy
   * samples are used. */
  bool success = spatial_sample_streaming(
      kg, state, &spatial_resampling, &current, &rng_state, samples, visibility, render_buffer);

  if (!success) {
    return false;
  }

  BsdfEval radiance = current.reservoir.radiance;
  bsdf_eval_mul(&radiance, current.reservoir.total_weight);

  integrate_direct_light_create_shadow_path<true>(
      kg, state, &rng_state, &current.sd, &current.reservoir.ls, &radiance, 0);

  return true;
}

CCL_NAMESPACE_END

#endif  // RESTIR_H_
