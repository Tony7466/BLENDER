/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_lightprobe_types.h"

#include "GPU_capabilities.h"
#include "GPU_debug.h"

#include "BLI_math_rotation.hh"

#include "eevee_instance.hh"

#include "eevee_irradiance_cache.hh"

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name Interface
 * \{ */

void IrradianceCache::init()
{
}

void IrradianceCache::sync()
{
  if (!inst_.is_baking()) {
    debug_pass_sync();
  }
  else {
    bake.sync();
  }
}

void IrradianceCache::debug_pass_sync()
{
  if (!ELEM(inst_.debug_mode,
            eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL,
            eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE)) {
    return;
  }

  LightCache *light_cache = inst_.scene->eevee.light_cache_data;
  if (light_cache == nullptr || light_cache->version != LIGHTCACHE_NEXT_STATIC_VERSION ||
      light_cache->grids == nullptr || light_cache->grid_len == 0) {
    return;
  }

  debug_surfels_ps_.init();
  debug_surfels_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                              DRW_STATE_DEPTH_LESS_EQUAL);
  debug_surfels_ps_.shader_set(inst_.shaders.static_shader_get(DEBUG_SURFELS));
  debug_surfels_ps_.push_constant("surfel_radius", 0.5f / 4.0f);
  debug_surfels_ps_.push_constant("debug_mode", static_cast<int>(inst_.debug_mode));

  surfels_buf_.clear();
  for (auto i : IndexRange(light_cache->grid_len)) {
    LightCacheIrradianceGrid &grid = light_cache->grids[i];
    if (grid.surfels_len > 0 && grid.surfels != nullptr) {
      Span<Surfel> grid_surfels(static_cast<Surfel *>(grid.surfels), grid.surfels_len);
      for (const Surfel &surfel : grid_surfels) {
        surfels_buf_.append(surfel);
      }
    }
  }
  surfels_buf_.push_update();
  debug_surfels_ps_.bind_ssbo("surfels_buf", surfels_buf_);
  debug_surfels_ps_.draw_procedural(GPU_PRIM_TRI_STRIP, surfels_buf_.size(), 4);
}

void IrradianceCache::debug_draw(View &view, GPUFrameBuffer *view_fb)
{
  switch (inst_.debug_mode) {
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL:
      inst_.info = "Debug Mode: Surfels Normal";
      break;
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE:
      inst_.info = "Debug Mode: Surfels Irradiance";
      break;
    default:
      /* Nothing to display. */
      return;
  }

  GPU_framebuffer_bind(view_fb);
  inst_.manager->submit(debug_surfels_ps_, view);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Baking
 * \{ */

void IrradianceBake::sync()
{
  {
    PassSimple &pass = surfel_light_eval_ps_;
    pass.init();
    /* Apply lights contribution to scene surfel representation. */
    pass.shader_set(inst_.shaders.static_shader_get(SURFEL_LIGHT));
    pass.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
    pass.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
    pass.bind_texture(RBUFS_UTILITY_TEX_SLOT, inst_.pipelines.utility_tx);
    inst_.lights.bind_resources(&pass);
    inst_.shadows.bind_resources(&pass);
    /* Sync with the surfel creation stage. */
    pass.barrier(GPU_BARRIER_SHADER_STORAGE);
    pass.dispatch(&dispatch_per_surfel_);
  }
  {
    PassSimple &pass = surfel_light_propagate_ps_;
    pass.init();
    {
      PassSimple::Sub &sub = pass.sub("ListBuild");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_BUILD));
      sub.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
      sub.bind_ssbo(CAPTURE_BUF_SLOT, &capture_info_buf_);
      sub.bind_ssbo("list_start_buf", &list_start_buf_);
      sub.bind_ssbo("list_info_buf", &list_info_buf_);
      sub.barrier(GPU_BARRIER_SHADER_STORAGE);
      sub.dispatch(&dispatch_per_surfel_);
    }
    {
      PassSimple::Sub &sub = pass.sub("ListSort");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_SORT));
      sub.bind_ssbo(SURFEL_BUF_SLOT, &surfels_buf_);
      sub.bind_ssbo("list_start_buf", &list_start_buf_);
      sub.bind_ssbo("list_info_buf", &list_info_buf_);
      sub.barrier(GPU_BARRIER_SHADER_STORAGE);
      sub.dispatch(&dispatch_per_list_);
    }
    {
      // PassSimple::Sub &sub = pass.sub("LightPropagate");
    }
  }
  {
    PassSimple &pass = irradiance_capture_ps_;
    pass.init();
    /* TODO */
  }
}

void IrradianceBake::surfels_create(const IrradianceGrid &grid)
{
  /**
   * We rasterize the scene along the 3 axes. Each generated fragment will write a surface element
   * so raster grid density need to match the desired surfel density. We do a first pass to know
   * how much surfel to allocate then render again to create the surfels.
   */
  using namespace blender::math;

  const float4x4 transform(grid.transform);
  float3 scale;
  math::to_loc_rot_scale(transform, grid_location_, grid_orientation_, scale);

  /* Extract bounding box. Order is arbitrary as it is not important for our usage. */
  const std::array<float3, 8> bbox_corners({float3{+1, +1, +1},
                                            float3{-1, +1, +1},
                                            float3{+1, -1, +1},
                                            float3{-1, -1, +1},
                                            float3{+1, +1, -1},
                                            float3{-1, +1, -1},
                                            float3{+1, -1, -1},
                                            float3{-1, -1, -1}});
  grid_bbox_vertices.clear();
  for (const float3 &point : bbox_corners) {
    grid_bbox_vertices.append(transform_point(transform, point));
  }

  /* We could use multi-view rendering here to avoid multiple submissions but it is unlikely to
   * make any difference. The bottleneck is still the light propagation loop. */
  auto sync_view = [&](View &view, CartesianBasis basis) {
    float3 extent = scale;
    float4x4 winmat = projection::orthographic(
        -extent.x, extent.x, -extent.y, extent.y, -extent.z, extent.z);
    float4x4 viewinv = math::from_loc_rot<float4x4>(
        grid_location_, grid_orientation_ * to_quaternion<float>(basis));
    view.sync(invert(viewinv), winmat);
  };

  sync_view(view_x_, basis_x_);
  sync_view(view_y_, basis_y_);
  sync_view(view_z_, basis_z_);

  grid_pixel_extent_ = max(int3(1), int3(surfel_density_ * 2.0f * scale));

  DRW_stats_group_start("IrradianceBake.SurfelsCount");

  /* Raster the scene to query the number of surfel needed. */
  capture_info_buf_.do_surfel_count = true;
  capture_info_buf_.do_surfel_output = false;
  capture_info_buf_.surfel_len = 0u;
  capture_info_buf_.push_update();

  empty_raster_fb_.ensure(grid_pixel_extent_.yz());
  inst_.pipelines.capture.render(view_x_);
  empty_raster_fb_.ensure(int2(grid_pixel_extent_.x, grid_pixel_extent_.z));
  inst_.pipelines.capture.render(view_y_);
  empty_raster_fb_.ensure(grid_pixel_extent_.xy());
  inst_.pipelines.capture.render(view_z_);

  DRW_stats_group_end();

  /* Allocate surfel pool. */
  GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
  capture_info_buf_.read();
  if (capture_info_buf_.surfel_len == 0) {
    /* No surfel to allocated. */
    return;
  }

  /* TODO(fclem): Check for GL limit and abort if the surfel cache doesn't fit the GPU memory. */
  surfels_buf_.resize(capture_info_buf_.surfel_len);

  dispatch_per_surfel_.x = divide_ceil_u(surfels_buf_.size(), SURFEL_GROUP_SIZE);

  DRW_stats_group_start("IrradianceBake.SurfelsCreate");

  /* Raster the scene to generate the surfels. */
  capture_info_buf_.do_surfel_count = true;
  capture_info_buf_.do_surfel_output = true;
  capture_info_buf_.surfel_len = 0u;
  capture_info_buf_.push_update();
  empty_raster_fb_.ensure(grid_pixel_extent_.yz());
  inst_.pipelines.capture.render(view_x_);
  empty_raster_fb_.ensure(int2(grid_pixel_extent_.x, grid_pixel_extent_.z));
  inst_.pipelines.capture.render(view_y_);
  empty_raster_fb_.ensure(grid_pixel_extent_.xy());
  inst_.pipelines.capture.render(view_z_);

  DRW_stats_group_end();
}

void IrradianceBake::surfels_lights_eval()
{
  /* Use the last setup view. This should work since the view is orthographic. */
  /* TODO(fclem): Remove this. It is only present to avoid crash inside `shadows.set_view` */
  inst_.render_buffers.acquire(int2(1));
  inst_.lights.set_view(view_z_, grid_pixel_extent_.xy());
  /* TODO: Instead of using the volume tagging we should tag using the surfels. */
  inst_.shadows.set_view(view_z_);
  inst_.render_buffers.release();

  inst_.manager->submit(surfel_light_eval_ps_);
}

void IrradianceBake::propagate_light_sample()
{
  using namespace blender::math;

  float2 rand_uv = inst_.sampling.rng_2d_get(eSamplingDimension::SAMPLING_FILTER_U);
  const float3 ray_direction = inst_.sampling.sample_hemisphere(rand_uv);
  const float3 up = ray_direction;
  /* Find the closest axis. */
  const float3 grid_local_ray_direction = transform_point(grid_orientation_, ray_direction);
  Axis closest_grid_axis = Axis::from_int(dominant_axis(grid_local_ray_direction));
  /* Use one of the other 2 grid axes to get a reference right vector. */
  Axis right_axis = AxisSigned(closest_grid_axis).next_after().axis();
  const float3 grid_right = from_rotation<float3x3>(grid_orientation_)[right_axis.as_int()];
  /* Create a view around the grid position with the ray direction as up axis.
   * The other axes are aligned to the grid local axes to avoid to allocate too many list start. */
  const float4x4 viewmat = invert(
      from_orthonormal_axes<float4x4>(grid_location_, normalize(cross(up, grid_right)), up));

  /* Compute projection bounds. */
  float2 min, max;
  INIT_MINMAX2(min, max);
  for (const float3 &point : grid_bbox_vertices) {
    min_max(transform_point(viewmat, point).xy(), min, max);
  }

  /* NOTE: Z values do not really matter since we are not doing any rasterization. */
  const float4x4 winmat = projection::orthographic<float>(min.x, max.x, min.y, max.y, 0, 1);

  View ray_view = {"RayProjectionView"};
  ray_view.sync(viewmat, winmat);

  list_info_buf_.ray_grid_size = math::max(int2(1), int2(surfel_density_ * (max - min)));
  list_info_buf_.list_max = list_info_buf_.ray_grid_size.x * list_info_buf_.ray_grid_size.y;
  list_info_buf_.push_update();

  dispatch_per_list_.x = divide_ceil_u(list_info_buf_.list_max, SURFEL_LIST_GROUP_SIZE);

  list_start_buf_.resize(ceil_to_multiple_u(list_info_buf_.list_max, 4));

  GPU_storagebuf_clear(list_start_buf_, -1);
  inst_.manager->submit(surfel_light_propagate_ps_, ray_view);
}

void IrradianceBake::read_result(LightCacheIrradianceGrid &light_cache_grid)
{
  switch (inst_.debug_mode) {
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_NORMAL:
    case eDebugMode::DEBUG_IRRADIANCE_CACHE_SURFELS_IRRADIANCE:
      GPU_memory_barrier(GPU_BARRIER_BUFFER_UPDATE);
      capture_info_buf_.read();
      surfels_buf_.read();
      light_cache_grid.surfels_len = capture_info_buf_.surfel_len;
      light_cache_grid.surfels = MEM_dupallocN(surfels_buf_.data());
      break;
    default:
      /* Nothing to display. */
      return;
  }
}

/** \} */

}  // namespace blender::eevee
