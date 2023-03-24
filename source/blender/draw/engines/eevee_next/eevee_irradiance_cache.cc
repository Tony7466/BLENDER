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

void IrradianceBake::surfels_create(const IrradianceGrid &grid)
{
  /**
   * We rasterize the scene along the 3 axes. Each generated fragment will write a surface element
   * so raster grid density need to match the desired surfel density. We do a first pass to know
   * how much surfel to allocate then render again to create the surfels.
   */
  using namespace blender::math;

  float4x4 transform(grid.transform);
  float3 location, scale;
  Quaternion rotation;
  math::to_loc_rot_scale(transform, location, rotation, scale);

  /** We could use multi-view rendering here to avoid multiple submissions but it is unlikely to
   * make any difference. The bottleneck is still the light propagation loop. */
  auto sync_view = [&](View &view, CartesianBasis basis) {
    float3 extent = scale;
    float4x4 winmat = projection::orthographic(
        -extent.x, extent.x, -extent.y, extent.y, -extent.z, extent.z);
    float4x4 viewinv = math::from_loc_rot<float4x4>(location,
                                                    rotation * to_quaternion<float>(basis));
    view.sync(invert(viewinv), winmat);
  };

  sync_view(view_x_, basis_x_);
  sync_view(view_y_, basis_y_);
  sync_view(view_z_, basis_z_);

  /* Surfel per unit distance. */
  float surfel_density = 2.0f;
  grid_pixel_extent_ = max(int3(1), int3(surfel_density * 2.0f * scale));

  DRW_stats_group_start("IrradianceBake.SurfelsCount");
  GPU_debug_capture_begin();

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

  GPU_debug_capture_end();
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

  /* Sync needs to happen after `surfels_buf_` is resized for correct dispatch size. */
  sync();
}

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
    pass.dispatch(int3(divide_ceil_u(surfels_buf_.size(), SURFEL_LIGHT_GROUP_SIZE), 1, 1));
    pass.barrier(GPU_BARRIER_SHADER_STORAGE);
  }
  {
    PassSimple &pass = surfel_light_propagate_ps_;
    pass.init();
    {
      PassSimple::Sub &sub = pass.sub("ListBuild");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_BUILD));
      /* TODO */
    }
    {
      PassSimple::Sub &sub = pass.sub("ListSort");
      sub.shader_set(inst_.shaders.static_shader_get(SURFEL_LIST_SORT));
      /* TODO */
    }
  }
  {
    PassSimple &pass = irradiance_capture_ps_;
    pass.init();
    /* TODO */
  }
}

void IrradianceBake::surfels_lights_eval()
{
  GPU_debug_capture_begin();
  /* Use the last setup view. This should work since the view is orthographic. */
  /* TODO(fclem): Remove this. It is only present to avoid crash inside `shadows.set_view` */
  inst_.render_buffers.acquire(int2(1));
  inst_.lights.set_view(view_z_, grid_pixel_extent_.xy());
  /* TODO: Instead of using the volume tagging we should tag using the surfels. */
  inst_.shadows.set_view(view_z_);
  inst_.render_buffers.release();

  inst_.manager->submit(surfel_light_eval_ps_);

  GPU_debug_capture_end();
}

void IrradianceBake::propagate_light_sample()
{
  /* Pick random ray direction over the sphere. */
  /* Project to regular grid and create the surfels lists. */
  /* Sort the surfels lists. */
  /* Propagate light. */
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
