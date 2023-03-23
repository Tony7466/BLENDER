/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_lightprobe_types.h"
#include "GPU_capabilities.h"

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
  if (inst_.is_baking()) {
    bake.sync();
  }
  else {
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

void IrradianceBake::sync()
{
}

void IrradianceBake::surfels_create(const IrradianceGrid & /* grid */)
{
  /**
   * We rasterize the scene along the 3 axes. Each generated fragment will write a surface element
   * so raster grid density need to match the desired surfel density. We do a first pass to know
   * how much surfel to allocate then render again to create the surfels.
   */
  using namespace blender::math;

  /* Attachment-less frame-buffer. */
  empty_raster_fb_.ensure(int2(20 * 4));

  /** We could use multi-view rendering here to avoid multiple submissions but it is unlikely to
   * make any difference. The bottleneck is still the light propagation loop. */
  auto render_axis = [&](Axis axis) {
    /* TODO(fclem): get scene bounds GPU or CPU side. Or use the irradiance grid extents. */
    float4x4 winmat = math::projection::orthographic(-20.0f, 20.0f, -20.0f, 20.0f, -20.0f, 20.0f);

    CartesianBasis basis = from_orthonormal_axes(AxisSigned(axis).next_after(), axis);
    view_.sync(from_rotation<float4x4>(basis), winmat);

    inst_.pipelines.capture.render(view_);
  };

  DRW_stats_group_start("IrradianceBake.SurfelsCount");

  /* Raster the scene to query the number of surfel needed. */
  capture_info_buf_.do_surfel_count = true;
  capture_info_buf_.do_surfel_output = false;
  capture_info_buf_.surfel_len = 0u;
  capture_info_buf_.push_update();
  render_axis(Axis::X);
  render_axis(Axis::Y);
  render_axis(Axis::Z);

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
  render_axis(Axis::X);
  render_axis(Axis::Y);
  render_axis(Axis::Z);

  DRW_stats_group_end();
}

void IrradianceBake::surfels_lights_eval()
{
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
