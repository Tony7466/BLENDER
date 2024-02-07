/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_bit_vector.hh"

#include "eevee_instance.hh"
#include "eevee_reflection_probes.hh"

#include <iostream>

namespace blender::eevee {

/* -------------------------------------------------------------------- */
/** \name ProbeLocationFinder
 * \{ */

/**
 * Utility class to find a location in the probes_tx_ that can be used to store a new probe in
 * a specified subdivision level.
 */
class ProbeLocationFinder {
  BitVector<> taken_spots_;
  int probes_per_dimension_;
  int probes_per_layer_;
  int subdivision_level_;

 public:
  ProbeLocationFinder(int num_layers, int subdivision_level)
  {
    subdivision_level_ = subdivision_level;
    probes_per_dimension_ = 1 << subdivision_level_;
    probes_per_layer_ = probes_per_dimension_ * probes_per_dimension_;
    int num_spots = num_layers * probes_per_layer_;
    taken_spots_.resize(num_spots, false);
  }

  void print_debug() const
  {
    std::ostream &os = std::cout;
    int layer = 0;
    int row = 0;
    int column = 0;

    os << "subdivision " << subdivision_level_ << "\n";

    for (bool spot_taken : taken_spots_) {
      if (row == 0 && column == 0) {
        os << "layer " << layer << "\n";
      }

      os << (spot_taken ? '1' : '0');

      column++;
      if (column == probes_per_dimension_) {
        os << "\n";
        column = 0;
        row++;
      }
      if (row == probes_per_dimension_) {
        row = 0;
        layer++;
      }
    }
  }

  /**
   * Mark space to be occupied by the given probe_data.
   *
   * The input probe data can be stored in a different subdivision level and should be converted to
   * the subdivision level what we are looking for.
   */
  void mark_space_used(const ReflectionProbeAtlasCoordinate &coord)
  {
    if (coord.layer == -1) {
      /* Coordinate not allocated yet. */
      return;
    }
    const int shift_right = max_ii(coord.layer_subdivision - subdivision_level_, 0);
    const int shift_left = max_ii(subdivision_level_ - coord.layer_subdivision, 0);
    const int spots_per_dimension = 1 << shift_left;
    const int probes_per_dimension_in_probe_data = 1 << coord.layer_subdivision;
    const int2 pos_in_probe_data = int2(coord.area_index % probes_per_dimension_in_probe_data,
                                        coord.area_index / probes_per_dimension_in_probe_data);
    const int2 pos_in_location_finder = int2((pos_in_probe_data.x >> shift_right) << shift_left,
                                             (pos_in_probe_data.y >> shift_right) << shift_left);
    const int layer_offset = coord.layer * probes_per_layer_;
    for (const int y : IndexRange(spots_per_dimension)) {
      for (const int x : IndexRange(spots_per_dimension)) {
        const int2 pos = pos_in_location_finder + int2(x, y);
        const int area_index = pos.x + pos.y * probes_per_dimension_;
        taken_spots_[area_index + layer_offset].set();
      }
    }
  }

  /**
   * Get the first free spot.
   *
   * .x contains the layer the first free spot was detected.
   * .y contains the area_index to use.
   *
   * Asserts when no free spot is found. ProbeLocationFinder should always be initialized with an
   * additional layer to make sure that there is always a free spot.
   */
  ReflectionProbeAtlasCoordinate first_free_spot() const
  {
    ReflectionProbeAtlasCoordinate result;
    result.layer_subdivision = subdivision_level_;
    for (int index : taken_spots_.index_range()) {
      if (!taken_spots_[index]) {
        result.layer = index / probes_per_layer_;
        result.area_index = index % probes_per_layer_;
        return result;
      }
    }
    BLI_assert_unreachable();
    return result;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Reflection Probe Module
 * \{ */

eLightProbeResolution ReflectionProbeModule::reflection_probe_resolution() const
{
  switch (instance_.scene->eevee.gi_cubemap_resolution) {
    case 64:
      return LIGHT_PROBE_RESOLUTION_64;
    case 128:
      return LIGHT_PROBE_RESOLUTION_128;
    case 256:
      return LIGHT_PROBE_RESOLUTION_256;
    case 512:
      return LIGHT_PROBE_RESOLUTION_512;
    case 1024:
      return LIGHT_PROBE_RESOLUTION_1024;
    default:
      return LIGHT_PROBE_RESOLUTION_2048;
  }
  return LIGHT_PROBE_RESOLUTION_2048;
}

int ReflectionProbeModule::probe_render_extent() const
{
  return instance_.scene->eevee.gi_cubemap_resolution / 2;
}

ReflectionProbeModule::ReflectionProbeModule(Instance &instance) : instance_(instance)
{
  /* Initialize the world probe. */
  world_probe.clipping_distances = float2(1.0f, 10.0f);
  world_probe.world_to_probe_transposed = float3x4::identity();
  world_probe.influence_shape = SHAPE_ELIPSOID;
  world_probe.parallax_shape = SHAPE_ELIPSOID;
  /* Full influence. */
  world_probe.influence_scale = 0.0f;
  world_probe.influence_bias = 1.0f;
  world_probe.parallax_distance = 1e10f;
  /* In any case, the world must always be up to valid and used for render. */
  world_probe.use_for_render = true;
}

void ReflectionProbeModule::init()
{
  if (!instance_.is_viewport()) {
    /* TODO(jbakker): should we check on the subtype as well? Now it also populates even when
     * there are other light probes in the scene. */
    update_probes_next_sample_ = DEG_id_type_any_exists(instance_.depsgraph, ID_LP);
  }

  {
    const RaytraceEEVEE &options = instance_.scene->eevee.ray_tracing_options;
    float probe_brightness_clamp = (options.sample_clamp > 0.0) ? options.sample_clamp : 1e20;

    PassSimple &pass = remap_ps_;
    pass.init();
    pass.shader_set(instance_.shaders.static_shader_get(REFLECTION_PROBE_REMAP));
    pass.bind_texture("cubemap_tx", &cubemap_tx_);
    pass.bind_texture("atlas_tx", &probes_tx_);
    pass.bind_image("atlas_img", &probes_tx_);
    pass.push_constant("probe_coord_packed", reinterpret_cast<int4 *>(&probe_sampling_coord_));
    pass.push_constant("write_coord_packed", reinterpret_cast<int4 *>(&probe_write_coord_));
    pass.push_constant("world_coord_packed", reinterpret_cast<int4 *>(&world_sampling_coord_));
    pass.push_constant("mip_level", &probe_mip_level_);
    pass.push_constant("probe_brightness_clamp", probe_brightness_clamp);
    pass.dispatch(&dispatch_probe_pack_);
  }

  {
    PassSimple &pass = update_irradiance_ps_;
    pass.init();
    pass.shader_set(instance_.shaders.static_shader_get(REFLECTION_PROBE_UPDATE_IRRADIANCE));
    pass.push_constant("world_coord_packed", reinterpret_cast<int4 *>(&world_sampling_coord_));
    pass.bind_image("irradiance_atlas_img", &instance_.irradiance_cache.irradiance_atlas_tx_);
    pass.bind_texture("reflection_probes_tx", &probes_tx_);
    pass.dispatch(int2(1, 1));
  }

  do_display_draw_ = false;
}

void ReflectionProbeModule::begin_sync()
{
  update_probes_this_sample_ = update_probes_next_sample_;

  {
    PassSimple &pass = select_ps_;
    pass.init();
    pass.shader_set(instance_.shaders.static_shader_get(REFLECTION_PROBE_SELECT));
    pass.push_constant("reflection_probe_count", &reflection_probe_count_);
    pass.bind_ssbo("reflection_probe_buf", &data_buf_);
    instance_.irradiance_cache.bind_resources(pass);
    instance_.sampling.bind_resources(pass);
    pass.dispatch(&dispatch_probe_select_);
    pass.barrier(GPU_BARRIER_UNIFORM);
  }
}

int ReflectionProbeModule::needed_layers_get() const
{
  int max_layer = 0;
  for (const ReflectionCube &probe : instance_.light_probes.cube_map_.values()) {
    max_layer = max_ii(max_layer, probe.atlas_coord.layer);
  }
  return max_layer + 1;
}

void ReflectionProbeModule::sync_world(::World *world)
{
  const eLightProbeResolution probe_resolution = static_cast<eLightProbeResolution>(
      world->probe_resolution);

  int subdivision_lvl = ReflectionCube::subdivision_level_get(max_resolution_, probe_resolution);
  if (subdivision_lvl != world_probe.atlas_coord.layer_subdivision) {
    world_probe.atlas_coord = find_empty_atlas_region(subdivision_lvl);
    ReflectionProbeData &world_probe_data = *static_cast<ReflectionProbeData *>(&world_probe);
    world_probe_data.atlas_coord = world_probe.atlas_coord.as_sampling_coord(max_resolution_);
    tag_world_for_update();
  }
  world_sampling_coord_ = world_probe.atlas_coord.as_sampling_coord(atlas_extent());
}

ReflectionProbeAtlasCoordinate ReflectionProbeModule::find_empty_atlas_region(
    int subdivision_level) const
{
  ProbeLocationFinder location_finder(needed_layers_get() + 1, subdivision_level);

  location_finder.mark_space_used(world_probe.atlas_coord);
  for (const ReflectionCube &probe : instance_.light_probes.cube_map_.values()) {
    location_finder.mark_space_used(probe.atlas_coord);
  }
  return location_finder.first_free_spot();
}

bool ReflectionProbeModule::ensure_atlas()
{
  /* Make sure the atlas is always initialized even if there is nothing to render to it to fullfil
   * the resource bindings. */
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_SHADER_WRITE | GPU_TEXTURE_USAGE_SHADER_READ;

  if (probes_tx_.ensure_2d_array(GPU_RGBA16F,
                                 int2(max_resolution_),
                                 needed_layers_get(),
                                 usage,
                                 nullptr,
                                 REFLECTION_PROBE_MIPMAP_LEVELS))
  {
    /* TODO(fclem): Clearing means that we need to render all probes again.
     * If existing data exists, copy it using `CopyImageSubData`. */
    probes_tx_.clear(float4(0.0f));
    GPU_texture_mipmap_mode(probes_tx_, true, true);
    /* Avoid undefined pixel data. Update all mips. */
    GPU_texture_update_mipmap_chain(probes_tx_);
    return true;
  }
  return false;
}

void ReflectionProbeModule::end_sync()
{
  const bool world_updated = world_probe.do_render;
  const bool atlas_resized = ensure_atlas();
  /* Detect if we need to render probe objects. */
  update_probes_next_sample_ = false;
  for (ReflectionCube &probe : instance_.light_probes.cube_map_.values()) {
    if (atlas_resized || world_updated) {
      /* Last minute tagging. */
      probe.do_render = true;
    }
    if (probe.do_render) {
      /* Tag the next redraw to warm up the probe pipeline.
       * Keep doing this until there is no update.
       * This avoids stuttering when moving a lightprobe. */
      update_probes_next_sample_ = true;
    }
  }
  std::cout << "end_sync " << std::endl;
  if (update_probes_this_sample_) {
    std::cout << "update_probes_this_sample_ " << std::endl;
  }
  if (update_probes_next_sample_) {
    std::cout << "update_probes_next_sample_ " << std::endl;
  }
  /* If we cannot render probes this redraw make sure we request another redraw. */
  if (update_probes_next_sample_ && (instance_.do_reflection_probe_sync() == false)) {
    DRW_viewport_request_redraw();
    std::cout << "Request redraw" << std::endl;
  }
  debug_print();
}

bool ReflectionProbeModule::has_only_world_probe() const
{
  return instance_.light_probes.cube_map_.size() == 0;
}

void ReflectionProbeModule::ensure_cubemap_render_target(int resolution)
{
  if (cubemap_tx_.ensure_cube(
          GPU_RGBA16F, resolution, GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_SHADER_READ))
  {
    GPU_texture_mipmap_mode(cubemap_tx_, false, true);
  }
  /* TODO(fclem): dealocate it. */
}

ReflectionProbeUpdateInfo ReflectionProbeModule::update_info_from_probe(
    const ReflectionCube &probe)
{
  const int max_shift = int(log2(max_resolution_));

  ReflectionProbeUpdateInfo info = {};
  info.atlas_coord = probe.atlas_coord;
  info.resolution = 1 << (max_shift - probe.atlas_coord.layer_subdivision - 1);
  info.clipping_distances = probe.clipping_distances;
  info.probe_pos = probe.location;
  info.do_render = probe.do_render;
  info.do_world_irradiance_update = false;
  return info;
}

std::optional<ReflectionProbeUpdateInfo> ReflectionProbeModule::world_update_info_pop()
{
  if (!world_probe.do_render && !do_world_irradiance_update) {
    return std::nullopt;
  }
  ReflectionProbeUpdateInfo info = update_info_from_probe(world_probe);
  info.do_world_irradiance_update = do_world_irradiance_update;
  world_probe.do_render = false;
  do_world_irradiance_update = false;
  ensure_cubemap_render_target(info.resolution);
  return info;
}

std::optional<ReflectionProbeUpdateInfo> ReflectionProbeModule::probe_update_info_pop()
{
  if (!instance_.do_reflection_probe_sync()) {
    /* Do not update probes during this sample as we did not sync the draw::Passes. */
    return std::nullopt;
  }

  for (ReflectionCube &probe : instance_.light_probes.cube_map_.values()) {
    if (!probe.do_render) {
      continue;
    }
    ReflectionProbeUpdateInfo info = update_info_from_probe(probe);
    probe.do_render = false;
    probe.use_for_render = true;
    ensure_cubemap_render_target(info.resolution);
    return info;
  }

  return std::nullopt;
}

void ReflectionProbeModule::remap_to_octahedral_projection(
    const ReflectionProbeAtlasCoordinate &atlas_coord)
{
  int resolution = max_resolution_ >> atlas_coord.layer_subdivision;
  /* Update shader parameters that change per dispatch. */
  probe_sampling_coord_ = atlas_coord.as_sampling_coord(atlas_extent());
  probe_write_coord_ = atlas_coord.as_write_coord(atlas_extent(), 0);
  probe_mip_level_ = atlas_coord.layer_subdivision;
  dispatch_probe_pack_ = int3(int2(ceil_division(resolution, REFLECTION_PROBE_GROUP_SIZE)), 1);

  instance_.manager->submit(remap_ps_);
}

void ReflectionProbeModule::update_world_irradiance()
{
  instance_.manager->submit(update_irradiance_ps_);
}

void ReflectionProbeModule::update_probes_texture_mipmaps()
{
  GPU_texture_update_mipmap_chain(probes_tx_);
}

void ReflectionProbeModule::set_view(View & /*view*/)
{
  Vector<ReflectionCube *> probe_active;
  for (auto &probe : instance_.light_probes.cube_map_.values()) {
    /* Last slot is reserved for the world probe. */
    if (reflection_probe_count_ >= REFLECTION_PROBES_MAX - 1) {
      break;
    }
    if (!probe.use_for_render) {
      continue;
    }
    /* TODO(fclem): Culling. */
    probe_active.append(&probe);
  }

  /* Stable sorting of probes. */
  std::sort(probe_active.begin(),
            probe_active.end(),
            [](const ReflectionCube *a, const ReflectionCube *b) {
              if (a->volume != b->volume) {
                /* Smallest first. */
                return a->volume < b->volume;
              }
              /* Volumes are identical. Any arbitrary criteria can be used to sort them.
               * Use position to avoid unstable result caused by depsgraph non deterministic eval
               * order. This could also become a priority parameter. */
              float3 _a = a->location;
              float3 _b = b->location;
              if (_a.x != _b.x) {
                return _a.x < _b.x;
              }
              if (_a.y != _b.y) {
                return _a.y < _b.y;
              }
              if (_a.z != _b.z) {
                return _a.z < _b.z;
              }
              /* Fallback to memory address, since there's no good alternative. */
              return a < b;
            });

  /* Push all sorted data to the UBO. */
  int probe_id = 0;
  for (auto &probe : probe_active) {
    data_buf_[probe_id++] = *probe;
  }
  /* Add world probe at the end. */
  data_buf_[probe_id++] = world_probe;
  /* Tag the end of the array. */
  if (probe_id < REFLECTION_PROBES_MAX) {
    data_buf_[probe_id].atlas_coord.layer = -1;
  }
  data_buf_.push_update();

  do_display_draw_ = DRW_state_draw_support() && probe_active.size() > 0;
  if (do_display_draw_) {
    int display_index = 0;
    for (int i : probe_active.index_range()) {
      if (probe_active[i]->viewport_display) {
        display_data_buf_.get_or_resize(display_index++) = {
            i, probe_active[i]->viewport_display_size};
      }
    }
    do_display_draw_ = display_index > 0;
    if (do_display_draw_) {
      display_data_buf_.resize(display_index);
      display_data_buf_.push_update();
    }
  }

  /* Add one for world probe. */
  reflection_probe_count_ = probe_active.size() + 1;
  dispatch_probe_select_.x = divide_ceil_u(reflection_probe_count_,
                                           REFLECTION_PROBE_SELECT_GROUP_SIZE);
  instance_.manager->submit(select_ps_);
}

ReflectionProbeAtlasCoordinate ReflectionProbeModule::world_atlas_coord_get() const
{
  return world_probe.atlas_coord;
}

void ReflectionProbeModule::viewport_draw(View &view, GPUFrameBuffer *view_fb)
{
  if (!do_display_draw_) {
    return;
  }

  viewport_display_ps_.init();
  viewport_display_ps_.state_set(DRW_STATE_WRITE_COLOR | DRW_STATE_WRITE_DEPTH |
                                 DRW_STATE_DEPTH_LESS_EQUAL | DRW_STATE_CULL_BACK);
  viewport_display_ps_.framebuffer_set(&view_fb);
  viewport_display_ps_.shader_set(instance_.shaders.static_shader_get(DISPLAY_PROBE_REFLECTION));
  bind_resources(viewport_display_ps_);
  viewport_display_ps_.bind_ssbo("display_data_buf", display_data_buf_);
  viewport_display_ps_.draw_procedural(GPU_PRIM_TRIS, 1, display_data_buf_.size() * 6);

  instance_.manager->submit(viewport_display_ps_, view);
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Debugging
 *
 * \{ */

void ReflectionProbeModule::debug_print() const
{
  std::ostream &os = std::cout;
  auto print_probe = [&](const ReflectionCube &probe) {
    os << " used: " << probe.used;
    os << " do_render: " << probe.do_render;
    os << " use_for_render: " << probe.use_for_render;
    os << "\n";
    os << " - layer: " << probe.atlas_coord.layer;
    os << " subdivision: " << probe.atlas_coord.layer_subdivision;
    os << " area: " << probe.atlas_coord.area_index;
    os << "\n";
  };

  print_probe(world_probe);
  for (const ReflectionCube &probe : instance_.light_probes.cube_map_.values()) {
    print_probe(probe);
  }
}

/** \} */

}  // namespace blender::eevee
