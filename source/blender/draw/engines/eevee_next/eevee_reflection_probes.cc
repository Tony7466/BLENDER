/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#include <sstream>

#include "BLI_bit_vector.hh"
#include "BLI_span.hh"

#include "eevee_instance.hh"
#include "eevee_reflection_probes.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (cubemaps_.is_empty()) {
    cubemaps_.reserve(INITIAL_PROBES);

    /* Initialize the world cubemap. */
    ReflectionProbeData world_probe_data{};
    world_probe_data.layer = 0;
    world_probe_data.layer_subdivision = WORLD_SUBDIVISION_LEVEL;
    world_probe_data.area_index = 0;
    world_probe_data.color = float4(0.0f);
    data_buf_[0] = world_probe_data;

    ReflectionProbe world_cubemap;
    world_cubemap.type = ReflectionProbe::Type::WORLD;
    world_cubemap.is_dirty = true;
    world_cubemap.index = 0;
    cubemaps_.append(world_cubemap);

    cubemaps_tx_.ensure_cube_array(GPU_RGBA16F,
                                   MAX_RESOLUTION,
                                   INITIAL_PROBES,
                                   GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                                   NULL,
                                   MIPMAP_LEVELS);
    GPU_texture_mipmap_mode(cubemaps_tx_, true, true);
  }

  for (ReflectionProbe &reflection_probe : cubemaps_) {
    if (reflection_probe.type == ReflectionProbe::Type::PROBE) {
      reflection_probe.is_used = false;
    }
  }
}

int ReflectionProbeModule::needed_layers_get() const
{
  const int max_probe_data_index = reflection_probe_data_index_max();
  int max_layer = 0;
  for (const ReflectionProbeData &data :
       Span<ReflectionProbeData>(data_buf_.data(), max_probe_data_index + 1))
  {
    max_layer = max_ii(max_layer, data.layer);
  }
  return max_layer + 1;
}

void ReflectionProbeModule::sync(const ReflectionProbe &cubemap)
{
  switch (cubemap.type) {
    case ReflectionProbe::Type::WORLD: {
      GPUMaterial *world_material = instance_.world.get_world_material();
      instance_.pipelines.world_probe.sync(world_material);
      break;
    }
    case ReflectionProbe::Type::PROBE: {
      // upload the baked probe to the cubemaps.
      upload_dummy_cubemap(cubemap);
      break;
    }
    case ReflectionProbe::Type::UNUSED: {
      break;
    }
  }
}

void ReflectionProbeModule::set_world_dirty()
{
  cubemaps_[WORLD_SLOT].is_dirty = true;
}

void ReflectionProbeModule::sync_object(Object *ob,
                                        ObjectHandle &ob_handle,
                                        ResourceHandle res_handle,
                                        bool is_dirty)
{
  ReflectionProbe &probe = find_or_insert(ob_handle, REFLECTION_PROBE_SUBDIVISION_LEVEL);
  /* TODO: remove debug color.*/
  data_buf_[probe.index].color = ob->color;
  probe.is_dirty |= is_dirty;
  probe.is_used = true;
}

ReflectionProbe &ReflectionProbeModule::find_or_insert(ObjectHandle &ob_handle,
                                                       int subdivision_level)
{
  ReflectionProbe *first_unused = nullptr;
  for (ReflectionProbe &reflection_probe : cubemaps_) {
    if (reflection_probe.type == ReflectionProbe::Type::PROBE &&
        reflection_probe.object_hash_value == ob_handle.object_key.hash_value)
    {
      return reflection_probe;
    }
    if (first_unused == nullptr && reflection_probe.type == ReflectionProbe::Type::UNUSED) {
      first_unused = &reflection_probe;
    }
  }

  if (first_unused == nullptr) {
    ReflectionProbe new_slot{};
    cubemaps_.append_as(new_slot);
    first_unused = &cubemaps_.last();
  }
  BLI_assert(first_unused != nullptr);
  first_unused->is_dirty = true;
  first_unused->object_hash_value = ob_handle.object_key.hash_value;
  first_unused->type = ReflectionProbe::Type::PROBE;
  first_unused->index = reflection_probe_data_index_max() + 1;

  /* TODO: support different subdivision levels. */
  ReflectionProbeData probe_data = find_empty_reflection_probe_data(subdivision_level);
  data_buf_[first_unused->index] = probe_data;

  return *first_unused;
}

int ReflectionProbeModule::reflection_probe_data_index_max() const
{
  int result = -1;
  for (const ReflectionProbe &probe : cubemaps_) {
    if (probe.type != ReflectionProbe::Type::UNUSED) {
      result = max_ii(result, probe.index);
    }
  }
  return result;
}

/**
 * Utility class to find a location in the cubemap that can be used to store a new probe cubemap in
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

  /**
   * Mark space to be occupied by the given probe_data.
   *
   * The input probe data can be stored in a different subdivision level and should be converted to
   * the subdivision level what we are looking for.
   */
  void mark_space_used(const ReflectionProbeData &probe_data)
  {
    /* Number of spots that the probe data will occupied in a single dimension. */
    int clamped_subdivision_shift = max_ii(probe_data.layer_subdivision - subdivision_level_, 0);
    int spots_per_dimension = 1 << max_ii(subdivision_level_ - probe_data.layer_subdivision, 0);
    int probes_per_dimension_in_probe_data = 1 << probe_data.layer_subdivision;
    int2 pos_in_probe_data = int2(probe_data.area_index % probes_per_dimension_in_probe_data,
                                  probe_data.area_index / probes_per_dimension_in_probe_data);
    int2 pos_in_location_finder = int2(pos_in_probe_data.x >> clamped_subdivision_shift,
                                       pos_in_probe_data.y >> clamped_subdivision_shift);
    int layer_offset = probe_data.layer * probes_per_layer_;
    for (int y : IndexRange(spots_per_dimension)) {
      for (int x : IndexRange(spots_per_dimension)) {
        int2 pos = pos_in_location_finder + int2(x, y);
        int area_index = pos.x + pos.y * probes_per_dimension_;
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
  ReflectionProbeData first_free_spot() const
  {
    ReflectionProbeData result = {};
    result.layer_subdivision = subdivision_level_;
    for (int index : taken_spots_.index_range()) {
      if (!taken_spots_[index]) {
        int layer = index / probes_per_layer_;
        int area_index = index % probes_per_layer_;
        result.layer = layer;
        result.area_index = area_index;
        return result;
      }
    }

    BLI_assert_unreachable();
    return result;
  }
};

ReflectionProbeData ReflectionProbeModule::find_empty_reflection_probe_data(
    int subdivision_level) const
{
  ProbeLocationFinder location_finder(needed_layers_get() + 1, subdivision_level);
  for (const ReflectionProbeData &data :
       Span<ReflectionProbeData>(data_buf_.data(), reflection_probe_data_index_max() + 1))
  {
    location_finder.mark_space_used(data);
  }
  return location_finder.first_free_spot();
}

void ReflectionProbeModule::end_sync()
{
  remove_unused_probes();
  data_buf_.push_update();

#if true
  debug_print();
  validate();
#endif

  int number_layers_needed = needed_layers_get();
  int current_layers = cubemaps_tx_.depth() / 6;
  bool resize_layers = current_layers < number_layers_needed;
  if (resize_layers) {
    cubemaps_tx_.ensure_cube_array(GPU_RGBA16F,
                                   MAX_RESOLUTION,
                                   number_layers_needed,
                                   GPU_TEXTURE_USAGE_SHADER_READ | GPU_TEXTURE_USAGE_ATTACHMENT,
                                   NULL,
                                   MIPMAP_LEVELS);
  }

  /* Regenerate mipmaps when a cubemap is updated. It can be postponed when the world probe is also
   * updated. In this case it would happen as part of the WorldProbePipeline. */
  bool regenerate_mipmaps = false;
  bool regenerate_mipmaps_postponed = false;

  for (ReflectionProbe &cubemap : cubemaps_) {
    cubemap.is_dirty |= resize_layers;
    if (!cubemap.needs_update()) {
      continue;
    }
    sync(cubemap);
    cubemap.is_dirty = false;
    if (cubemap.type == ReflectionProbe::Type::WORLD) {
      regenerate_mipmaps_postponed = true;
    }
    else {
      regenerate_mipmaps = true;
    }
  }

  if (regenerate_mipmaps) {
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
    if (!regenerate_mipmaps_postponed) {
      GPU_texture_update_mipmap_chain(cubemaps_tx_);
    }
  }
}

void ReflectionProbeModule::remove_unused_probes()
{
  for (ReflectionProbe &probe : cubemaps_) {
    if (probe.type == ReflectionProbe::Type::PROBE && !probe.is_used) {
      probe.type = ReflectionProbe::Type::UNUSED;
      probe.object_hash_value = 0;
      BLI_assert(probe.index != -1);
      remove_reflection_probe_data(probe.index);
    }
  }
}

void ReflectionProbeModule::remove_reflection_probe_data(int reflection_probe_data_index)
{
  int max_index = reflection_probe_data_index_max() + 1;
  for (int index = reflection_probe_data_index; index < max_index - 1; index++) {
    data_buf_[index] = data_buf_[index + 1];
  }
  for (ReflectionProbe &probe : cubemaps_) {
    if (probe.index == reflection_probe_data_index) {
      probe.index = -1;
    }
    if (probe.index > reflection_probe_data_index) {
      probe.index--;
    }
  }
  BLI_assert(reflection_probe_data_index_max() + 1 == max_index - 1);
}

void ReflectionProbeModule::debug_print() const
{
  std::stringstream out;

  out << "\n *** " << __func__ << " ***\n";
  for (const ReflectionProbe &probe : cubemaps_) {
    switch (probe.type) {
      case ReflectionProbe::Type::UNUSED: {
        out << "UNUSED\n";

        break;
      }
      case ReflectionProbe::Type::WORLD: {
        out << "WORLD";
        out << " is_dirty: " << probe.is_dirty;
        out << " index: " << probe.index;
        out << "\n";
        break;
      }
      case ReflectionProbe::Type::PROBE: {
        out << "PROBE";
        out << " is_dirty: " << probe.is_dirty;
        out << " is_used: " << probe.is_used;
        out << " ob_hash: " << probe.object_hash_value;
        out << " index: " << probe.index;
        out << "\n";
        break;
      }
    }

    if (probe.index != -1) {
      const ReflectionProbeData &data = data_buf_[probe.index];
      out << " - layer: " << data.layer;
      out << " subdivision: " << data.layer_subdivision;
      out << " area: " << data.area_index;
      out << "\n";
    }
  }

  printf("%s", out.str().c_str());
}

void ReflectionProbeModule::validate() const
{
  for (const ReflectionProbe &probe : cubemaps_) {
    switch (probe.type) {
      case ReflectionProbe::Type::UNUSED: {
        BLI_assert(probe.index == -1);
        break;
      }
      case ReflectionProbe::Type::WORLD: {
        BLI_assert(probe.index != -1);
        break;
      }
      case ReflectionProbe::Type::PROBE: {
        BLI_assert(probe.index != -1);
        break;
      }
    }
  }
}

/* -------------------------------------------------------------------- */
/** \name World
 *
 * \{ */

bool ReflectionProbe::needs_update() const
{
  switch (type) {
    case Type::UNUSED:
      return false;
    case Type::WORLD:
      return is_dirty;
    case Type::PROBE:
      return is_dirty && is_used;
  }
  return false;
}

/** \} */

void ReflectionProbeModule::upload_dummy_cubemap(const ReflectionProbe &probe)
{
  const ReflectionProbeData &probe_data = data_buf_[probe.index];
  const int resolution = MAX_RESOLUTION >> probe_data.layer_subdivision;
  float4 *data = static_cast<float4 *>(
      MEM_mallocN(sizeof(float4) * resolution * resolution, __func__));

  /* Generate dummy checker pattern. */
  int index = 0;
  const int BLOCK_SIZE = max_ii(256 >> probe_data.layer_subdivision, 1);
  for (int y : IndexRange(resolution)) {
    for (int x : IndexRange(resolution)) {
      int tx = (x / BLOCK_SIZE) & 1;
      int ty = (y / BLOCK_SIZE) & 1;
      bool solid = (tx + ty) & 1;
      if (solid) {
        data[index] = float4(probe_data.color.x, probe_data.color.y, probe_data.color.z, 1.0f);
      }
      else {
        data[index] = float4(0.0f);
      }

      index++;
    }
  }

  /* Upload the checker pattern to each side of the cubemap*/
  int probes_per_dimension = 1 << probe_data.layer_subdivision;
  int2 probe_area_pos(probe_data.area_index % probes_per_dimension,
                      probe_data.area_index / probes_per_dimension);
  int2 pos = probe_area_pos * int2(MAX_RESOLUTION / probes_per_dimension);
  for (int side : IndexRange(6)) {
    int layer = 6 * probe_data.layer + side;
    GPU_texture_update_sub(
        cubemaps_tx_, GPU_DATA_FLOAT, data, UNPACK2(pos), layer, resolution, resolution, 1);
  }

  MEM_freeN(data);
}

}  // namespace blender::eevee
