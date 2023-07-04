/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_reflection_probes.hh"
#include "eevee_instance.hh"

namespace blender::eevee {

void ReflectionProbeModule::init()
{
  if (probes_.is_empty()) {
    ReflectionProbeData init_probe_data = {};
    init_probe_data.layer = -1;
    for (int i : IndexRange(REFLECTION_PROBES_MAX)) {
      data_buf_[i] = init_probe_data;
    }

    /* Initialize the world cubemap. */
    ReflectionProbeData world_probe_data{};
    world_probe_data.layer = 0;
    world_probe_data.layer_subdivision = world_subdivision_level_;
    world_probe_data.area_index = 1;
    world_probe_data.color = float4(0.0f);
    world_probe_data.pos = float3(0.0f);
    world_probe_data.intensity = 1.0f;
    data_buf_[0] = world_probe_data;

    ReflectionProbe world_cubemap;
    world_cubemap.type = ReflectionProbe::Type::World;
    world_cubemap.is_dirty = true;
    world_cubemap.is_probes_tx_dirty = true;
    world_cubemap.index = 0;
    probes_.append(world_cubemap);

    const int max_mipmap_levels = log(max_resolution_) + 1;
    probes_tx_.ensure_2d_array(GPU_RGBA16F,
                               int2(max_resolution_),
                               init_num_probes_,
                               GPU_TEXTURE_USAGE_SHADER_WRITE,
                               nullptr,
                               max_mipmap_levels);
    GPU_texture_mipmap_mode(probes_tx_, true, true);

    /* Cubemap is half of the resolution of the octahedral map. */
    cubemap_tx_.ensure_cube(
        GPU_RGBA16F, max_resolution_ / 2, GPU_TEXTURE_USAGE_ATTACHMENT, nullptr, 1);
    GPU_texture_mipmap_mode(cubemap_tx_, false, true);
  }

  {
    PassSimple &pass = remap_ps_;
    pass.init();
    pass.shader_set(instance_.shaders.static_shader_get(REFLECTION_PROBE_REMAP));
    pass.bind_texture("cubemap_tx", cubemap_tx_);
    pass.bind_image("octahedral_img", probes_tx_);
    pass.bind_ssbo(REFLECTION_PROBE_BUF_SLOT, data_buf_);
    pass.dispatch(int2(ceil_division(max_resolution_, REFLECTION_PROBE_GROUP_SIZE)));
  }
}
void ReflectionProbeModule::begin_sync()
{
  for (ReflectionProbe &reflection_probe : probes_) {
    if (reflection_probe.type == ReflectionProbe::Type::Probe) {
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
    case ReflectionProbe::Type::World: {
      break;
    }
    case ReflectionProbe::Type::Probe: {
      // TODO: upload the baked probe to the cubemaps.
      if (cubemap.is_probes_tx_dirty) {
        upload_dummy_cubemap(cubemap);
      }
      break;
    }
    case ReflectionProbe::Type::Unused: {
      break;
    }
  }
}

void ReflectionProbeModule::sync_object(Object *ob, ObjectHandle &ob_handle)
{
  const ::LightProbe *light_probe = (::LightProbe *)ob->data;
  if (light_probe->type != LIGHTPROBE_TYPE_CUBE) {
    return;
  }
  const bool is_dirty = ob_handle.recalc != 0;
  ReflectionProbe &probe = find_or_insert(ob_handle, reflection_probe_subdivision_level_);
  probe.is_dirty |= is_dirty;
  probe.is_used = true;

  /* TODO: remove debug color.*/
  ReflectionProbeData &probe_data = data_buf_[probe.index];
  probe_data.color = ob->color;
  probe_data.pos = float3(float4x4(ob->object_to_world) * float4(0.0, 0.0, 0.0, 1.0));
  probe_data.intensity = light_probe->intensity;
}

ReflectionProbe &ReflectionProbeModule::find_or_insert(ObjectHandle &ob_handle,
                                                       int subdivision_level)
{
  ReflectionProbe *first_unused = nullptr;
  for (ReflectionProbe &reflection_probe : probes_) {
    if (reflection_probe.type == ReflectionProbe::Type::Probe &&
        reflection_probe.object_hash_value == ob_handle.object_key.hash_value)
    {
      return reflection_probe;
    }
    if (first_unused == nullptr && reflection_probe.type == ReflectionProbe::Type::Unused) {
      first_unused = &reflection_probe;
    }
  }

  if (first_unused == nullptr) {
    ReflectionProbe new_slot{};
    probes_.append_as(new_slot);
    first_unused = &probes_.last();
  }
  BLI_assert(first_unused != nullptr);
  first_unused->index = -1;
  ReflectionProbeData probe_data = find_empty_reflection_probe_data(subdivision_level);

  first_unused->is_dirty = true;
  first_unused->is_probes_tx_dirty = true;
  first_unused->object_hash_value = ob_handle.object_key.hash_value;
  first_unused->type = ReflectionProbe::Type::Probe;
  first_unused->index = reflection_probe_data_index_max() + 1;

  data_buf_[first_unused->index] = probe_data;

  return *first_unused;
}

int ReflectionProbeModule::reflection_probe_data_index_max() const
{
  int result = -1;
  for (const ReflectionProbe &probe : probes_) {
    if (probe.type != ReflectionProbe::Type::Unused) {
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
  int current_layers = probes_tx_.depth();
  bool resize_layers = current_layers < number_layers_needed;
  if (resize_layers) {
    /* TODO: Create new texture and copy previous texture so we don't need to rerender all the
     * probes.*/
    const int max_mipmap_levels = log(max_resolution_) + 1;
    probes_tx_.ensure_2d_array(GPU_RGBA16F,
                               int2(max_resolution_),
                               number_layers_needed,
                               GPU_TEXTURE_USAGE_SHADER_WRITE,
                               nullptr,
                               max_mipmap_levels);
    GPU_texture_mipmap_mode(probes_tx_, true, true);
  }

  /* Regenerate mipmaps when a cubemap is updated. It can be postponed when the world probe is also
   * updated. In this case it would happen as part of the WorldProbePipeline. */
  bool regenerate_mipmaps = false;
  bool regenerate_mipmaps_postponed = false;

  for (ReflectionProbe &cubemap : probes_) {
    cubemap.is_dirty |= resize_layers;
    cubemap.is_probes_tx_dirty |= resize_layers;

    if (!cubemap.needs_update()) {
      continue;
    }
    sync(cubemap);

    switch (cubemap.type) {
      case ReflectionProbe::Type::World:
        regenerate_mipmaps_postponed = true;
        break;

      case ReflectionProbe::Type::Probe:
        regenerate_mipmaps = cubemap.is_probes_tx_dirty;
        break;

      case ReflectionProbe::Type::Unused:
        BLI_assert_unreachable();
        break;
    }
    cubemap.is_dirty = false;
    cubemap.is_probes_tx_dirty = false;
  }

  if (regenerate_mipmaps) {
    GPU_memory_barrier(GPU_BARRIER_TEXTURE_UPDATE);
    if (!regenerate_mipmaps_postponed) {
      GPU_texture_update_mipmap_chain(probes_tx_);
    }
  }
}

void ReflectionProbeModule::remove_unused_probes()
{
  for (ReflectionProbe &probe : probes_) {
    if (probe.type == ReflectionProbe::Type::Probe && !probe.is_used) {
      remove_reflection_probe_data(probe.index);
      probe.type = ReflectionProbe::Type::Unused;
      probe.object_hash_value = 0;
      BLI_assert(probe.index == -1);
    }
  }
}

void ReflectionProbeModule::remove_reflection_probe_data(int reflection_probe_data_index)
{
  int max_index = reflection_probe_data_index_max();
  BLI_assert_msg(reflection_probe_data_index <= max_index,
                 "Trying to remove reflection probes when it isn't part of the reflection probe "
                 "data. This can also happens when the state is set to "
                 "ReflectionProbe::Type::UNUSED, before removing the data.");
  for (int index = reflection_probe_data_index; index < max_index; index++) {
    data_buf_[index] = data_buf_[index + 1];
  }
  for (ReflectionProbe &probe : probes_) {
    if (probe.index == reflection_probe_data_index) {
      probe.index = -1;
    }
    if (probe.index > reflection_probe_data_index) {
      probe.index--;
    }
  }
  data_buf_[max_index].layer = -1;
  BLI_assert(reflection_probe_data_index_max() == max_index - 1);
}

void ReflectionProbeModule::debug_print() const
{
  std::stringstream out;

  out << "\n *** " << __func__ << " ***\n";
  for (const ReflectionProbe &probe : probes_) {
    switch (probe.type) {
      case ReflectionProbe::Type::Unused: {
        out << "UNUSED\n";

        break;
      }
      case ReflectionProbe::Type::World: {
        out << "WORLD";
        out << " is_dirty: " << probe.is_dirty;
        out << " index: " << probe.index;
        out << "\n";
        break;
      }
      case ReflectionProbe::Type::Probe: {
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
      out << "   intensity: " << data.intensity;
      out << "\n";
    }
  }

  printf("%s", out.str().c_str());
}

void ReflectionProbeModule::validate() const
{
  for (const ReflectionProbe &probe : probes_) {
    switch (probe.type) {
      case ReflectionProbe::Type::Unused: {
        BLI_assert(probe.index == -1);
        break;
      }
      case ReflectionProbe::Type::World: {
        BLI_assert(probe.index != -1);
        break;
      }
      case ReflectionProbe::Type::Probe: {
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
    case Type::Unused:
      return false;
    case Type::World:
      return is_dirty;
    case Type::Probe:
      return is_dirty && is_used;
  }
  return false;
}

/** \} */

void ReflectionProbeModule::upload_dummy_cubemap(const ReflectionProbe &probe)
{
  const ReflectionProbeData &probe_data = data_buf_[probe.index];
  const int resolution = max_resolution_ >> probe_data.layer_subdivision;
  float4 *data = static_cast<float4 *>(
      MEM_mallocN(sizeof(float4) * resolution * resolution, __func__));

  /* Generate dummy checker pattern. */
  int index = 0;
  const int BLOCK_SIZE = max_ii(1024 >> probe_data.layer_subdivision, 1);
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
  /* TODO: Apply octahedral mapping. */
  int probes_per_dimension = 1 << probe_data.layer_subdivision;
  int2 probe_area_pos(probe_data.area_index % probes_per_dimension,
                      probe_data.area_index / probes_per_dimension);
  int2 pos = probe_area_pos * int2(max_resolution_ / probes_per_dimension);
  GPU_texture_update_sub(
      probes_tx_, GPU_DATA_FLOAT, data, UNPACK2(pos), probe_data.layer, resolution, resolution, 1);

  MEM_freeN(data);
}

void ReflectionProbeModule::remap_to_octahedral_projection()
{
  instance_.manager->submit(remap_ps_);
  GPU_texture_update_mipmap_chain(probes_tx_);
}

}  // namespace blender::eevee
