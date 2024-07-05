/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <sstream>

#include "BKE_bake_geometry_nodes_modifier.hh"
#include "BKE_collection.hh"
#include "BKE_main.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "BLI_binary_search.hh"
#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string.h"

#include "MOD_nodes.hh"

namespace blender::bke::bake {

void SimulationNodeCache::reset()
{
  std::destroy_at(this);
  new (this) SimulationNodeCache();
}

void BakeNodeCache::reset()
{
  std::destroy_at(this);
  new (this) BakeNodeCache();
}

void NodeBakeCache::reset()
{
  std::destroy_at(this);
  new (this) NodeBakeCache();
}

IndexRange NodeBakeCache::frame_range() const
{
  if (this->frames.is_empty()) {
    return {};
  }
  const int start_frame = this->frames.first()->frame.frame();
  const int end_frame = this->frames.last()->frame.frame();
  return IndexRange::from_begin_end_inclusive(start_frame, end_frame);
}

SimulationNodeCache *ModifierCache::get_simulation_node_cache(const int id)
{
  std::unique_ptr<SimulationNodeCache> *ptr = this->simulation_cache_by_id.lookup_ptr(id);
  return ptr ? (*ptr).get() : nullptr;
}

BakeNodeCache *ModifierCache::get_bake_node_cache(const int id)
{
  std::unique_ptr<BakeNodeCache> *ptr = this->bake_cache_by_id.lookup_ptr(id);
  return ptr ? (*ptr).get() : nullptr;
}

NodeBakeCache *ModifierCache::get_node_bake_cache(const int id)
{
  if (SimulationNodeCache *cache = this->get_simulation_node_cache(id)) {
    return &cache->bake;
  }
  if (BakeNodeCache *cache = this->get_bake_node_cache(id)) {
    return &cache->bake;
  }
  return nullptr;
}

void scene_simulation_states_reset(Scene &scene)
{
  FOREACH_SCENE_OBJECT_BEGIN (&scene, ob) {
    LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
      if (md->type != eModifierType_Nodes) {
        continue;
      }
      NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
      if (!nmd->runtime->cache) {
        continue;
      }
      for (auto item : nmd->runtime->cache->simulation_cache_by_id.items()) {
        item.value->reset();
      }
    }
  }
  FOREACH_SCENE_OBJECT_END;
}

std::optional<std::string> get_modifier_bake_path(const Main &bmain,
                                                  const Object &object,
                                                  const NodesModifierData &nmd)
{
  if (StringRef(nmd.bake_directory).is_empty()) {
    return std::nullopt;
  }
  const char *base_path = ID_BLEND_PATH(&bmain, &object.id);
  if (StringRef(base_path).is_empty()) {
    return std::nullopt;
  }
  char absolute_bake_dir[FILE_MAX];
  STRNCPY(absolute_bake_dir, nmd.bake_directory);
  BLI_path_abs(absolute_bake_dir, base_path);
  return absolute_bake_dir;
}

std::optional<bake::BakePath> get_node_bake_path(const Main &bmain,
                                                 const Object &object,
                                                 const NodesModifierData &nmd,
                                                 int node_id)
{
  const NodesModifierBake *bake = nmd.find_bake(node_id);
  if (bake == nullptr) {
    return std::nullopt;
  }
  if (bake->flag & NODES_MODIFIER_BAKE_CUSTOM_PATH) {
    if (StringRef(bake->directory).is_empty()) {
      return std::nullopt;
    }
    const char *base_path = ID_BLEND_PATH(&bmain, &object.id);
    if (StringRef(base_path).is_empty()) {
      return std::nullopt;
    }
    char absolute_bake_dir[FILE_MAX];
    STRNCPY(absolute_bake_dir, bake->directory);
    BLI_path_abs(absolute_bake_dir, base_path);
    return bake::BakePath::from_single_root(absolute_bake_dir);
  }
  const std::optional<std::string> modifier_bake_path = get_modifier_bake_path(bmain, object, nmd);
  if (!modifier_bake_path) {
    return std::nullopt;
  }
  char bake_dir[FILE_MAX];
  BLI_path_join(
      bake_dir, sizeof(bake_dir), modifier_bake_path->c_str(), std::to_string(node_id).c_str());
  return bake::BakePath::from_single_root(bake_dir);
}

static IndexRange fix_frame_range(const int start, const int end)
{
  const int num_frames = std::max(1, end - start + 1);
  return IndexRange(start, num_frames);
}

std::optional<IndexRange> get_node_bake_frame_range(const Scene &scene,
                                                    const Object & /*object*/,
                                                    const NodesModifierData &nmd,
                                                    int node_id)
{
  const NodesModifierBake *bake = nmd.find_bake(node_id);
  if (bake == nullptr) {
    return std::nullopt;
  }
  if (bake->flag & NODES_MODIFIER_BAKE_CUSTOM_SIMULATION_FRAME_RANGE) {
    return fix_frame_range(bake->frame_start, bake->frame_end);
  }
  if (scene.flag & SCE_CUSTOM_SIMULATION_RANGE) {
    return fix_frame_range(scene.simulation_frame_start, scene.simulation_frame_end);
  }
  return fix_frame_range(scene.r.sfra, scene.r.efra);
}

}  // namespace blender::bke::bake
