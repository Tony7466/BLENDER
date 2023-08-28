/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collection.h"
#include "BKE_curves.hh"
#include "BKE_main.h"
#include "BKE_simulation_state.hh"
#include "BKE_simulation_state_serialize.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"
#include "DNA_pointcloud_types.h"

#include "BLI_binary_search.hh"
#include "BLI_fileops.hh"
#include "BLI_hash_md5.h"
#include "BLI_path_util.h"
#include "BLI_string_utils.h"

#include "MOD_nodes.hh"

namespace blender::bke::sim {

void scene_simulation_states_reset(Scene &scene)
{
  FOREACH_SCENE_OBJECT_BEGIN (&scene, ob) {
    LISTBASE_FOREACH (ModifierData *, md, &ob->modifiers) {
      if (md->type != eModifierType_Nodes) {
        continue;
      }
      NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
      // TODO
      // nmd->runtime->simulation_cache->reset();
    }
  }
  FOREACH_SCENE_OBJECT_END;
}

std::optional<bake_paths::BakePath> get_simulation_zone_bake_path(const Main &bmain,
                                                                  const Object &object,
                                                                  const NodesModifierData &nmd,
                                                                  int zone_id)
{
  const StringRefNull bmain_path = BKE_main_blendfile_path(&bmain);
  if (bmain_path.is_empty()) {
    return std::nullopt;
  }
  if (StringRef(nmd.simulation_bake_directory).is_empty()) {
    return std::nullopt;
  }
  const char *base_path = ID_BLEND_PATH(&bmain, &object.id);
  char absolute_bake_dir[FILE_MAX];
  STRNCPY(absolute_bake_dir, nmd.simulation_bake_directory);
  BLI_path_abs(absolute_bake_dir, base_path);

  char zone_bake_dir[FILE_MAX];
  BLI_path_join(
      zone_bake_dir, sizeof(zone_bake_dir), absolute_bake_dir, std::to_string(zone_id).c_str());
  return bake_paths::BakePath::from_single_root(zone_bake_dir);
}

}  // namespace blender::bke::sim
