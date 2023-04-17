/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_simulation_state.hh"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"

namespace blender::ed::object::bake_simulation {
void load_simulation_state(const StringRefNull meta_path,
                           const StringRefNull bdata_dir,
                           bke::sim::ModifierSimulationState &r_state);
}

namespace blender::bke::sim {

void ModifierSimulationCache::load_baked_states(const StringRefNull meta_dir,
                                                const StringRefNull bdata_dir)
{
  this->reset();

  direntry *dir_entries = nullptr;
  const int dir_entries_num = BLI_filelist_dir_contents(meta_dir.c_str(), &dir_entries);
  BLI_SCOPED_DEFER([&]() { BLI_filelist_free(dir_entries, dir_entries_num); });

  for (const int i : IndexRange(dir_entries_num)) {
    const direntry &dir_entry = dir_entries[i];
    const StringRefNull dir_entry_path = dir_entry.path;
    if (!dir_entry_path.endswith(".json")) {
      continue;
    }
    char modified_file_name[FILENAME_MAX];
    BLI_strncpy(modified_file_name, dir_entry.relname, sizeof(modified_file_name));
    BLI_str_replace_char(modified_file_name, '_', '.');

    const SubFrame frame = std::stof(modified_file_name);

    auto new_state_at_frame = std::make_unique<ModifierSimulationStateAtFrame>();
    new_state_at_frame->frame = frame;
    ed::object::bake_simulation::load_simulation_state(
        dir_entry.path, bdata_dir, new_state_at_frame->state);
    states_at_frames_.append(std::move(new_state_at_frame));
  }

  cache_state_ = CacheState::Baked;
}

}  // namespace blender::bke::sim
