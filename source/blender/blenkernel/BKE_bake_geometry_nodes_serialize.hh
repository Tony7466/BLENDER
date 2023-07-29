/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <optional>

#include "BKE_bake_geometry_nodes.hh"
#include "BKE_bake_items_serialize.hh"

namespace blender::bke::bake {

struct MetaFile {
  SubFrame frame;
  std::string path;
};

struct BakePath {
  std::string meta_dir;
  std::string bdata_dir;

  static BakePath from_single_root(StringRefNull root_dir);
};

std::string frame_to_file_name(const SubFrame &frame);
std::optional<SubFrame> file_name_to_frame(const StringRefNull file_name);

Vector<MetaFile> find_sorted_meta_files(const StringRefNull meta_dir);

std::shared_ptr<io::serialize::Value> serialize_bake_node_state(const BakeNodeState &state,
                                                                BDataWriter &bdata_writer,
                                                                BDataSharing &bdata_sharing);

bool deserialize_bake_node_state(const io::serialize::Value &io_root,
                                 const BDataReader &bdata_reader,
                                 const BDataSharing &bdata_sharing,
                                 BakeNodeState &r_state);

bool serialize_bake_node_state_to_disk(const SubFrame &frame,
                                       const BakeNodeState &state,
                                       const BakePath &bake_path,
                                       BDataSharing &bdata_sharing);

bool deserialize_bake_node_state_from_disk(StringRefNull meta_file_path,
                                           StringRefNull bdata_dir,
                                           const BDataSharing &bdata_sharing,
                                           BakeNodeState &r_state);

void try_discover_bake_on_disk(const BakePath &bake_path, BakeNodeStorage &r_bake_storage);

}  // namespace blender::bke::bake
