/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_geometry_nodes_serialize.hh"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string_utils.h"

namespace blender::bke::bake {

std::string frame_to_file_name(const SubFrame &frame)
{
  char file_name_c[FILE_MAX];
  SNPRINTF(file_name_c, "%011.5f", double(frame));
  BLI_string_replace_char(file_name_c, '.', '_');
  return file_name_c;
}

std::optional<SubFrame> file_name_to_frame(const StringRefNull file_name)
{
  char modified_file_name[FILE_MAX];
  STRNCPY(modified_file_name, file_name.c_str());
  BLI_string_replace_char(modified_file_name, '_', '.');
  const SubFrame frame = std::stof(modified_file_name);
  return frame;
}

Vector<MetaFile> find_sorted_meta_files(const StringRefNull meta_dir)
{
  if (!BLI_is_dir(meta_dir.c_str())) {
    return {};
  }

  direntry *dir_entries = nullptr;
  const int dir_entries_num = BLI_filelist_dir_contents(meta_dir.c_str(), &dir_entries);
  BLI_SCOPED_DEFER([&]() { BLI_filelist_free(dir_entries, dir_entries_num); });

  Vector<MetaFile> meta_files;
  for (const int i : IndexRange(dir_entries_num)) {
    const direntry &dir_entry = dir_entries[i];
    const StringRefNull dir_entry_path = dir_entry.path;
    if (!dir_entry_path.endswith(".json")) {
      continue;
    }
    const std::optional<SubFrame> frame = file_name_to_frame(dir_entry.relname);
    if (!frame) {
      continue;
    }
    meta_files.append({*frame, dir_entry_path});
  }

  std::sort(meta_files.begin(), meta_files.end(), [](const MetaFile &a, const MetaFile &b) {
    return a.frame < b.frame;
  });

  return meta_files;
}

BakePath BakePath::from_single_root(StringRefNull root_dir)
{
  char meta_dir[FILE_MAX];
  BLI_path_join(meta_dir, sizeof(meta_dir), root_dir.c_str(), "meta");
  char bdata_dir[FILE_MAX];
  BLI_path_join(bdata_dir, sizeof(bdata_dir), root_dir.c_str(), "bdata");

  BakePath bake_path;
  bake_path.meta_dir = meta_dir;
  bake_path.bdata_dir = bdata_dir;
  return bake_path;
}

#define SERIALIZE_VERSION 1

std::shared_ptr<io::serialize::Value> serialize_bake_node_state(const BakeNodeState &state,
                                                                BDataWriter &bdata_writer,
                                                                BDataSharing &bdata_sharing)
{
  auto io_root = std::make_shared<io::serialize::DictionaryValue>();
  io_root->append_int("version", SERIALIZE_VERSION);
  auto io_bake_items = io_root->append_array("bake_items");
  for (const auto item : state.item_by_identifier.items()) {
    const int id = item.key;
    auto io_bake_item = io_bake_items->append_dict();
    io_bake_item->append_int("id", id);
    bke::serialize_bake_item(*item.value, bdata_writer, bdata_sharing, *io_bake_item);
  }
  return io_root;
}

bool deserialize_bake_node_state(const io::serialize::Value &io_root,
                                 const BDataReader &bdata_reader,
                                 const BDataSharing &bdata_sharing,
                                 BakeNodeState &r_state)
{
  r_state.item_by_identifier.clear();
  const auto *io_root_dict = io_root.as_dictionary_value();
  if (!io_root_dict) {
    return false;
  }
  const std::optional<int> version = io_root_dict->lookup_int("version");
  if (!version) {
    return false;
  }
  if (*version != SERIALIZE_VERSION) {
    return false;
  }
  auto io_bake_items = io_root_dict->lookup_array("bake_items");
  if (!io_bake_items) {
    return false;
  }
  Vector<std::pair<std::unique_ptr<BakeItem>, int>> loaded_items;
  for (const auto &io_bake_item : io_bake_items->elements()) {
    const auto *io_bake_item_dict = io_bake_item->as_dictionary_value();
    if (!io_bake_item_dict) {
      return false;
    }
    const std::optional<int> id = io_bake_item_dict->lookup_int("id");
    if (!id) {
      return false;
    }
    std::unique_ptr<BakeItem> bake_item = bke::deserialize_bake_item(
        *io_bake_item_dict, bdata_reader, bdata_sharing);
    if (!bake_item) {
      return false;
    }
    loaded_items.append({std::move(bake_item), *id});
  }
  for (auto &item : loaded_items) {
    r_state.item_by_identifier.add(item.second, std::move(item.first));
  }
  return true;
}

bool serialize_bake_node_state_to_disk(const SubFrame &frame,
                                       const BakeNodeState &state,
                                       const BakePath &bake_path,
                                       BDataSharing &bdata_sharing)
{
  const std::string frame_str = frame_to_file_name(frame);
  const std::string meta_file_name = frame_str + ".json";
  char meta_file_path[FILE_MAX];
  BLI_path_join(
      meta_file_path, sizeof(meta_file_path), bake_path.meta_dir.c_str(), meta_file_name.c_str());

  char bdata_file_name[FILE_MAX];
  char bdata_file_path[FILE_MAX];
  int bdata_check_i = 0;
  while (true) {
    if (bdata_check_i == 0) {
      SNPRINTF(bdata_file_name, "%s.bdata", frame_str.c_str());
    }
    else {
      SNPRINTF(bdata_file_name, "%s_%d.bdata", frame_str.c_str(), bdata_check_i);
    }
    BLI_path_join(
        bdata_file_path, sizeof(bdata_file_path), bake_path.bdata_dir.c_str(), bdata_file_name);
    if (!BLI_exists(bdata_file_path)) {
      break;
    }
    bdata_check_i++;
  }

  BLI_file_ensure_parent_dir_exists(meta_file_path);
  BLI_file_ensure_parent_dir_exists(bdata_file_path);

  fstream bdata_file{bdata_file_path, std::ios::out | std::ios::binary};
  DiskBDataWriter bdata_writer{bdata_file_name, bdata_file, 0};
  std::shared_ptr<io::serialize::Value> io_root = serialize_bake_node_state(
      state, bdata_writer, bdata_sharing);
  io::serialize::write_json_file(meta_file_path, *io_root);
  return true;
}

bool deserialize_bake_node_state_from_disk(const StringRefNull meta_file_path,
                                           const StringRefNull bdata_dir,
                                           const BDataSharing &bdata_sharing,
                                           BakeNodeState &r_state)
{
  const std::shared_ptr<io::serialize::Value> io_root = io::serialize::read_json_file(
      meta_file_path);
  if (!io_root) {
    return false;
  }
  const DiskBDataReader bdata_reader(bdata_dir);
  return deserialize_bake_node_state(*io_root, bdata_reader, bdata_sharing, r_state);
}

}  // namespace blender::bke::bake
