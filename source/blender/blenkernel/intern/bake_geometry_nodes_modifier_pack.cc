/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_bake_geometry_nodes_modifier_pack.hh"
#include "BKE_packedFile.h"
#include "BKE_report.hh"

#include "BLI_fileops.hh"
#include "BLI_path_util.h"
#include "BLI_string.h"

namespace blender::bke::bake {

static Vector<NodesModifierBakeFile> pack_files_from_directory(const StringRefNull directory,
                                                               ReportList *reports)
{
  if (!BLI_is_dir(directory.c_str())) {
    BKE_reportf(reports, RPT_ERROR, "%s is no directory", directory.c_str());
    return {};
  }

  direntry *dir_entries = nullptr;
  const int dir_entries_num = BLI_filelist_dir_contents(directory.c_str(), &dir_entries);
  BLI_SCOPED_DEFER([&]() { BLI_filelist_free(dir_entries, dir_entries_num); });

  Vector<NodesModifierBakeFile> bake_files;
  for (const int i : IndexRange(dir_entries_num)) {
    const direntry &dir_entry = dir_entries[i];
    const StringRefNull dir_entry_path = dir_entry.path;
    const StringRefNull name = dir_entry.relname;
    NodesModifierBakeFile bake_file;
    bake_file.name = BLI_strdup_null(name.c_str());
    bake_file.packed_file = BKE_packedfile_new(reports, dir_entry_path.c_str(), "");
    if (bake_file.packed_file) {
      bake_files.append(bake_file);
    }
  }

  return bake_files;
}

NodesModifierPackedBake *pack_bake_from_disk(const BakePath &bake_path, ReportList *reports)
{
  const Vector<NodesModifierBakeFile> meta_bake_files = pack_files_from_directory(
      bake_path.meta_dir, reports);
  if (meta_bake_files.is_empty()) {
    return nullptr;
  }

  const Vector<NodesModifierBakeFile> blob_bake_files = pack_files_from_directory(
      bake_path.blobs_dir, reports);

  NodesModifierPackedBake *packed_bake = MEM_cnew<NodesModifierPackedBake>(__func__);
  packed_bake->meta_files_num = meta_bake_files.size();
  packed_bake->blob_files_num = blob_bake_files.size();

  packed_bake->meta_files = MEM_cnew_array<NodesModifierBakeFile>(packed_bake->meta_files_num,
                                                                  __func__);
  packed_bake->blob_files = MEM_cnew_array<NodesModifierBakeFile>(packed_bake->blob_files_num,
                                                                  __func__);

  uninitialized_copy_n(meta_bake_files.data(), meta_bake_files.size(), packed_bake->meta_files);
  uninitialized_copy_n(blob_bake_files.data(), blob_bake_files.size(), packed_bake->blob_files);

  return packed_bake;
}

bool unpack_bake_to_disk(const NodesModifierPackedBake &packed_bake,
                         const BakePath &bake_path,
                         ReportList *reports)
{
  return false;
}

}  // namespace blender::bke::bake
