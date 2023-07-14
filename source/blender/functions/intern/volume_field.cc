/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "FN_volume_field.hh"

#ifdef WITH_OPENVDB

namespace blender::volume_mask {

bool VolumeMask::is_empty() const
{
  return grid_.empty();
}

int64_t VolumeMask::min_voxel_count() const
{
  return grid_.activeVoxelCount();
}

}  // namespace blender::volume_mask

namespace blender::fn {

}  // namespace blender::fn

#else

namespace blender::volume_mask {

bool VolumeMask::is_empty() const
{
  return true;
}

int64_t VolumeMask::min_voxel_count() const
{
  return 0;
}

}  // namespace blender::volume_mask

#endif
