/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#pragma once

namespace blender {
class CPPType;
class ResourceScope;
template<typename T> class VArray;
}  // namespace blender

namespace blender::volume_mask {

/* XXX Placeholder, this will define the grid voxels on which volume fields are evaluated. */
class VolumeMask {
#ifdef WITH_OPENVDB
  const openvdb::MaskGrid &grid_;
#endif

 public:
  bool is_empty() const;
  int64_t min_voxel_count() const;
};

}  // namespace blender::volume_mask

namespace blender::fn {

using volume_mask::VolumeMask;

struct VolumeGrid {
#ifdef WITH_OPENVDB
  openvdb::GridBase::Ptr grid_ = nullptr;
#endif

  static VolumeGrid create(ResourceScope &scope, const CPPType &type, int64_t voxel_count);
  static VolumeGrid create(ResourceScope &scope,
                           const CPPType &type,
                           const void *background_value);

  operator bool() const
  {
#ifdef WITH_OPENVDB
    return grid_ != nullptr;
#else
    return false;
#endif
  }

  int64_t voxel_count() const
  {
#ifdef WITH_OPENVDB
    return grid_ ? grid_->activeVoxelCount() : 0;
#else
    return 0;
#endif
  }

  bool is_empty() const
  {
#ifdef WITH_OPENVDB
    grid_ ? grid_->empty() : true;
#else
    return true;
#endif
  }
};

}  // namespace blender::fn
