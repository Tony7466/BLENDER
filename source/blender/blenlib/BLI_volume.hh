/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bli
 */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender {

class CPPType;
class ResourceScope;

namespace volume_mask {

/* Mask defined by active voxels of the grid. */
class VolumeMask {
#ifdef WITH_OPENVDB
  const openvdb::MaskGrid &grid_;
#endif

 public:
  bool is_empty() const;
  int64_t min_voxel_count() const;

#ifdef WITH_OPENVDB
  const openvdb::MaskGrid &grid() const
  {
    return grid_;
  }
#endif
};

}  // namespace volume_mask

using volume_mask::VolumeMask;

struct VolumeGrid {
#ifdef WITH_OPENVDB
  openvdb::GridBase::Ptr grid_ = nullptr;
#endif

  /* Create an empty grid with a background value. */
  static VolumeGrid create(ResourceScope &scope,
                           const CPPType &type,
                           const void *background_value);
  /* Create an empty grid with the type default as background value. */
  static VolumeGrid create(ResourceScope &scope, const CPPType &type);
  /* Create a grid with the active volume mask voxels. */
  static VolumeGrid create(ResourceScope &scope,
                           const CPPType &type,
                           const VolumeMask &mask,
                           const void *inactive_value,
                           const void *active_value);

  int64_t voxel_count() const;
  bool is_empty() const;
  operator bool() const;
};

}  // namespace blender
