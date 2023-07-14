/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#pragma once

namespace blender {
template<typename T> class VArray;
}

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

}  // namespace blender::fn
