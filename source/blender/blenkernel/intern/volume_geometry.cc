/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "DNA_volume_types.h"

#include "BKE_volume_geometry.hh"

#include "attribute_access_volume.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name VolumeGeometry
 * \{ */

VolumeGeometry::VolumeGeometry() {}

/**
 * \note Expects `dst` to be initialized, since the original attributes must be freed.
 */
static void copy_volume_geometry(VolumeGeometry &dst, const VolumeGeometry &src)
{
  dst.grid = static_cast<VolumeGeometryGrid *>(MEM_dupallocN(src.grid));
}

VolumeGeometry::VolumeGeometry(const VolumeGeometry &other) : VolumeGeometry()
{
  copy_volume_geometry(*this, other);
}

VolumeGeometry &VolumeGeometry::operator=(const VolumeGeometry &other)
{
  if (this != &other) {
    copy_volume_geometry(*this, other);
  }
  return *this;
}

/* The source should be empty, but in a valid state so that using it further will work. */
static void move_volume_geometry(VolumeGeometry &dst, VolumeGeometry &src)
{
  std::swap(dst.grid, src.grid);
  src.grid = nullptr;
}

VolumeGeometry::VolumeGeometry(VolumeGeometry &&other) : VolumeGeometry()
{
  move_volume_geometry(*this, other);
}

VolumeGeometry &VolumeGeometry::operator=(VolumeGeometry &&other)
{
  if (this != &other) {
    move_volume_geometry(*this, other);
  }
  return *this;
}

VolumeGeometry::~VolumeGeometry()
{
  if (grid) {
    MEM_delete(grid);
  }
}

int VolumeGeometry::domain_size(eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return grid ? int(grid->active_voxel_num()) : 0;
    default:
      return 0;
  }
}

GVArray VolumeGeometry::adapt_domain(const GVArray &varray, eAttrDomain from, eAttrDomain to) const
{
  if (from == to) {
    return varray;
  }
  return {};
}

void VolumeGeometry::blend_read_data(BlendDataReader & /*reader*/)
{
  grid = nullptr;
}

void VolumeGeometry::blend_write(BlendWriter & /*writer*/, ID & /*id*/) {}

/** \} */

}  // namespace blender::bke
