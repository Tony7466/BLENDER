/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "DNA_volume_types.h"

#include "BKE_volume.h"
#include "BKE_volume_geometry.hh"

namespace blender::bke {

/* -------------------------------------------------------------------- */
/** \name Constructors/Destructor
 * \{ */

VolumeGeometry::VolumeGeometry() {}

/**
 * \note Expects `dst` to be initialized, since the original attributes must be freed.
 */
static void copy_volume_geometry(VolumeGeometry &dst, const VolumeGeometry &src)
{
  dst.grid = src.grid;
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
  dst.grid = src.grid;
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
  BKE_volume_grid_free(grid);
}

/** \} */

}  // namespace blender::bke
