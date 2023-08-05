/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#ifdef WITH_OPENVDB

#  include "BLI_volume.hh"

#  include "BKE_volume.h"

#  include "BLI_math_matrix_types.hh"
#  include "BLI_math_vector_types.hh"
#  include "BLI_string_ref.hh"

struct VolumeGrid;

VolumeGrid *BKE_volume_grid_add_vdb(Volume &volume,
                                    blender::StringRef name,
                                    blender::volume::GMutableGrid vdb_grid);

bool BKE_volume_grid_bounds(blender::volume::GGrid grid,
                            blender::float3 &r_min,
                            blender::float3 &r_max);

/**
 * Return a new grid pointer with only the metadata and transform changed.
 * This is useful for instances, where there is a separate transform on top of the original
 * grid transform that must be applied for some operations that only take a grid argument.
 */
blender::volume::GGrid BKE_volume_grid_shallow_transform(blender::volume::GGrid grid,
                                                         const blender::float4x4 &transform);

blender::volume::GGrid BKE_volume_grid_openvdb_for_metadata(const VolumeGrid *grid);
blender::volume::GGrid BKE_volume_grid_openvdb_for_read(const Volume *volume,
                                                        const VolumeGrid *grid);
blender::volume::GMutableGrid BKE_volume_grid_openvdb_for_write(const Volume *volume,
                                                                VolumeGrid *grid,
                                                                bool clear);

void BKE_volume_grid_clear_tree(Volume &volume, VolumeGrid &volume_grid);
void BKE_volume_grid_clear_tree(openvdb::GridBase &grid);

VolumeGridType BKE_volume_grid_type_openvdb(const openvdb::GridBase &grid);

template<typename OpType>
auto BKE_volume_grid_type_operation(const VolumeGridType grid_type, OpType &&op)
{
  namespace grid_types = blender::volume::grid_types;
  switch (grid_type) {
    case VOLUME_GRID_FLOAT:
      return op.template operator()<grid_types::FloatGrid>();
    case VOLUME_GRID_VECTOR_FLOAT:
      return op.template operator()<grid_types::Float3Grid>();
    case VOLUME_GRID_BOOLEAN:
      return op.template operator()<grid_types::BoolGrid>();
    case VOLUME_GRID_DOUBLE:
      return op.template operator()<grid_types::DoubleGrid>();
    case VOLUME_GRID_INT:
      return op.template operator()<grid_types::IntGrid>();
    case VOLUME_GRID_INT64:
      return op.template operator()<grid_types::GridCommon<int64_t>>();
    case VOLUME_GRID_VECTOR_INT:
      return op.template operator()<grid_types::Int3Grid>();
    case VOLUME_GRID_VECTOR_DOUBLE:
      return op.template operator()<grid_types::Double3Grid>();
    case VOLUME_GRID_MASK:
      return op.template operator()<grid_types::MaskGrid>();
    case VOLUME_GRID_POINTS:
      return op.template operator()<grid_types::PointDataGrid>();
    case VOLUME_GRID_UNKNOWN:
      break;
  }

  /* Should never be called. */
  BLI_assert_msg(0, "should never be reached");
  return op.template operator()<grid_types::FloatGrid>();
}

blender::volume::GMutableGrid BKE_volume_grid_create_with_changed_resolution(
    const VolumeGridType grid_type, const openvdb::GridBase &old_grid, float resolution_factor);

#endif
