/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

#include <string>

#include "BKE_volume_enums.hh"

#include "BLI_math_matrix_types.hh"

/**
 * This header gives contains declarations for dealing with volume grids without requiring
 * including any OpenVDB headers (which can have a significant impact on compile times).
 *
 * These functions are available even if `WITH_OPENVDB` is false, but they may just be empty.
 */

namespace blender::bke::volume_grid {

/**
 * Wraps an OpenVDB grid and adds features like implicit sharing and lazy-loading.
 */
class VolumeGridData;

/**
 * Owning container for a #VolumeGridData instance.
 */
class GVolumeGrid;

/**
 * Same as #GVolumeGrid but makes sure that the contained grid is of a specific type.
 */
template<typename T> class VolumeGrid;

/**
 * Access token required to use the tree stored in a volume grid. This allows detecting whether a
 * tree is currently used or not, for the purpose of safely freeing unused trees.
 */
class VolumeTreeUser;

/**
 * Compile time check to see of a type is a #VolumeGrid. This is false for e.g. `float` or
 * `GVolumeGrid` and true for e.g. `VolumeGrid<int>` and `VolumeGrid<float>`.
 */
template<typename T> static constexpr bool is_VolumeGrid_v = false;
template<typename T> static constexpr bool is_VolumeGrid_v<VolumeGrid<T>> = true;

/**
 * Get the name stored in the volume grid, e.g. "density".
 */
std::string get_name(const VolumeGridData &grid);

/**
 * Get the data type stored in the volume grid.
 */
VolumeGridType get_type(const VolumeGridData &grid);

/**
 * Get the number of primitive values stored per voxel. For example, for a float-grid this is 1 and
 * for a vector-grid it is 3 (for x, y and z).
 */
int get_channels_num(VolumeGridType type);

/**
 * Unloads the tree data if no one is using it right now and it could be reloaded later on.
 */
void unload_tree_if_possible(const VolumeGridData &grid);

/**
 * Get the transform of the grid as an affine matrix.
 */
float4x4 get_transform_matrix(const VolumeGridData &grid);

/**
 * Replaces the transform matrix with the given one.
 */
void set_transform_matrix(VolumeGridData &grid, const float4x4 &matrix);

/**
 * Clears the tree data in the grid, but keeps meta-data and the transform intact.
 */
void clear_tree(VolumeGridData &grid);

}  // namespace blender::bke::volume_grid

/**
 * Put the most common types directly into the `blender::bke` namespace.
 */
namespace blender::bke {
using volume_grid::GVolumeGrid;
using volume_grid::is_VolumeGrid_v;
using volume_grid::VolumeGrid;
using volume_grid::VolumeGridData;
using volume_grid::VolumeTreeUser;
}  // namespace blender::bke
