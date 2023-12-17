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

namespace blender::bke {

class VolumeGridData;

class GVolumeGrid;
template<typename T> class VolumeGrid;

class VolumeTreeUser;

template<typename T> static constexpr bool is_VolumeGrid_v = false;
template<typename T> static constexpr bool is_VolumeGrid_v<VolumeGrid<T>> = true;

namespace volume_grid_fwd {

std::string get_name(const VolumeGridData &grid);
VolumeGridType get_type(const VolumeGridData &grid);
int get_channels_num(VolumeGridType type);
void unload_tree_if_possible(const VolumeGridData &grid);
float4x4 get_transform_matrix(const VolumeGridData &grid);
void set_transform_matrix(VolumeGridData &grid, const float4x4 &matrix);

}  // namespace volume_grid_fwd

}  // namespace blender::bke
