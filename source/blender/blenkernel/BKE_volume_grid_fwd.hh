/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 */

namespace blender::bke {

class VolumeGridData;

class GVolumeGrid;
template<typename T> class VolumeGrid;

class VolumeTreeUser;

template<typename T> static constexpr bool is_VolumeGrid_v = false;
template<typename T> static constexpr bool is_VolumeGrid_v<VolumeGrid<T>> = true;

}  // namespace blender::bke
