/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_function_ref.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_string_ref.hh"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_modifier_types.h"

#pragma once

struct Volume;
struct VolumeGrid;
struct Depsgraph;

/** \file
 * \ingroup geo
 */

namespace blender::geometry {

struct MeshToVolumeSettings {
  int voxels;
  float voxel_size;
  float interior_band_width;
  float exterior_band_width;
  float density;
  float simplify;
  bool fill_volume;
  bool use_world_space_units;
  bool convert_to_fog;
  bool unsigned_distance;
};

#ifdef WITH_OPENVDB

/**
 * \param bounds_fn: Return the bounds of the mesh positions,
 * used for deciding the voxel size in "Amount" mode.
 */
float volume_compute_voxel_size(const MeshToVolumeSettings &settings,
                                FunctionRef<void(float3 &r_min, float3 &r_max)> bounds_fn,
                                const float4x4 &transform);
/**
 * Add a new VolumeGrid to the Volume by converting the supplied mesh.
 */
VolumeGrid *volume_grid_add_from_mesh(Volume *volume,
                                      StringRefNull name,
                                      const Mesh *mesh,
                                      const float4x4 &mesh_to_volume_space_transform,
                                      const MeshToVolumeSettings &settings);
#endif
}  // namespace blender::geometry
