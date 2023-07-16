/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "FN_multi_function_procedure.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#pragma once

namespace blender {
class CPPType;
class ResourceScope;
template<typename T> class VArray;
namespace fn {
class GFieldRef;
}
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

#ifdef WITH_OPENVDB
  const openvdb::MaskGrid &grid() const
  {
    return grid_;
  }
#endif
};

}  // namespace blender::volume_mask

namespace blender::fn {

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

void evaluate_procedure_on_varying_volume_fields(ResourceScope &scope,
                                                 const VolumeMask &mask,
                                                 const mf::Procedure &procedure,
                                                 Span<VolumeGrid> field_context_inputs,
                                                 Span<GFieldRef> fields_to_evaluate,
                                                 Span<int> field_indices,
                                                 Span<VolumeGrid> dst_grids,
                                                 MutableSpan<VolumeGrid> r_grids,
                                                 MutableSpan<bool> r_is_output_written_to_dst);
void evaluate_procedure_on_constant_volume_fields(ResourceScope &scope,
                                                  const VolumeMask &mask,
                                                  const mf::Procedure &procedure,
                                                  Span<VolumeGrid> field_context_inputs,
                                                  Span<GFieldRef> fields_to_evaluate,
                                                  Span<int> field_indices,
                                                  Span<VolumeGrid> dst_grids,
                                                  MutableSpan<VolumeGrid> r_grids,
                                                  MutableSpan<bool> r_is_output_written_to_dst);

}  // namespace blender::fn
