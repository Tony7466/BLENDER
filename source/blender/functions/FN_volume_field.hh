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
class VolumeGrid;
class VolumeMask;

namespace fn {
class GFieldRef;
}
}  // namespace blender

namespace blender::fn {

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
                                                  const mf::Procedure &procedure,
                                                  Span<VolumeGrid> field_context_inputs,
                                                  Span<GFieldRef> fields_to_evaluate,
                                                  Span<int> field_indices,
                                                  MutableSpan<VolumeGrid> r_grids);

}  // namespace blender::fn
