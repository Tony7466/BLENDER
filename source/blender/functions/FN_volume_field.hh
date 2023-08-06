/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_volume.hh"

#include "FN_multi_function_procedure.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

#pragma once

namespace blender::fn {
class GFieldRef;
}  // namespace blender::fn

namespace blender::fn {

void evaluate_procedure_on_varying_volume_fields(ResourceScope &scope,
                                                 const volume::GGrid &mask,
                                                 const multi_function::Procedure &procedure,
                                                 Span<volume::GGrid> field_context_inputs,
                                                 Span<GFieldRef> fields_to_evaluate,
                                                 Span<int> field_indices,
                                                 Span<volume::GMutableGrid> dst_grids,
                                                 MutableSpan<volume::GGrid> r_grids,
                                                 MutableSpan<bool> r_is_output_written_to_dst);
void evaluate_procedure_on_constant_volume_fields(ResourceScope &scope,
                                                  const multi_function::Procedure &procedure,
                                                  Span<volume::GGrid> field_context_inputs,
                                                  Span<GFieldRef> fields_to_evaluate,
                                                  Span<int> field_indices,
                                                  MutableSpan<volume::GGrid> r_grids);

}  // namespace blender::fn
