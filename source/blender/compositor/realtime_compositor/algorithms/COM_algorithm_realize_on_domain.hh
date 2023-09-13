/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_math_matrix_types.hh"

#include "COM_context.hh"
#include "COM_domain.hh"
#include "COM_result.hh"

namespace blender::realtime_compositor {

/* Given a potentially transformed domain, compute a domain such that the transformations specified
 * in the given realization options become identity and the size of the domain is increased/reduced
 * to adapt to the new transformation. For instance, if realize_rotation is true and the domain is
 * rotated, the returned domain will have zero rotation but expanded size to account for the
 * bounding box of the domain after rotation. */
Domain compute_realized_transformation_domain(const Domain &domain,
                                              bool realize_rotation,
                                              bool realize_scale);

/* Projects the input on a target domain, copies the area of the input that intersects the target
 * domain, and fill the rest with zeros or repetitions of the input depending on the realization
 * options. The transformation and realization options of the input are ignored and the given
 * input_transformation and realization_options are used instead to allow the caller to change them
 * without mutating the input result directly. See the discussion in COM_domain.hh for more
 * information on what realization on domain means. */
void realize_on_domain(Context &context,
                       Result &input,
                       Result &output,
                       const Domain &domain,
                       const float3x3 &input_transformation,
                       const RealizationOptions &realization_options);

}  // namespace blender::realtime_compositor
