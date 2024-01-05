/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup animrig
 *
 * \brief Functions to work with drivers.
 */

#include "RNA_types.hh"

struct AnimationEvalContext;
struct FCurve;

namespace blender::animrig {

/** Adjust frame on which to add keyframe, to make it easier to add corrective drivers. */
float remap_driver_frame(const AnimationEvalContext *anim_eval_context,
                         PointerRNA *ptr,
                         PropertyRNA *prop,
                         const FCurve *fcu);

}  // namespace blender::animrig