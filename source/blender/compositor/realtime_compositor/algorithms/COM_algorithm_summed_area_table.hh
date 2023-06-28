/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "COM_context.hh"
#include "COM_result.hh"

namespace blender::realtime_compositor {

/* Computes a summed area table from the given input and write the table to the given output. A
 * summed are table is an image where each pixel contains the sum of all pixels in the areas down
 * and to its left toward the zero index, including the pixel itself. This table is particularly
 * useful to accelerate filters that requires averaging large rectangular areas of the input, like
 * a box filter. */
void summed_area_table(Context &context, Result &input, Result &output);

}  // namespace blender::realtime_compositor
