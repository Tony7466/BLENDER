/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_span.hh"

struct Sculpt;
struct Object;
struct PBVHNode;

namespace blender::ed::sculpt_paint {

void do_draw_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes);
void do_draw_vector_displacement_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes);
void do_smooth_brush(const Sculpt &sd, Object &object, Span<PBVHNode *> nodes);

}  // namespace blender::ed::sculpt_paint
