/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include "draw_common.h"
#include "draw_manager.hh"
#include "draw_pass.hh"

namespace blender::draw {

bool volume_sub_pass(PassMain::Sub &ps, Scene *scene, Object *ob, GPUMaterial *gpu_material);

}  // namespace blender::draw
