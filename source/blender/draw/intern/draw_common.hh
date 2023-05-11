/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include "draw_common.h"
#include "draw_manager.hh"
#include "draw_pass.hh"

namespace blender::draw {

/**
 * Add attributes bindings of volume grids to an existing pass.
 * No draw call is added so the caller can decide how to use the data.
 * \return nullptr if there is nothing to draw.
 */
PassMain::Sub *volume_sub_pass(PassMain::Sub &ps,
                               Scene *scene,
                               Object *ob,
                               GPUMaterial *gpu_material);
/**
 * Add attributes bindings of volume grids to an existing pass.
 * No draw call is added so the caller can decide how to use the data.
 * \return nullptr if there is nothing to draw.
 */
PassSimple::Sub *volume_sub_pass(PassSimple::Sub &ps,
                                 Scene *scene,
                                 Object *ob,
                                 GPUMaterial *gpu_material);

}  // namespace blender::draw
