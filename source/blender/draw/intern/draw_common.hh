/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup draw
 */

#pragma once

#include "draw_common.h"
#include "draw_manager.hh"
#include "draw_pass.hh"

namespace blender::draw {

struct VolumeAttribute {
  StringRefNull input_name;
  StringRefNull name;
  eGPUDefaultValue default_value;
};

bool volume_sub_pass(PassMain::Sub &ps, Scene *scene, Object *ob, Span<VolumeAttribute> attrs);

}  // namespace blender::draw
