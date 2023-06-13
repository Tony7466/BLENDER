/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_context.h"
#include "BKE_image.h"

namespace blender::render::hydra {

std::string cache_or_get_image_file(Image *image, bContext *context, ImageUser *iuser);
std::string cache_image_color(float color[4]);

}  // namespace blender::render::hydra
