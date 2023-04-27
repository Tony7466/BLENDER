/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include "BKE_image.h"
#include "BKE_image_save.h"

namespace blender::render::hydra {

class BlenderSceneDelegate;

std::string cache_or_get_image_file(Image *image,
                                    BlenderSceneDelegate *scene_delegate,
                                    ImageUser *iuser);

}  // namespace blender::render::hydra
