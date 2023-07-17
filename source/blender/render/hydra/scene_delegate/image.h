/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#pragma once

#include <string>

struct Main;
struct Scene;
struct Image;
struct ImageUser;

namespace blender::render::hydra {

std::string cache_or_get_image_file(Main *bmain, Scene *Scene, Image *image, ImageUser *iuser);
std::string cache_image_color(float color[4]);

}  // namespace blender::render::hydra
