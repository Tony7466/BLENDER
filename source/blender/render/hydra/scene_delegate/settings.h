/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2023 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>

#include "BLI_map.hh"

namespace blender::render::hydra {

struct SceneDelegateSettings {
  pxr::TfToken mx_filename_key;
  Map<pxr::TfToken, pxr::VtValue> render_tokens;
};

};  // namespace blender::render::hydra
