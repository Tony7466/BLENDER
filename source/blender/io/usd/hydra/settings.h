/* SPDX-License-Identifier: GPL-2.0-or-later
 * SPDX-FileCopyrightText: 2011-2023 Blender Foundation */

#pragma once

#include <pxr/base/tf/token.h>

#include "BLI_map.hh"

namespace blender::io::hydra {

struct HydraDelegateSettings {
  pxr::TfToken mx_filename_key;
  Map<pxr::TfToken, pxr::VtValue> render_tokens;
};

};  // namespace blender::io::hydra
