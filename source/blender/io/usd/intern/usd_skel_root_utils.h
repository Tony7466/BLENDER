/* SPDX-FileCopyrightText: 2023 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "usd.h"

#include <pxr/usd/usd/stage.h>

namespace blender::io::usd {

void create_skel_roots(pxr::UsdStageRefPtr stage, const USDExportParams &params);

}  // namespace blender::io::usd
