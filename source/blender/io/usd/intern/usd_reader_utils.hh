/* SPDX-FileCopyrightText: 2024 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "usd.hh"

#include <pxr/usd/usd/prim.h>

#include "BKE_idprop.hh"

namespace blender::io::usd {

void set_id_props_from_prim(ID *id,
                            const pxr::UsdPrim &prim,
                            const eUSDAttrImportMode attr_import_mode = USD_ATTR_IMPORT_ALL,
                            const std::optional<double> motionSampleTime = std::nullopt);

}  // namespace blender::io::usd
