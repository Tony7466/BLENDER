//
// Created by Charles Wardlaw on 2024-02-29.
//

#pragma once

#include <optional>
#include <string>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>

#include "BKE_idprop.h"
#include "usd.hh"


namespace blender::io::usd {


void set_id_props_from_prim(ID *id,
               const pxr::UsdPrim &prim,
               const eUSDAttrImportMode attr_import_mode = USD_ATTR_IMPORT_ALL,
               const std::optional<double> motionSampleTime = std::nullopt);


} // end namesapce blender::io::usd
