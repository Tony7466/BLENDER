/* SPDX-FileCopyrightText: 2023 NVIDIA Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "usd_reader_xform.h"

#include <pxr/usd/usdGeom/xform.h>

struct Collection;

namespace blender::io::usd {

class USDInstanceReader : public USDXformReader {

 public:
  USDInstanceReader(const pxr::UsdPrim &prim,
                    const USDImportParams &import_params,
                    const ImportSettings &settings);

  bool valid() const override;

  void create_object(Main *bmain, double motionSampleTime) override;

  void set_instance_collection(Collection *coll);

  pxr::SdfPath proto_path() const;
};

}  // namespace blender::io::usd
