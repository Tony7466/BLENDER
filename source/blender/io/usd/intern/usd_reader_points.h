/* SPDX-FileCopyrightText: 2023 ????. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "usd.hh"
#include "usd_reader_geom.hh"
#include "usd_reader_xform.hh"
#include <pxr/usd/usdGeom/points.h>

struct Mesh;

namespace blender::io::usd {

/*
 * Read UsdGeomPoints primitives as Blender point clouds.
 */
class USDPointsReader : public USDGeomReader {
 private:
  pxr::UsdGeomPoints points_prim_;

 public:
  USDPointsReader(const pxr::UsdPrim &prim,
                  const USDImportParams &import_params,
                  const ImportSettings &settings);

  bool valid() const override;

  /* Initial object creation. */
  void create_object(Main *bmain, double /*motionSampleTime*/) override;
  /* Initial point cloud data update. */
  void read_object_data(Main *bmain, double motionSampleTime) override;
  /* Implement point cloud update. This may be called by the cache modifier
   * to update animated geomtery. */
  void read_geometry(bke::GeometrySet &geometry_set,
                     USDMeshReadParams /*params*/,
                     const char ** /*err_str*/) override;

  /* Return true if the USD data may be time varying. */
  bool is_animated() const;
};

}  // namespace blender::io::usd
