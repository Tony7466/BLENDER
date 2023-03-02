/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */
#pragma once

#include "DNA_curves_types.h"
#include "usd_writer_abstract.h"

#include <pxr/usd/usdGeom/basisCurves.h>

namespace blender::io::usd {

/* Writer for writing Curves data as USD curves. */
class USDCurvesWriter : public USDAbstractWriter {
 public:
  USDCurvesWriter(const USDExporterContext &ctx);
  USDCurvesWriter(const USDExporterContext &ctx, Curves *converted_legacy_curves);

 protected:
  virtual void do_write(HierarchyContext &context) override;
  virtual bool check_is_animated(const HierarchyContext &context) const override;
  void assign_materials(const HierarchyContext &context, pxr::UsdGeomCurves usd_curve);

 private:
  Curves *converted_curves;
  pxr::UsdGeomCurves DefineUsdGeomBasisCurves(pxr::VtValue curve_basis, bool cyclic, bool cubic);
};

}  // namespace blender::io::usd

