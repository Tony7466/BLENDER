/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2022 Blender Foundation. All rights reserved. */

#include <numeric>

#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/usd/usdGeom/nurbsCurves.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "usd_hierarchy_iterator.h"
#include "usd_writer_curves.h"

#include "BKE_curves.hh"
#include "BKE_material.h"

#include "BLI_math_geom.h"

#include "WM_api.h"
#include "WM_types.h"

namespace blender::io::usd {

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx)
    : USDAbstractWriter(ctx), converted_curves(nullptr)
{
}

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx, Curves *converted_legacy_curves)
    : USDAbstractWriter(ctx), converted_curves(converted_legacy_curves)
{
}

pxr::UsdGeomCurves USDCurvesWriter::DefineUsdGeomBasisCurves(pxr::VtValue curve_basis,
                                                             bool cyclic,
                                                             bool cubic)
{
  pxr::UsdGeomCurves curves = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                              usd_export_context_.usd_path);

  pxr::UsdGeomBasisCurves basis_curves = pxr::UsdGeomBasisCurves(curves);
  // Not required to set the basis attribute for linear curves
  // https://graphics.pixar.com/usd/dev/api/class_usd_geom_basis_curves.html#details
  if (cubic) {
    basis_curves.CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->cubic));
    basis_curves.CreateBasisAttr(curve_basis);
  }
  else {
    basis_curves.CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
  }

  if (cyclic) {
    basis_curves.CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->periodic));
  }
  else {
    basis_curves.CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->nonperiodic));
  }

  return curves;
}

void populate_curve_widths(const bke::CurvesGeometry &geometry, pxr::VtArray<float> &widths)
{
  const bke::AttributeAccessor curve_attributes = geometry.attributes();
  const VArray<float> radii = curve_attributes.lookup<float>("radius", ATTR_DOMAIN_POINT);

  widths.resize(radii.size());

  if (widths.size() == 0) {
    return;
  }

  for (const int i : radii.index_range()) {
    widths[i] = radii[i] * 2.0f;
  }
}

void set_curve_width_interpolation(const pxr::VtArray<float> &widths,
                                   const pxr::VtArray<int> &segments,
                                   const pxr::VtIntArray &control_point_counts,
                                   const bool cyclic,
                                   pxr::TfToken &interpolation)
{
  size_t expectedVaryingSize;
  if (cyclic) {
    expectedVaryingSize = std::accumulate(segments.begin(), segments.end(), 0);
  }
  else {
    expectedVaryingSize = std::accumulate(segments.begin(), segments.end(), 0) +
                          control_point_counts.size();
  }

  size_t accumulatedControlPointCount = std::accumulate(
      control_point_counts.begin(), control_point_counts.end(), 0);

  if (widths.size() == 1)
    interpolation = pxr::UsdGeomTokens->constant;
  else if (widths.size() == accumulatedControlPointCount)
    interpolation = pxr::UsdGeomTokens->vertex;
  else if (widths.size() == control_point_counts.size())
    interpolation = pxr::UsdGeomTokens->uniform;
  else if (widths.size() == expectedVaryingSize)
    interpolation = pxr::UsdGeomTokens->varying;
  else {
    WM_reportf(RPT_WARNING, "Curve width size not supported for standard USD interpolation.");
  }
}

void populate_curve_props(const bke::CurvesGeometry &geometry,
                          pxr::VtArray<pxr::GfVec3f> &verts,
                          pxr::VtIntArray &control_point_counts,
                          pxr::VtArray<float> &widths,
                          pxr::TfToken &interpolation,
                          const bool cyclic,
                          const bool cubic)
{
  const int num_curves = geometry.curve_num;
  const Span<float3> positions = geometry.positions();

  pxr::VtArray<int> segments;
  segments.resize(num_curves);

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {
    auto curve_points = geometry.points_for_curve(i_curve);
    long tot_points = 0;
    for (const auto i_point : curve_points) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
      tot_points++;
    }

    control_point_counts[i_curve] = tot_points;

    if (cubic) {
      if (cyclic) {
        segments[i_curve] = tot_points;
      }
      else {
        segments[i_curve] = (tot_points - 4) + 1;
      }
    }
    else {
      if (cyclic) {
        segments[i_curve] = tot_points;
      }
      else {
        segments[i_curve] = tot_points - 1;
      }
    }
  }

  populate_curve_widths(geometry, widths);
  set_curve_width_interpolation(widths, segments, control_point_counts, cyclic, interpolation);
}

void populate_curve_props_for_bezier(const bke::CurvesGeometry &geometry,
                                     pxr::VtArray<pxr::GfVec3f> &verts,
                                     pxr::VtIntArray &control_point_counts,
                                     pxr::VtArray<float> &widths,
                                     pxr::TfToken &interpolation,
                                     const bool cyclic)
{
  const int bezier_vstep = 3;
  const int num_curves = geometry.curve_num;

  const Span<float3> positions = geometry.positions();

  const Span<float3> handles_l = geometry.handle_positions_left();
  const Span<float3> handles_r = geometry.handle_positions_right();

  pxr::VtArray<int> segments;
  segments.resize(num_curves);

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {
    auto curve_points = geometry.points_for_curve(i_curve);

    long tot_points = 0;
    for (const long i_point : curve_points) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
      tot_points++;

      if (i_point < curve_points.size() - 1) {
        verts.push_back(
            pxr::GfVec3f(handles_r[i_point][0], handles_r[i_point][1], handles_r[i_point][2]));
        verts.push_back(pxr::GfVec3f(
            handles_l[i_point + 1][0], handles_l[i_point + 1][1], handles_l[i_point + 1][2]));
        tot_points += 2;
      }
      else if (i_point == curve_points.size() - 1 && cyclic) {
        verts.push_back(
            pxr::GfVec3f(handles_r[i_point][0], handles_r[i_point][1], handles_r[i_point][2]));

        auto start_point = curve_points[0];
        verts.push_back(pxr::GfVec3f(
            handles_l[start_point][0], handles_l[start_point][1], handles_l[start_point][2]));

        tot_points += 2;
      }
    }

    control_point_counts[i_curve] = tot_points;

    if (cyclic) {
      segments[i_curve] = tot_points / bezier_vstep;
    }
    else {
      segments[i_curve] = ((tot_points - 4) / bezier_vstep) + 1;
    }
  }

  populate_curve_widths(geometry, widths);
  set_curve_width_interpolation(widths, segments, control_point_counts, cyclic, interpolation);
}

void populate_curve_props_for_nurbs(const bke::CurvesGeometry &geometry,
                                    pxr::VtArray<pxr::GfVec3f> &verts,
                                    pxr::VtIntArray &control_point_counts,
                                    pxr::VtArray<float> &widths,
                                    pxr::VtArray<double> &knots,
                                    pxr::VtArray<int> &orders,
                                    pxr::TfToken &interpolation,
                                    const bool cyclic)
{
  const int num_curves = geometry.curve_num;
  orders.resize(num_curves);

  const Span<float3> positions = geometry.positions();

  VArray<int8_t> geom_orders = geometry.nurbs_orders();
  VArray<int8_t> knots_modes = geometry.nurbs_knots_modes();

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {
    auto curve_points = geometry.points_for_curve(i_curve);
    long tot_points = 0;
    for (const auto i_point : curve_points) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
      tot_points++;
    }

    control_point_counts[i_curve] = tot_points;

    const int8_t order = geom_orders[i_curve];
    orders[i_curve] = (int)geom_orders[i_curve];

    const KnotsMode mode = KnotsMode(knots_modes[i_curve]);

    const int knots_num = bke::curves::nurbs::knots_num(curve_points.size(), order, cyclic);
    Array<float> temp_knots(knots_num);
    bke::curves::nurbs::calculate_knots(curve_points.size(), mode, order, cyclic, temp_knots);

    knots.resize(knots_num);
    for (int i_knot = 0; i_knot < knots_num; i_knot++) {
      knots[i_knot] = (double)temp_knots[i_knot];
    }

    /* Set end knots according to the USD spec for periodic curves
       https://graphics.pixar.com/usd/dev/api/class_usd_geom_nurbs_curves.html#details */
    if (cyclic) {
      knots[0] = knots[1] - (knots[knots.size() - 2] - knots[knots.size() - 3]);
      knots[knots.size() - 1] = knots[knots.size() - 2] + (knots[2] - knots[1]);
    }
    else {
      // Set end knots according to the USD spec for non-periodic curves
      knots[0] = knots[1];
      knots[knots.size() - 1] = knots[knots.size() - 2];
    }
  }

  populate_curve_widths(geometry, widths);
  interpolation = pxr::UsdGeomTokens->vertex;
}

void USDCurvesWriter::do_write(HierarchyContext &context)
{

  Curves *curve = converted_curves ? converted_curves :
                                     static_cast<Curves *>(context.object->data);

  const bke::CurvesGeometry &geometry = bke::CurvesGeometry::wrap(curve->geometry);
  if (geometry.points_num() == 0) {
    return;
  }

  const std::array<int, CURVE_TYPES_NUM> curve_type_counts = geometry.curve_type_counts();
  const int number_of_curve_types = std::reduce(
      curve_type_counts.begin(), curve_type_counts.end(), 0, [](int previousResult, int item) {
        return item > 0 ? ++previousResult : previousResult;
      });

  if (number_of_curve_types > 1) {
    WM_reportf(RPT_WARNING, "Cannot export mixed curve types.");
    return;
  }

  const VArray<bool> cyclic_values = geometry.cyclic();
  const bool cyclic = cyclic_values[0];
  bool all_same_cyclic_type = true;

  for (const int i : cyclic_values.index_range()) {
    if (cyclic_values[i] != cyclic) {
      all_same_cyclic_type = false;
      break;
    }
  }

  if (!all_same_cyclic_type) {
    WM_reportf(RPT_WARNING, "Cannot export mixed cyclic and non-cyclic curves.");
    return;
  }

  pxr::UsdTimeCode timecode = get_export_time_code();
  pxr::UsdGeomCurves usd_curves;

  pxr::VtArray<pxr::GfVec3f> verts;
  pxr::VtIntArray control_point_counts;
  control_point_counts.resize(geometry.curves_num());
  pxr::VtArray<float> widths;
  pxr::TfToken interpolation;

  const int8_t curve_type = geometry.curve_types()[0];
  switch (curve_type) {
    case CURVE_TYPE_POLY:
      usd_curves = DefineUsdGeomBasisCurves(pxr::VtValue(), cyclic, false);

      populate_curve_props(
          geometry, verts, control_point_counts, widths, interpolation, cyclic, false);
      break;
    case CURVE_TYPE_CATMULL_ROM:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->catmullRom), cyclic, true);

      populate_curve_props(
          geometry, verts, control_point_counts, widths, interpolation, cyclic, true);
      break;
    case CURVE_TYPE_BEZIER:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->bezier), cyclic, true);

      populate_curve_props_for_bezier(
          geometry, verts, control_point_counts, widths, interpolation, cyclic);
      break;
    case CURVE_TYPE_NURBS: {
      pxr::VtArray<double> knots;
      pxr::VtArray<int> orders;
      orders.resize(geometry.curves_num());

      usd_curves = pxr::UsdGeomNurbsCurves::Define(usd_export_context_.stage,
                                                   usd_export_context_.usd_path);

      populate_curve_props_for_nurbs(
          geometry, verts, control_point_counts, widths, knots, orders, interpolation, cyclic);

      pxr::UsdAttribute attr_knots =
          pxr::UsdGeomNurbsCurves(usd_curves).CreateKnotsAttr(pxr::VtValue(), true);
      usd_value_writer_.SetAttribute(attr_knots, pxr::VtValue(knots), timecode);
      pxr::UsdAttribute attr_order =
          pxr::UsdGeomNurbsCurves(usd_curves).CreateOrderAttr(pxr::VtValue(), true);
      usd_value_writer_.SetAttribute(attr_order, pxr::VtValue(orders), timecode);
      break;
    }
    default:
      BLI_assert_unreachable();
  }

  pxr::UsdAttribute attr_points = usd_curves.CreatePointsAttr(pxr::VtValue(), true);
  usd_value_writer_.SetAttribute(attr_points, pxr::VtValue(verts), timecode);

  pxr::UsdAttribute attr_vertex_counts = usd_curves.CreateCurveVertexCountsAttr(pxr::VtValue(),
                                                                                true);
  usd_value_writer_.SetAttribute(attr_vertex_counts, pxr::VtValue(control_point_counts), timecode);

  if (widths.size() > 0) {
    pxr::UsdAttribute attr_widths = usd_curves.CreateWidthsAttr(pxr::VtValue(), true);
    usd_value_writer_.SetAttribute(attr_widths, pxr::VtValue(widths), timecode);

    usd_curves.SetWidthsInterpolation(interpolation);
  }

  assign_materials(context, usd_curves);
}

bool USDCurvesWriter::check_is_animated(const HierarchyContext &) const
{
  return true;
}

void USDCurvesWriter::assign_materials(const HierarchyContext &context,
                                       pxr::UsdGeomCurves usd_curve)
{
  if (context.object->totcol == 0) {
    return;
  }

  bool curve_material_bound = false;
  for (short mat_num = 0; mat_num < context.object->totcol; mat_num++) {
    Material *material = BKE_object_material_get(context.object, mat_num + 1);
    if (material == nullptr) {
      continue;
    }

    pxr::UsdShadeMaterialBindingAPI api = pxr::UsdShadeMaterialBindingAPI(usd_curve.GetPrim());
    pxr::UsdShadeMaterial usd_material = ensure_usd_material(context, material);
    api.Bind(usd_material);

    /* USD seems to support neither per-material nor per-face-group double-sidedness, so we just
     * use the flag from the first non-empty material slot. */
    usd_curve.CreateDoubleSidedAttr(
        pxr::VtValue((material->blend_flag & MA_BL_CULL_BACKFACE) == 0));

    curve_material_bound = true;
    break;
  }

  if (!curve_material_bound) {
    /* Blender defaults to double-sided, but USD to single-sided. */
    usd_curve.CreateDoubleSidedAttr(pxr::VtValue(true));
  }
}

}  // namespace blender::io::usd
