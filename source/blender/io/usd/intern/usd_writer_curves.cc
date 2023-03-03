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
#include "BKE_lib_id.h"
#include "BKE_material.h"

#include "BLI_math_geom.h"

#include "WM_api.h"
#include "WM_types.h"

namespace blender::io::usd {

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx)
    : USDAbstractWriter(ctx), converted_curves_(nullptr)
{
}

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx,
                                 std::unique_ptr<Curves> converted_legacy_curves)
    : USDAbstractWriter(ctx), converted_curves_(std::move(converted_legacy_curves))
{
}

USDCurvesWriter::~USDCurvesWriter()
{
  if (converted_curves_) {
    BKE_id_free(nullptr, converted_curves_.release());
  }
}

pxr::UsdGeomCurves USDCurvesWriter::DefineUsdGeomBasisCurves(pxr::VtValue curve_basis,
                                                             const bool is_cyclic,
                                                             const bool is_cubic)
{
  pxr::UsdGeomCurves curves = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                              usd_export_context_.usd_path);

  pxr::UsdGeomBasisCurves basis_curves = pxr::UsdGeomBasisCurves(curves);
  /* Not required to set the basis attribute for linear curves
   * https://graphics.pixar.com/usd/dev/api/class_usd_geom_basis_curves.html#details */
  if (is_cubic) {
    basis_curves.CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->cubic));
    basis_curves.CreateBasisAttr(curve_basis);
  }
  else {
    basis_curves.CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
  }

  if (is_cyclic) {
    basis_curves.CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->periodic));
  }
  else {
    basis_curves.CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->nonperiodic));
  }

  return curves;
}

static void populate_curve_widths(const bke::CurvesGeometry &geometry, pxr::VtArray<float> &widths)
{
  const bke::AttributeAccessor curve_attributes = geometry.attributes();
  const VArray<float> radii = curve_attributes.lookup<float>("radius", ATTR_DOMAIN_POINT);

  widths.resize(radii.size());

  for (const int i : radii.index_range()) {
    widths[i] = radii[i] * 2.0f;
  }
}

static pxr::TfToken get_curve_width_interpolation(const pxr::VtArray<float> &widths,
                                                  const pxr::VtArray<int> &segments,
                                                  const pxr::VtIntArray &control_point_counts,
                                                  const bool is_cyclic)
{
  if (widths.empty()) {
    return pxr::TfToken();
  }

  const size_t accumulatedControlPointCount = std::accumulate(
      control_point_counts.begin(), control_point_counts.end(), 0);

  /* For Blender curves, radii are always stored per point. For linear curves, this should match
   * with USD's vertex interpolation. For cubic curves, this should match with USD's varying
   * interpolation. */
  if (widths.size() == accumulatedControlPointCount) {
    return pxr::UsdGeomTokens->vertex;
  }

  size_t expectedVaryingSize = std::accumulate(segments.begin(), segments.end(), 0);
  if (!is_cyclic) {
    expectedVaryingSize += control_point_counts.size();
  }

  if (widths.size() == expectedVaryingSize) {
    return pxr::UsdGeomTokens->varying;
  }

  WM_report(RPT_WARNING, "Curve width size not supported for USD interpolation.");
  return pxr::TfToken();
}

static void populate_curve_verts(const bke::CurvesGeometry &geometry,
                                 const Span<float3> positions,
                                 pxr::VtArray<pxr::GfVec3f> &verts,
                                 pxr::VtIntArray &control_point_counts,
                                 pxr::VtArray<int> &segments,
                                 const bool is_cyclic,
                                 const bool is_cubic)
{
  const OffsetIndices points_by_curve = geometry.points_by_curve();
  for (const int i_curve : geometry.curves_range()) {

    const IndexRange curve_points = points_by_curve[i_curve];
    for (const int i_point : curve_points) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
    }

    const int tot_points = curve_points.size();
    control_point_counts[i_curve] = tot_points;

    /* For periodic linear curve, segment count = curveVertexCount.
       For periodic cubic curve, segment count = curveVertexCount / vstep.
       For nonperiodic linear curve, segment count = curveVertexCount - 1.
       For nonperiodic cubic curve, segment count = ((curveVertexCount - 4) / vstep) + 1.
       This function handles linear and Catmull-Rom curves. For Catmull-Rom, vstep is 1.
       https://graphics.pixar.com/usd/dev/api/class_usd_geom_basis_curves.html */
    if (is_cyclic) {
      segments[i_curve] = tot_points;
    }
    else if (is_cubic) {
      segments[i_curve] = (tot_points - 4) + 1;
    }
    else {
      segments[i_curve] = tot_points - 1;
    }
  }
}

static void populate_curve_props(const bke::CurvesGeometry &geometry,
                                 pxr::VtArray<pxr::GfVec3f> &verts,
                                 pxr::VtIntArray &control_point_counts,
                                 pxr::VtArray<float> &widths,
                                 pxr::TfToken &interpolation,
                                 const bool is_cyclic,
                                 const bool is_cubic)
{
  const int num_curves = geometry.curve_num;
  const Span<float3> positions = geometry.positions();

  pxr::VtArray<int> segments(num_curves);

  populate_curve_verts(
      geometry, positions, verts, control_point_counts, segments, is_cyclic, is_cubic);

  populate_curve_widths(geometry, widths);
  interpolation = get_curve_width_interpolation(widths, segments, control_point_counts, is_cyclic);
}

static void populate_curve_verts_for_bezier(const bke::CurvesGeometry &geometry,
                                            const Span<float3> positions,
                                            const Span<float3> handles_l,
                                            const Span<float3> handles_r,
                                            pxr::VtArray<pxr::GfVec3f> &verts,
                                            pxr::VtIntArray &control_point_counts,
                                            pxr::VtArray<int> &segments,
                                            const bool is_cyclic)
{
  const int bezier_vstep = 3;
  const OffsetIndices points_by_curve = geometry.points_by_curve();
  const int start_verts_count = verts.size();

  for (int i_curve = 0; i_curve < geometry.curve_num; i_curve++) {

    const IndexRange curve_points = points_by_curve[i_curve];
    const int start_point_index = curve_points[0];
    const int last_point_index = curve_points[curve_points.size() - 1];

    for (int i_point = start_point_index; i_point < last_point_index; i_point++) {

      /* The order verts in the USD bezier curve representation is [control point 0, right handle
       * 0, left handle 1, control point 1, right handle 1, left handle 2, control point 2, ...].
       * The last vert in the array doesn't need a right handle because the curve stops at that
       * point. */
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));

      const blender::float3 right_handle = handles_r[i_point];
      verts.push_back(pxr::GfVec3f(right_handle[0], right_handle[1], right_handle[2]));

      const blender::float3 left_handle = handles_l[i_point + 1];
      verts.push_back(pxr::GfVec3f(left_handle[0], left_handle[1], left_handle[2]));
    }

    verts.push_back(pxr::GfVec3f(positions[last_point_index][0],
                                 positions[last_point_index][1],
                                 positions[last_point_index][2]));

    /* For USD representation of periodic bezier curve, one of the curve's points must be repeated
     * to close the curve. The repeated point is the first point. Since the curve is closed, we now
     * need to include the right handle of the last point and the left handle of the first point.
     */
    if (is_cyclic) {
      const blender::float3 right_handle = handles_r[last_point_index];
      verts.push_back(pxr::GfVec3f(right_handle[0], right_handle[1], right_handle[2]));

      const blender::float3 left_handle = handles_l[start_point_index];
      verts.push_back(pxr::GfVec3f(left_handle[0], left_handle[1], left_handle[2]));

      verts.push_back(pxr::GfVec3f(positions[start_point_index][0],
                                   positions[start_point_index][1],
                                   positions[start_point_index][2]));
    }

    const int tot_points = verts.size() - start_verts_count;
    control_point_counts[i_curve] = tot_points;

    if (is_cyclic) {
      segments[i_curve] = tot_points / bezier_vstep;
    }
    else {
      segments[i_curve] = ((tot_points - 4) / bezier_vstep) + 1;
    }
  }
}

static void populate_curve_props_for_bezier(const bke::CurvesGeometry &geometry,
                                            pxr::VtArray<pxr::GfVec3f> &verts,
                                            pxr::VtIntArray &control_point_counts,
                                            pxr::VtArray<float> &widths,
                                            pxr::TfToken &interpolation,
                                            const bool is_cyclic)
{

  const int num_curves = geometry.curve_num;

  const Span<float3> positions = geometry.positions();

  const Span<float3> handles_l = geometry.handle_positions_left();
  const Span<float3> handles_r = geometry.handle_positions_right();

  pxr::VtArray<int> segments(num_curves);

  populate_curve_verts_for_bezier(
      geometry, positions, handles_l, handles_r, verts, control_point_counts, segments, is_cyclic);

  populate_curve_widths(geometry, widths);
  interpolation = get_curve_width_interpolation(widths, segments, control_point_counts, is_cyclic);
}

static void populate_curve_props_for_nurbs(const bke::CurvesGeometry &geometry,
                                           pxr::VtArray<pxr::GfVec3f> &verts,
                                           pxr::VtIntArray &control_point_counts,
                                           pxr::VtArray<float> &widths,
                                           pxr::VtArray<double> &knots,
                                           pxr::VtArray<int> &orders,
                                           pxr::TfToken &interpolation,
                                           const bool is_cyclic)
{
  const int num_curves = geometry.curve_num;
  orders.resize(num_curves);

  const Span<float3> positions = geometry.positions();

  VArray<int8_t> geom_orders = geometry.nurbs_orders();
  VArray<int8_t> knots_modes = geometry.nurbs_knots_modes();

  const OffsetIndices points_by_curve = geometry.points_by_curve();
  for (const int i_curve : geometry.curves_range()) {
    const IndexRange curve_points = points_by_curve[i_curve];
    for (const int i_point : curve_points) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
    }

    const int tot_points = curve_points.size();
    control_point_counts[i_curve] = tot_points;

    const int8_t order = geom_orders[i_curve];
    orders[i_curve] = int(geom_orders[i_curve]);

    const KnotsMode mode = KnotsMode(knots_modes[i_curve]);

    const int knots_num = bke::curves::nurbs::knots_num(tot_points, order, is_cyclic);
    Array<float> temp_knots(knots_num);
    bke::curves::nurbs::calculate_knots(tot_points, mode, order, is_cyclic, temp_knots);

    knots.resize(knots_num);
    for (int i_knot = 0; i_knot < knots_num; i_knot++) {
      knots[i_knot] = double(temp_knots[i_knot]);
    }

    /* Set end knots according to the USD spec for periodic curves
     * https://graphics.pixar.com/usd/dev/api/class_usd_geom_nurbs_curves.html#details */
    if (is_cyclic) {
      knots[0] = knots[1] - (knots[knots.size() - 2] - knots[knots.size() - 3]);
      knots[knots.size() - 1] = knots[knots.size() - 2] + (knots[2] - knots[1]);
    }
    else {
      /* Set end knots according to the USD spec for non-periodic curves */
      knots[0] = knots[1];
      knots[knots.size() - 1] = knots[knots.size() - 2];
    }
  }

  populate_curve_widths(geometry, widths);
  interpolation = pxr::UsdGeomTokens->vertex;
}

void USDCurvesWriter::set_writer_attributes_for_nurbs(const pxr::UsdGeomCurves usd_curves,
                                                      const pxr::VtArray<double> knots,
                                                      const pxr::VtArray<int> orders,
                                                      const pxr::UsdTimeCode timecode)
{
  pxr::UsdAttribute attr_knots =
      pxr::UsdGeomNurbsCurves(usd_curves).CreateKnotsAttr(pxr::VtValue(), true);
  usd_value_writer_.SetAttribute(attr_knots, pxr::VtValue(knots), timecode);
  pxr::UsdAttribute attr_order =
      pxr::UsdGeomNurbsCurves(usd_curves).CreateOrderAttr(pxr::VtValue(), true);
  usd_value_writer_.SetAttribute(attr_order, pxr::VtValue(orders), timecode);
}

void USDCurvesWriter::set_writer_attributes(pxr::UsdGeomCurves &usd_curves,
                                            const pxr::VtArray<pxr::GfVec3f> verts,
                                            const pxr::VtIntArray control_point_counts,
                                            const pxr::VtArray<float> widths,
                                            const pxr::UsdTimeCode timecode,
                                            const pxr::TfToken interpolation)
{
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
}

void USDCurvesWriter::do_write(HierarchyContext &context)
{

  const Curves *curve = converted_curves_ ? converted_curves_.get() :
                                            static_cast<Curves *>(context.object->data);

  const bke::CurvesGeometry &geometry = curve->geometry.wrap();
  if (geometry.points_num() == 0) {
    return;
  }

  const std::array<int, CURVE_TYPES_NUM> curve_type_counts = geometry.curve_type_counts();
  const int number_of_curve_types = std::reduce(
      curve_type_counts.begin(), curve_type_counts.end(), 0, [](int previous_result, int item) {
        return item > 0 ? ++previous_result : previous_result;
      });

  if (number_of_curve_types > 1) {
    WM_report(RPT_WARNING, "Cannot export mixed curve types.");
    return;
  }

  const VArray<bool> cyclic_values = geometry.cyclic();
  const bool is_cyclic = cyclic_values[0];
  bool all_same_cyclic_type = true;

  for (const int i : cyclic_values.index_range()) {
    if (cyclic_values[i] != is_cyclic) {
      all_same_cyclic_type = false;
      break;
    }
  }

  if (!all_same_cyclic_type) {
    WM_report(RPT_WARNING, "Cannot export mixed cyclic and non-cyclic curves.");
    return;
  }

  const pxr::UsdTimeCode timecode = get_export_time_code();
  pxr::UsdGeomCurves usd_curves;

  pxr::VtArray<pxr::GfVec3f> verts;
  pxr::VtIntArray control_point_counts;
  control_point_counts.resize(geometry.curves_num());
  pxr::VtArray<float> widths;
  pxr::TfToken interpolation;

  const int8_t curve_type = geometry.curve_types()[0];
  switch (curve_type) {
    case CURVE_TYPE_POLY:
      usd_curves = DefineUsdGeomBasisCurves(pxr::VtValue(), is_cyclic, false);

      populate_curve_props(
          geometry, verts, control_point_counts, widths, interpolation, is_cyclic, false);
      break;
    case CURVE_TYPE_CATMULL_ROM:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->catmullRom), is_cyclic, true);

      populate_curve_props(
          geometry, verts, control_point_counts, widths, interpolation, is_cyclic, true);
      break;
    case CURVE_TYPE_BEZIER:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->bezier), is_cyclic, true);

      populate_curve_props_for_bezier(
          geometry, verts, control_point_counts, widths, interpolation, is_cyclic);
      break;
    case CURVE_TYPE_NURBS: {
      pxr::VtArray<double> knots;
      pxr::VtArray<int> orders;
      orders.resize(geometry.curves_num());

      usd_curves = pxr::UsdGeomNurbsCurves::Define(usd_export_context_.stage,
                                                   usd_export_context_.usd_path);

      populate_curve_props_for_nurbs(
          geometry, verts, control_point_counts, widths, knots, orders, interpolation, is_cyclic);

      set_writer_attributes_for_nurbs(usd_curves, knots, orders, timecode);

      break;
    }
    default:
      BLI_assert_unreachable();
  }

  set_writer_attributes(usd_curves, verts, control_point_counts, widths, timecode, interpolation);

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
