/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2019 Blender Foundation.
 * All rights reserved.
 */
#include "usd_writer_curves.h"
#include "usd_hierarchy_iterator.h"

#include <pxr/usd/usdGeom/basisCurves.h>
#include <pxr/usd/usdGeom/curves.h>
#include <pxr/usd/usdGeom/nurbsCurves.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "BKE_curves.hh"

#include <numeric>

extern "C" {

#include "BKE_material.h"

#include "BLI_math_geom.h"

#include "WM_api.h"
#include "WM_types.h"
}

namespace blender::io::usd {

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx)
    : USDAbstractWriter(ctx), converted_curves(nullptr)
{
}

USDCurvesWriter::USDCurvesWriter(const USDExporterContext &ctx, Curves *converted_legacy_curves)
    : USDAbstractWriter(ctx), converted_curves(converted_legacy_curves)
{
}

/*
void USDCurvesWriter::do_write(HierarchyContext &context)
{
  Curves *curve = static_cast<Curves *>(context.object->data);
  bke::CurvesGeometry &geometry = bke::CurvesGeometry::wrap(curve->geometry);

  pxr::UsdTimeCode timecode = get_export_time_code();

  const int num_curves = geometry.curve_num;
  const Span<float3> positions = geometry.positions();

  const bke::AttributeAccessor curve_attributes = geometry.attributes();
  auto radii = curve_attributes.lookup_or_default<float>("radius", ATTR_DOMAIN_POINT, 0.0f);

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {

    pxr::UsdGeomCurves curve;
    int8_t curve_type = geometry.curve_types()[i_curve];

    switch (curve_type) {
      case CURVE_TYPE_CATMULL_ROM:
        curve = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                 usd_export_context_.usd_path);
        pxr::UsdGeomBasisCurves(curve).CreateBasisAttr(
            pxr::VtValue(pxr::UsdGeomTokens->catmullRom));

        pxr::UsdGeomBasisCurves(curve).CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
        break;
      case CURVE_TYPE_POLY:
        curve = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                 usd_export_context_.usd_path);
        pxr::UsdGeomBasisCurves(curve).CreateBasisAttr(pxr::VtValue(pxr::UsdGeomTokens->bezier));


        pxr::UsdGeomBasisCurves(curve).CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
        return;
      case CURVE_TYPE_BEZIER:
        curve = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                 usd_export_context_.usd_path);
        pxr::UsdGeomBasisCurves(curve).CreateBasisAttr(pxr::VtValue(pxr::UsdGeomTokens->bezier));

        pxr::UsdGeomBasisCurves(curve).CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
        return;
      case CURVE_TYPE_NURBS:
        curve = pxr::UsdGeomNurbsCurves::Define(usd_export_context_.stage,
                                                 usd_export_context_.usd_path);
        break;
      default:
        BLI_assert_unreachable();
    }

    bool cyclic = geometry.cyclic()[i_curve];
    if (cyclic) {
      pxr::UsdGeomBasisCurves(curve).CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->periodic));
    }
    else {
      pxr::UsdGeomBasisCurves(curve).CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->nonperiodic));
    }

    pxr::VtArray<pxr::GfVec3f> verts;
    pxr::VtIntArray curve_point_counts;
    pxr::VtArray<float> widths;

    long tot_points = 0;
    for (const auto i_point : geometry.points_for_curve(i_curve)) {
      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
      widths.push_back(radii[i_point] * 2.0f);
      tot_points++;
    }

    curve_point_counts.push_back(tot_points);

    pxr::UsdAttribute attr_points = curve.CreatePointsAttr(pxr::VtValue(), true);
    usd_value_writer_.SetAttribute(attr_points, pxr::VtValue(verts), timecode);

    pxr::UsdAttribute attr_vertex_counts = curve.CreateCurveVertexCountsAttr(pxr::VtValue(),
                                                                              true);
    usd_value_writer_.SetAttribute(attr_vertex_counts, pxr::VtValue(curve_point_counts), timecode);

    pxr::UsdAttribute attr_widths = curve.CreateWidthsAttr(pxr::VtValue(), true);
    usd_value_writer_.SetAttribute(attr_widths, pxr::VtValue(widths), timecode);
  }


}
*/

pxr::UsdGeomCurves USDCurvesWriter::DefineUsdGeomBasisCurves(pxr::VtValue curve_basis,
                                                             bool cyclic,
                                                             bool cubic)
{
  pxr::UsdGeomCurves curves = pxr::UsdGeomBasisCurves::Define(usd_export_context_.stage,
                                                              usd_export_context_.usd_path);

  // Not required to set the basis attribute for linear curves
  // https://graphics.pixar.com/usd/dev/api/class_usd_geom_basis_curves.html#details
  if (cubic) {
    pxr::UsdGeomBasisCurves(curves).CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->cubic));
    pxr::UsdGeomBasisCurves(curves).CreateBasisAttr(curve_basis);
  }
  else {
    pxr::UsdGeomBasisCurves(curves).CreateTypeAttr(pxr::VtValue(pxr::UsdGeomTokens->linear));
  }

  if (cyclic) {
    pxr::UsdGeomBasisCurves(curves).CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->periodic));
  }
  else {
    pxr::UsdGeomBasisCurves(curves).CreateWrapAttr(pxr::VtValue(pxr::UsdGeomTokens->nonperiodic));
  }

  return curves;
}

void populate_curve_verts(const bke::CurvesGeometry &geometry,
                          pxr::VtArray<pxr::GfVec3f> &verts,
                          pxr::VtIntArray &curve_point_counts,
                          pxr::VtArray<float> &widths)
{
  const int num_curves = geometry.curve_num;
  const Span<float3> positions = geometry.positions();
  const bke::AttributeAccessor curve_attributes = geometry.attributes();

  const VArray<float> radii = curve_attributes.lookup_or_default<float>(
      "radius", ATTR_DOMAIN_POINT, 0.0f);

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {
    auto curve_points = geometry.points_for_curve(i_curve);
    long tot_points = 0;
    for (const auto i_point : curve_points) {
      widths.push_back(radii[i_point] * 2.0f);

      verts.push_back(
          pxr::GfVec3f(positions[i_point][0], positions[i_point][1], positions[i_point][2]));
      tot_points++;
    }

    curve_point_counts.push_back(tot_points);
  }
}

void populate_curve_verts_for_bezier(const bke::CurvesGeometry &geometry,
                                     pxr::VtArray<pxr::GfVec3f> &verts,
                                     pxr::VtIntArray &curve_point_counts,
                                     pxr::VtArray<float> &widths)
{
  const int num_curves = geometry.curve_num;
  const Span<float3> positions = geometry.positions();

  const bke::AttributeAccessor curve_attributes = geometry.attributes();
  const VArray<float> radii = curve_attributes.lookup_or_default<float>(
      "radius", ATTR_DOMAIN_POINT, 0.0f);

  const Span<float3> handles_l = geometry.handle_positions_left();
  const Span<float3> handles_r = geometry.handle_positions_right();

  for (int i_curve = 0; i_curve < num_curves; i_curve++) {
    auto curve_points = geometry.points_for_curve(i_curve);
    long tot_points = 0;
    for (const auto i_point : curve_points) {
      widths.push_back(radii[i_point] * 2.0f);

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
    }

    curve_point_counts.push_back(tot_points);
  }
}

void USDCurvesWriter::do_write(HierarchyContext &context)
{

  Curves *curve = converted_curves ? converted_curves :
                                     static_cast<Curves *>(context.object->data);

  bke::CurvesGeometry &geometry = bke::CurvesGeometry::wrap(curve->geometry);
  if (geometry.points_num() == 0) {
    WM_reportf(RPT_WARNING, "Curve geometry has zero points. Will not export curve.");
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
  if (cyclic_values.size() == 0) {
    WM_reportf(RPT_ERROR, "Cannot determine if curve is cyclic or not. Cannot export curve.");
    return;
  }

  const bool cyclic = cyclic_values[0];
  bool all_same_cyclic_type = true;

  for (int i = 0; i < cyclic_values.size(); i++) {
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
  pxr::VtIntArray curve_point_counts;
  pxr::VtArray<float> widths;

  int8_t curve_type = geometry.curve_types()[0];
  switch (curve_type) {
    case CURVE_TYPE_CATMULL_ROM:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->catmullRom), cyclic, true);

      populate_curve_verts(geometry, verts, curve_point_counts, widths);
      break;
    case CURVE_TYPE_BEZIER:
      usd_curves = DefineUsdGeomBasisCurves(
          pxr::VtValue(pxr::UsdGeomTokens->bezier), cyclic, true);

      populate_curve_verts_for_bezier(geometry, verts, curve_point_counts, widths);
      break;
    case CURVE_TYPE_POLY:
      usd_curves = DefineUsdGeomBasisCurves(pxr::VtValue(), cyclic, false);
      populate_curve_verts(geometry, verts, curve_point_counts, widths);
      break;
    case CURVE_TYPE_NURBS:
      // curves = pxr::UsdGeomNurbsCurves::Define(usd_export_context_.stage,
      //                                          usd_export_context_.usd_path);
      return;
    default:
      BLI_assert_unreachable();
  }

  pxr::UsdAttribute attr_points = usd_curves.CreatePointsAttr(pxr::VtValue(), true);
  usd_value_writer_.SetAttribute(attr_points, pxr::VtValue(verts), timecode);

  pxr::UsdAttribute attr_vertex_counts = usd_curves.CreateCurveVertexCountsAttr(pxr::VtValue(),
                                                                                true);
  usd_value_writer_.SetAttribute(attr_vertex_counts, pxr::VtValue(curve_point_counts), timecode);

  pxr::UsdAttribute attr_widths = usd_curves.CreateWidthsAttr(pxr::VtValue(), true);
  usd_value_writer_.SetAttribute(attr_widths, pxr::VtValue(widths), timecode);
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

  if (!curve_material_bound) {
    /* Either all material slots were empty or there is only one material in use. As geometry
     * subsets are only written when actually used to assign a material, and the mesh already has
     * the material assigned, there is no need to continue. */
    return;
  }
}

}  // namespace blender::io::usd
