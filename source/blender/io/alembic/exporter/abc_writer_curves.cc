/* SPDX-FileCopyrightText: 2016 Kévin Dietrich. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup balembic
 */

#include <functional>
#include <memory>

#include "abc_writer_curves.h"
#include "intern/abc_axis_conversion.h"

#include "DNA_curve_types.h"
#include "DNA_object_types.h"

#include "BKE_curve_legacy_convert.hh"
#include "BKE_curves.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.hh"
#include "BKE_object.hh"

#include "CLG_log.h"
static CLG_LogRef LOG = {"io.alembic"};

using Alembic::AbcGeom::OCompoundProperty;
using Alembic::AbcGeom::OCurves;
using Alembic::AbcGeom::OCurvesSchema;
using Alembic::AbcGeom::OInt16Property;
using Alembic::AbcGeom::ON3fGeomParam;
using Alembic::AbcGeom::OV2fGeomParam;

namespace blender::io::alembic {
#pragma optimize("", off)
const std::string ABC_CURVE_RESOLUTION_U_PROPNAME("blender:resolution");

ABCCurveWriter::ABCCurveWriter(const ABCWriterConstructorArgs &args) : ABCAbstractWriter(args) {}

void ABCCurveWriter::create_alembic_objects(const HierarchyContext *context)
{
  CLOG_INFO(&LOG, 2, "exporting %s", args_.abc_path.c_str());
  abc_curve_ = OCurves(args_.abc_parent, args_.abc_name, timesample_index_);
  abc_curve_schema_ = abc_curve_.getSchema();

  Curve *cu = static_cast<Curve *>(context->object->data);
  OCompoundProperty user_props = abc_curve_schema_.getUserProperties();
  OInt16Property user_prop_resolu(user_props, ABC_CURVE_RESOLUTION_U_PROPNAME);
  user_prop_resolu.set(cu->resolu);
}

Alembic::Abc::OObject ABCCurveWriter::get_alembic_object() const
{
  return abc_curve_;
}

Alembic::Abc::OCompoundProperty ABCCurveWriter::abc_prop_for_custom_props()
{
  return abc_schema_prop_for_custom_props(abc_curve_schema_);
}

void ABCCurveWriter::do_write(HierarchyContext &context)
{
  Curves *curves;
  std::unique_ptr<Curves, std::function<void(Curves *)>> converted_curves;

  switch (context.object->type) {
    case OB_CURVES_LEGACY: {
      const Curve *legacy_curve = static_cast<Curve *>(context.object->data);
      converted_curves = std::unique_ptr<Curves, std::function<void(Curves *)>>(
          bke::curve_legacy_to_curves(*legacy_curve), [](Curves *c) { BKE_id_free(nullptr, c); });
      curves = converted_curves.get();
      break;
    }
    case OB_CURVES:
      curves = static_cast<Curves *>(context.object->data);
      break;
    default:
      BLI_assert_unreachable();
      return;
  }

  const bke::CurvesGeometry &geometry = curves->geometry.wrap();
  if (geometry.points_num() == 0) {
    return;
  }

  /* Alembic only supports 1 curve type / periodicity combination per object. Enforce this here.
   * See: Alembic source code for OCurves.h as no documentation explicitly exists for this. */
  const std::array<int, CURVE_TYPES_NUM> &curve_type_counts = geometry.curve_type_counts();
  const int number_of_curve_types = std::count_if(curve_type_counts.begin(),
                                                  curve_type_counts.end(),
                                                  [](const int count) { return count > 0; });
  if (number_of_curve_types > 1) {
    CLOG_WARN(&LOG, "Cannot export mixed curve types in the same Curves object");
    return;
  }

  const VArray<bool> cyclic_values = geometry.cyclic();
  const bool is_cyclic = cyclic_values[0];
  bool all_same_cyclic_type = std::all_of(
      cyclic_values.index_range().begin(), cyclic_values.index_range().end(), [&](const int i) {
        return cyclic_values[i] == is_cyclic;
      });
  if (!all_same_cyclic_type) {
    CLOG_WARN(&LOG, "Cannot export mixed cyclic and non-cyclic curves in the same Curves object");
    return;
  }

  Alembic::AbcGeom::BasisType curve_basis = Alembic::AbcGeom::kNoBasis;
  Alembic::AbcGeom::CurveType curve_type = Alembic::AbcGeom::kVariableOrder;
  Alembic::AbcGeom::CurvePeriodicity periodicity = is_cyclic ? Alembic::AbcGeom::kPeriodic :
                                                               Alembic::AbcGeom::kNonPeriodic;
  const int8_t blender_curve_type = geometry.curve_types().first();
  switch (blender_curve_type) {
    case CURVE_TYPE_POLY:
      curve_basis = Alembic::AbcGeom::kNoBasis;
      curve_type = Alembic::AbcGeom::kVariableOrder;
      break;
    case CURVE_TYPE_CATMULL_ROM:
      curve_basis = Alembic::AbcGeom::kCatmullromBasis;
      curve_type = Alembic::AbcGeom::kVariableOrder;
      break;
    case CURVE_TYPE_BEZIER:
      curve_basis = Alembic::AbcGeom::kBezierBasis;
      curve_type = Alembic::AbcGeom::kCubic;
      break;
    case CURVE_TYPE_NURBS:
      curve_basis = Alembic::AbcGeom::kBsplineBasis;
      curve_type = Alembic::AbcGeom::kVariableOrder;
      break;
  }

  std::vector<Imath::V3f> verts;
  std::vector<int32_t> vert_counts;
  std::vector<float> widths;
  std::vector<float> weights;
  std::vector<float> knots;
  std::vector<uint8_t> orders;
  Imath::V3f temp_vert;

  const Span<float3> positions = geometry.positions();
  const Span<float3> handles_l = geometry.handle_positions_left();
  const Span<float3> handles_r = geometry.handle_positions_right();
  const Span<float> nurbs_weights = geometry.nurbs_weights();
  const VArray<int8_t> nurbs_orders = geometry.nurbs_orders();
  const bke::AttributeAccessor curve_attributes = geometry.attributes();
  const bke::AttributeReader<float> radii = curve_attributes.lookup<float>("radius",
                                                                           bke::AttrDomain::Point);

  const OffsetIndices points_by_curve = geometry.points_by_curve();
  for (const int i_curve : geometry.curves_range()) {
    const IndexRange points = points_by_curve[i_curve];
    const size_t current_vert_count = verts.size();

    switch (blender_curve_type) {
      case CURVE_TYPE_POLY:
      case CURVE_TYPE_CATMULL_ROM:
        for (const int i_point : points) {
          copy_yup_from_zup(temp_vert.getValue(), positions[i_point]);
          verts.push_back(temp_vert);
          widths.push_back(radii.varray[i_point] * 2.0f);
        }
        break;

      case CURVE_TYPE_BEZIER: {
        const int start_point_index = points.first();
        const int last_point_index = points.last();

        /* Vert order in the bezier curve representation is [
         *   control point 0(+ width), right handle 0, left handle 1,
         *   control point 1(+ width), right handle 1, left handle 2,
         *   control point 2(+ width), ...] */
        for (int i_point = start_point_index; i_point < last_point_index; i_point++) {
          copy_yup_from_zup(temp_vert.getValue(), positions[i_point]);
          verts.push_back(temp_vert);
          widths.push_back(radii.varray[last_point_index] * 2.0f);

          copy_yup_from_zup(temp_vert.getValue(), handles_r[i_point]);
          verts.push_back(temp_vert);

          copy_yup_from_zup(temp_vert.getValue(), handles_l[i_point + 1]);
          verts.push_back(temp_vert);
        }

        /* The last vert in the array doesn't need a right handle because the curve stops
         * at that point. */
        copy_yup_from_zup(temp_vert.getValue(), positions[last_point_index]);
        verts.push_back(temp_vert);
        widths.push_back(radii.varray[last_point_index] * 2.0f);

        /* If the curve is cyclic, include the right handle of the last point and the
         * left handle of the first point. */
        if (is_cyclic) {
          copy_yup_from_zup(temp_vert.getValue(), handles_r[last_point_index]);
          verts.push_back(temp_vert);

          copy_yup_from_zup(temp_vert.getValue(), handles_l[start_point_index]);
          verts.push_back(temp_vert);
        }

      } break;

      case CURVE_TYPE_NURBS:
        for (const int i_point : points) {
          copy_yup_from_zup(temp_vert.getValue(), positions[i_point]);
          verts.push_back(temp_vert);
          weights.push_back(nurbs_weights[i_point]);
          widths.push_back(radii.varray[i_point] * 2.0f);
        }
        break;
    }

    orders.push_back(nurbs_orders[i_curve]);
    vert_counts.push_back(verts.size() - current_vert_count);
  }

  Alembic::AbcGeom::OFloatGeomParam::Sample width_sample;
  width_sample.setVals(widths);

  OCurvesSchema::Sample sample(verts,
                               vert_counts,
                               curve_type,
                               periodicity,
                               width_sample,
                               OV2fGeomParam::Sample(), /* UVs */
                               ON3fGeomParam::Sample(), /* normals */
                               curve_basis,
                               weights,
                               orders,
                               knots);

  update_bounding_box(context.object);
  sample.setSelfBounds(bounding_box_);
  abc_curve_schema_.set(sample);
}

ABCCurveMeshWriter::ABCCurveMeshWriter(const ABCWriterConstructorArgs &args)
    : ABCGenericMeshWriter(args)
{
}

Mesh *ABCCurveMeshWriter::get_export_mesh(Object *object_eval, bool &r_needsfree)
{
  Mesh *mesh_eval = BKE_object_get_evaluated_mesh(object_eval);
  if (mesh_eval != nullptr) {
    /* Mesh_eval only exists when generative modifiers are in use. */
    r_needsfree = false;
    return mesh_eval;
  }

  r_needsfree = true;
  return BKE_mesh_new_nomain_from_curve(object_eval);
}

}  // namespace blender::io::alembic
