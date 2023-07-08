/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_length_parameterize.hh"
#include "BLI_math_vector.hh"

#include "BKE_curves.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_socket_search_link.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_shift_for_curve_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curves").only_realized_data().supported_type(GeometryComponent::Type::Curve);

  b.add_input<decl::Float>("Factor").min(0.0f).max(1.0f).subtype(PROP_FACTOR).field_on_all();
  b.add_input<decl::Int>("Curve Index").min(0.0f).field_on_all();
  b.add_input<decl::Float>("Distance").subtype(PROP_DISTANCE).field_on_all();

  b.add_output<decl::Float>("Factor").dependent_field();
}

class ShiftForCurveFunction : public mf::MultiFunction {
 private:
  GeometrySet geometry_set_;
  mf::Signature signature_;

 public:
  ShiftForCurveFunction(GeometrySet geometry_set) : geometry_set_(std::move(geometry_set))
  {
    BLI_assert(geometry_set_.has_curves());
    mf::SignatureBuilder builder{"Shift for Curve", signature_};
    builder.single_input<float>("Factor");
    builder.single_input<int>("Curve Index");
    builder.single_input<float>("Distance");
    builder.single_output<float>("Factor");
    this->set_signature(&signature_);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> factors = params.readonly_single_input<float>(0, "Factor");
    const VArray<int> curve_indices = params.readonly_single_input<int>(1, "Curve Index");
    const VArray<float> distances = params.readonly_single_input<float>(2, "Distance");
    MutableSpan<float> sampled_values = params.uninitialized_single_output<float>(3, "Factor");

    const Curves &curves_id = *geometry_set_.get_curves_for_read();
    const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
    BLI_assert(curves.points_num() > 0);
    curves.ensure_can_interpolate_to_evaluated();
    const Span<float3> evaluated_positions = curves.evaluated_positions();
    const OffsetIndices evaluated_points_by_curve = curves.evaluated_points_by_curve();
    const VArray<bool> cyclic = curves.cyclic();

    const auto shift_for_curve = [&](const int curve_index, const IndexMask &mask) {
      const IndexRange evaluated_points = evaluated_points_by_curve[curve_index];
      const Span<float> accumulated_lengths = curves.evaluated_lengths_for_curve(curve_index, cyclic[curve_index]);

      Array<int> segmet_indices(mask.size());
      Array<float> segmet_factors(mask.size());

      mask.foreach_index([&](const int index, const int pos){
        const float factor = factors[index];
        length_parameterize::sample_at_length(accumulated_lengths, factor, segmet_indices[pos], segmet_factors[pos]);
      });

      Array<float3> target_positions(mask.size());
      length_parameterize::interpolate<float3>(evaluated_positions.slice(evaluated_points), segmet_indices, segmet_factors, target_positions);

      mask.foreach_index([&](const int index, const int pos){
        const float3 &target_position = target_positions[pos];
        const float target_distances = distances[index];
        const int prev_segment_index = segmet_indices[pos];

        const int segment_index = [&]() -> int {
          const IndexRange next_range = evaluated_points.index_range().drop_back(1).drop_front(prev_segment_index);
          for (const int index : next_range) {
            const float3 &position = evaluated_positions[evaluated_points[index + 1]];
            const bool in_range = math::distance_manhattan(target_position, position) < target_distances;
            if (!in_range) {
              return index;
            }
          }
          return prev_segment_index;
        }();

        const float3 position = evaluated_positions[segment_index];
        const float3 position_next = evaluated_positions[segment_index + 1];
        
        const float l1 = math::distance_manhattan(target_position, position);
        const float l2 = math::distance_manhattan(target_position, position_next);
        
        const float factor = (target_distances - l1) / (l2 - l1);
        
        float length;
        length_parameterize::interpolate<float>(accumulated_lengths, {segment_index}, {factor}, MutableSpan(&length, 1));
        sampled_values[index] = length;
      });
    };

    if (const std::optional<int> curve_index = curve_indices.get_if_single()) {
      if (curves.curves_range().contains(*curve_index)) {
        shift_for_curve(*curve_index, mask);
      }
      else {
        return;
      }
    }
    else {
      Vector<int> invalid_indices;
      VectorSet<int> used_curves;
      devirtualize_varray(curve_indices, [&](const auto curve_indices) {
        mask.foreach_index([&](const int index) {
          const int curve_index = curve_indices[index];
          if (curves.curves_range().contains(curve_index)) {
            used_curves.add(curve_index);
          }
          else {
            invalid_indices.append(index);
          }
        });
      });

      IndexMaskMemory memory;
      Array<IndexMask> mask_by_curve(used_curves.size());
      IndexMask::from_groups<int>(
          mask,
          memory,
          [&](const int index) { return used_curves.index_of(curve_indices[index]); },
          mask_by_curve);

      for (const int index : mask_by_curve.index_range()) {
        shift_for_curve(used_curves[index], mask_by_curve[index]);
      }
      return;
    }
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Curves");
  if (!geometry_set.has_curves()) {
    params.set_default_remaining_outputs();
    return;
  }

  const Curves &curves_id = *geometry_set.get_curves_for_read();
  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  if (curves.points_num() == 0) {
    params.set_default_remaining_outputs();
    return;
  }

  curves.ensure_evaluated_lengths();

  Field<float> length_field = params.extract_input<Field<float>>("Factor");
  Field<int> index_field = params.extract_input<Field<int>>("Curve Index");
  Field<float> distance_field = params.extract_input<Field<float>>("Distance");

  std::shared_ptr<FieldOperation> sample_op = FieldOperation::Create(
        std::make_unique<ShiftForCurveFunction>(std::move(geometry_set)),
        {std::move(length_field), std::move(index_field), std::move(distance_field)});

  params.set_output("Factor", Field<float>(sample_op, 0));
}

}  // namespace blender::nodes::node_geo_shift_for_curve_cc

void register_node_type_geo_shift_for_curve()
{
  namespace file_ns = blender::nodes::node_geo_shift_for_curve_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SHIFT_FOR_CURVE, "Shift for Curve", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
