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
  b.add_input<decl::Geometry>("Curves").only_realized_data().supported_type(
      GeometryComponent::Type::Curve);

  b.add_input<decl::Float>("Factor").min(0.0f).max(1.0f).subtype(PROP_FACTOR).field_on_all();
  b.add_input<decl::Float>("Length").min(0.0f).subtype(PROP_DISTANCE).field_on_all();

  b.add_input<decl::Int>("Curve Index").min(0.0f).field_on_all();
  b.add_input<decl::Float>("Distance").min(0.0f).subtype(PROP_DISTANCE).field_on_all();

  b.add_output<decl::Float>("Factor").dependent_field();
  b.add_output<decl::Float>("Length").dependent_field();
  b.add_output<decl::Bool>("Valid").dependent_field();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", UI_ITEM_R_EXPAND, nullptr, ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_SHIFT_FOR_CURVE_LENGTH;
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  const GeometryNodeShiftForCurveMode mode = GeometryNodeShiftForCurveMode(node->custom1);

  bNodeSocket *in_curves = static_cast<bNodeSocket *>(node->inputs.first);

  bNodeSocket *in_factor = in_curves->next;
  bNodeSocket *in_length = in_factor->next;

  bNodeSocket *out_factor = static_cast<bNodeSocket *>(node->outputs.first);
  bNodeSocket *out_length = out_factor->next;

  bke::nodeSetSocketAvailability(ntree, in_factor, mode == GEO_NODE_SHIFT_FOR_CURVE_FACTOR);
  bke::nodeSetSocketAvailability(ntree, in_length, mode == GEO_NODE_SHIFT_FOR_CURVE_LENGTH);

  bke::nodeSetSocketAvailability(ntree, out_factor, mode == GEO_NODE_SHIFT_FOR_CURVE_FACTOR);
  bke::nodeSetSocketAvailability(ntree, out_length, mode == GEO_NODE_SHIFT_FOR_CURVE_LENGTH);
}

template<typename Fn>
static void split_mask_predicat(const IndexMask &universe,
                                const Fn &&predicate,
                                IndexMask &true_mask,
                                IndexMask &false_mask,
                                IndexMaskMemory &memory)
{
  true_mask = IndexMask::from_predicate(
      universe, GrainSize(1024), memory, [&](const int index) { return predicate(index); });
  false_mask = IndexMask::from_predicate(
      universe, GrainSize(1024), memory, [&](const int index) { return !predicate(index); });
}

Array<IndexMask> mask_from_groups(const IndexMask &universe,
                                  const Span<int> group_ids,
                                  VectorSet<int> &r_groups,
                                  IndexMaskMemory &memory)
{
  Array<IndexMask> mask_by_group;
  universe.foreach_index([&](const int index) { r_groups.add(group_ids[index]); });
  mask_by_group.reinitialize(r_groups.size());
  IndexMask::from_groups<int>(
      universe,
      memory,
      [&](const int index) { return r_groups.index_of(group_ids[index]); },
      mask_by_group);
  return mask_by_group;
}

static const bke::CurvesGeometry &geometry_to_curves(const GeometrySet &geometry_set)
{
  BLI_assert(geometry_set.has_curves());
  const Curves &curves_id = *geometry_set.get_curves_for_read();
  const bke::CurvesGeometry &curves = curves_id.geometry.wrap();
  BLI_assert(curves.points_num() != 0);
  return curves;
}

static float factor_of_projection(const float3 &point_to_project, const float3 &vector)
{
  return math::dot(point_to_project, vector) / math::dot(vector, vector);
}

static float segment_intersects_sphere(const float3 &segment_vector,
                                       const float3 &sphere_vector,
                                       const float sphere_radius)
{
  const float projection_factor = factor_of_projection(sphere_vector, segment_vector);
  const float3 projection = math::project(sphere_vector, segment_vector);
  const float projection_length = math::distance(projection, sphere_vector);
  if (UNLIKELY(math::abs(projection_length - sphere_radius) < FLT_EPSILON)) {
    return projection_factor;
  }

  const float segment_cos = projection_length / sphere_radius;
  const float segment_sin = math::sqrt(1.0f - segment_cos * segment_cos);

  const float secant_offset = (segment_sin * sphere_radius) / math::length(segment_vector);

  const float result_point_a = projection_factor - secant_offset;
  const float result_point_b = projection_factor + secant_offset;

  const float min_point = math::min(result_point_a, result_point_b);
  const float max_point = math::max(result_point_a, result_point_b);

  if (math::is_in_range(min_point, 0.0f, 1.0f)) {
    return min_point;
  }
  if (math::is_in_range(max_point, 0.0f, 1.0f)) {
    return max_point;
  }

  return math::clamp(min_point, 0.0f, 1.0f);
}

static float segment_intersects_sphere(const float3 &segment_point_a,
                                       const float3 &segment_point_b,
                                       const float3 &sphere_centre,
                                       const float sphere_radius)
{
  BLI_assert(sphere_radius >= 0.0f);
  BLI_assert(math::distance(segment_point_a, sphere_centre) <= sphere_radius ||
             math::distance(segment_point_b, sphere_centre) <= sphere_radius);

  const float3 segment_vector = segment_point_b - segment_point_a;
  const float3 sphere_vector = sphere_centre - segment_point_a;

  return segment_intersects_sphere(segment_vector, sphere_vector, sphere_radius);
}

/**
 * Segment is a vector of current points ot curve offset and next one.
 * Used to solve bridge betwean end and start points.
 * While looking for last point of range of curve points inside a sphere, last segment point should
 * be outside.
 */
static std::optional<int2> target_curve_segment(const Span<float3> positions,
                                                const bool cyclic,
                                                const int start_segment,
                                                const float3 &start_position,
                                                const float distance)
{
  [[maybe_unused]] const auto segment_intersects_sphere = [&](const int2 &segment) -> bool {
    const float3 &position_first = positions[segment[0]];
    const float3 &position_last = positions[segment[1]];
    const bool first_in_sphere = math::distance(start_position, position_first) < distance;
    const bool last_in_sphere = math::distance(start_position, position_last) < distance;
    const bool sphere_in_segment = segment[0] == start_segment;
    /* Both segment points can not be in sphere. Algorithm looking for first segment that going
     * outside of sphere. */
    return first_in_sphere != last_in_sphere || sphere_in_segment;
  };

  const auto segment_out_of_sphere = [&](const int segment_index) -> std::optional<int2> {
    const int2 segment(segment_index, segment_index + 1);
    const float3 &position = positions[segment[1]];
    const bool in_sphere = math::distance(start_position, position) < distance;
    if (!in_sphere) {
      BLI_assert(segment_intersects_sphere(segment));
      return segment;
    }
    return std::nullopt;
  };

  const IndexRange segments_range = positions.index_range().drop_back(1);
  for (const int segment_index : segments_range.drop_front(start_segment)) {
    if (const auto segment = segment_out_of_sphere(segment_index)) {
      return segment;
    }
  }
  if (!cyclic) {
    return std::nullopt;
  }
  {
    const int2 bridge_segment(positions.size() - 1, 0);
    const float3 &position = positions[bridge_segment[1]];
    const bool in_sphere = math::distance(start_position, position) < distance;
    if (!in_sphere) {
      BLI_assert(segment_intersects_sphere(bridge_segment));
      return bridge_segment;
    }
  }
  for (const int segment_index : segments_range.take_front(start_segment)) {
    if (const auto segment = segment_out_of_sphere(segment_index)) {
      return segment;
    }
  }
  return std::nullopt;
}

class ShiftForCurveFunction : public mf::MultiFunction {
 private:
  const GeometrySet curve_geometry_;

 public:
  ShiftForCurveFunction(const GeometrySet curve_geometry) : curve_geometry_(curve_geometry)
  {
    const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
    curves.ensure_evaluated_lengths();
    curves.ensure_can_interpolate_to_evaluated();

    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Shift for Curve", signature};
      builder.single_input<float>("Length");
      builder.single_input<int>("Curve Index");
      builder.single_input<float>("Distance");

      builder.single_output<float>("Length");
      builder.single_output<bool>("Valid");
      return signature;
    }();

    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<float> &factors = params.readonly_single_input<float>(0, "Length");
    const VArray<int> &curve_indices = params.readonly_single_input<int>(1, "Curve Index");
    const VArray<float> &distances = params.readonly_single_input<float>(2, "Distance");
    MutableSpan<float> sampled_values = params.uninitialized_single_output<float>(3, "Length");
    MutableSpan<bool> valid_values = params.uninitialized_single_output<bool>(4, "Valid");

    const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
    const Span<float3> evaluated_positions = curves.evaluated_positions();
    const OffsetIndices<int> points_by_curve = curves.evaluated_points_by_curve();
    const VArray<bool> &cyclic = curves.cyclic();

    const auto shift_for_curve = [&](const int curve_index, const IndexMask &mask) {
      const bool curve_cyclic = cyclic[curve_index];
      const IndexRange points = points_by_curve[curve_index];
      const Span<float3> positions = evaluated_positions.slice(points);
      const Span<float> lengths = curves.evaluated_lengths_for_curve(curve_index, curve_cyclic);

      Array<int> segmet_indices(mask.size());
      Array<float> segmet_factors(mask.size());

      mask.foreach_index([&](const int index, const int pos) {
        const float factor = factors[index];
        length_parameterize::sample_at_length(
            lengths, factor, segmet_indices[pos], segmet_factors[pos]);
      });

      Array<float3> target_positions(mask.size());
      length_parameterize::interpolate<float3>(
          positions, segmet_indices, segmet_factors, target_positions);

      mask.foreach_index([&](const int index, const int pos) {
        const float3 &target_position = target_positions[pos];
        const float target_distances = distances[index];
        const int prev_segment_index = segmet_indices[pos];

        const std::optional<int2> op_segment = target_curve_segment(
            positions, curve_cyclic, prev_segment_index, target_position, target_distances);
        if (!op_segment.has_value()) {
          sampled_values[index] = lengths.last();
          valid_values[index] = false;
          return;
        }
        valid_values[index] = true;
        const int2 &segment = *op_segment;

        const float2 segment_lengths = [&]() {
          const int2 shifted_segment = segment - int2(1);
          if (segment[0] == 0) {
            return float2(0.0f, lengths[shifted_segment[1]]);
          }
          if (segment[1] == 0 && curve_cyclic) {
            return float2(lengths[shifted_segment[0]], lengths[segment[0]]);
          }
          return float2(lengths[shifted_segment[0]], lengths[shifted_segment[1]]);
        }();

        const float3 &position = positions[segment[0]];
        const float3 &position_next = positions[segment[1]];

        if (UNLIKELY(segment[0] == prev_segment_index)) {
          const float length_from_start = math::distance(target_position, position);
          const float segmet_length = segment_lengths[1] - segment_lengths[0];
          const float result_factor = (target_distances + length_from_start) / segmet_length;
          sampled_values[index] = math::interpolate(
              segment_lengths[0], segment_lengths[1], result_factor);
          return;
        }

        const float result_factor = segment_intersects_sphere(
            position, position_next, target_position, target_distances);
        sampled_values[index] = math::interpolate(
            segment_lengths[0], segment_lengths[1], result_factor);
      });
    };

    if (const std::optional<int> curve_index = curve_indices.get_if_single()) {
      if (curves.curves_range().contains(*curve_index)) {
        shift_for_curve(*curve_index, mask);
        return;
      }
      index_mask::masked_fill(sampled_values, 0.0f, mask);
      index_mask::masked_fill(valid_values, false, mask);
      return;
    }

    VArraySpan<int> curve_indices_span(curve_indices);
    IndexMaskMemory memory;
    IndexMask mask_valid;
    IndexMask mask_invalid;
    split_mask_predicat(
        mask,
        [&](const int index) {
          const int curve_index = curve_indices_span[index];
          return curves.curves_range().contains(curve_index);
        },
        mask_valid,
        mask_invalid,
        memory);

    VectorSet<int> used_curves;
    Array<IndexMask> mask_by_curve = mask_from_groups(
        mask_valid, curve_indices_span, used_curves, memory);
    for (const int i : mask_by_curve.index_range()) {
      shift_for_curve(used_curves[i], mask_by_curve[i]);
    }

    index_mask::masked_fill(sampled_values, 0.0f, mask_invalid);
    index_mask::masked_fill(valid_values, false, mask_invalid);
  }
};

class ClipInCurveFunction : public mf::MultiFunction {
 private:
  const GeometrySet curve_geometry_;

 public:
  enum class Mode : int8_t {
    ClipFactor,
    ClipLength,
  };

 private:
  const Mode mode_;

 public:
  ClipInCurveFunction(const GeometrySet curve_geometry, const Mode mode)
      : curve_geometry_(curve_geometry), mode_(mode)
  {
    if (mode == Mode::ClipLength) {
      const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
      curves.ensure_evaluated_lengths();
    }

    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Clip In Curve", signature};
      builder.single_input<int>("Curve Index");
      builder.single_input<float>("Value");

      builder.single_output<float>("Value");
      return signature;
    }();

    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<int> indices = params.readonly_single_input<int>(0, "Curve Index");
    const VArray<float> length_or_factor_input = params.readonly_single_input<float>(1, "Value");
    MutableSpan<float> length_or_factor_output = params.uninitialized_single_output<float>(
        2, "Value");

    const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
    const VArray<bool> cyclic = curves.cyclic();
    switch (this->mode_) {
      case Mode::ClipFactor:
        mask.foreach_index_optimized<int>([&](const int index) {
          const bool curve_cyclic = cyclic[index];
          const float factor = length_or_factor_input[index];
          if (curve_cyclic) {
            length_or_factor_output[index] = math::fract(factor);
          }
          else {
            length_or_factor_output[index] = math::clamp(factor, 0.0f, 1.0f);
          }
        });
        break;
      case Mode::ClipLength:
        mask.foreach_index_optimized<int>([&](const int index) {
          const int curve_index = indices[index];
          const bool curve_cyclic = cyclic[index];
          const float total_curve_length = curves.evaluated_length_total_for_curve(curve_index,
                                                                                   curve_cyclic);
          const float factor = length_or_factor_input[index] / total_curve_length;
          if (curve_cyclic) {
            length_or_factor_output[index] = math::fract(factor) * total_curve_length;
          }
          else {
            length_or_factor_output[index] = math::clamp(factor, 0.0f, 1.0f) * total_curve_length;
          }
        });
        break;
    }
  }
};

class CurveFactorOrLengthFunction : public mf::MultiFunction {
 private:
  const GeometrySet curve_geometry_;

 public:
  enum class Mode : int8_t {
    FactorToLength,
    LengthToFactor,
  };

 private:
  const Mode mode_;

 public:
  CurveFactorOrLengthFunction(const GeometrySet curve_geometry, const Mode mode)
      : curve_geometry_(curve_geometry), mode_(mode)
  {
    const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
    curves.ensure_evaluated_lengths();

    static const mf::Signature signature = []() {
      mf::Signature signature;
      mf::SignatureBuilder builder{"Curve Factor or Length", signature};
      builder.single_input<int>("Curve Index");
      builder.single_input<float>("Value");

      builder.single_output<float>("Value");
      return signature;
    }();

    this->set_signature(&signature);
  }

  void call(const IndexMask &mask, mf::Params params, mf::Context /*context*/) const override
  {
    const VArray<int> indices = params.readonly_single_input<int>(0, "Curve Index");
    const VArray<float> length_or_factor_input = params.readonly_single_input<float>(1, "Value");
    MutableSpan<float> length_or_factor_output = params.uninitialized_single_output<float>(
        2, "Value");

    const bke::CurvesGeometry &curves = geometry_to_curves(this->curve_geometry_);
    const VArray<bool> cyclic = curves.cyclic();
    switch (this->mode_) {
      case Mode::FactorToLength:
        mask.foreach_index_optimized<int>([&](const int index) {
          const int curve_index = indices[index];
          const float total_curve_length = curves.evaluated_length_total_for_curve(curve_index,
                                                                                   cyclic[index]);
          const float length = length_or_factor_input[index] * total_curve_length;
          length_or_factor_output[index] = length;
        });
        break;
      case Mode::LengthToFactor:
        mask.foreach_index_optimized<int>([&](const int index) {
          const int curve_index = indices[index];
          const float total_curve_length = curves.evaluated_length_total_for_curve(curve_index,
                                                                                   cyclic[index]);
          const float factor = length_or_factor_input[index] / total_curve_length;
          length_or_factor_output[index] = factor;
        });
        break;
    }
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const GeometrySet geometry_set = params.extract_input<GeometrySet>("Curves");
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

  const Field<int> index_field = params.extract_input<Field<int>>("Curve Index");
  Field<float> distance_field = params.extract_input<Field<float>>("Distance");

  static auto unit_distance_fn = mf::build::SI1_SO<float, float>(
      "Unit Distance",
      [](const float &distance) { return math::max(0.0f, distance); },
      mf::build::exec_presets::AllSpanOrSingle());
  auto unit_distance_op = FieldOperation::Create(std::move(unit_distance_fn),
                                                 {std::move(distance_field)});
  Field<float> unit_distances = Field<float>(unit_distance_op, 0);

  const GeometryNodeShiftForCurveMode mode = GeometryNodeShiftForCurveMode(params.node().custom1);

  Vector<GField, 2> clipped_length_inputs;
  clipped_length_inputs.append(index_field);
  switch (mode) {
    case GEO_NODE_SHIFT_FOR_CURVE_FACTOR: {
      Field<float> factor_field = params.extract_input<Field<float>>("Factor");
      auto factor_to_length_fn = std::make_unique<CurveFactorOrLengthFunction>(
          geometry_set, CurveFactorOrLengthFunction::Mode::FactorToLength);
      auto factor_to_length_op = FieldOperation::Create(std::move(factor_to_length_fn),
                                                        {index_field, std::move(factor_field)});
      clipped_length_inputs.append(Field<float>(factor_to_length_op, 0));
      break;
    }
    case GEO_NODE_SHIFT_FOR_CURVE_LENGTH:
      Field<float> length_field = params.extract_input<Field<float>>("Length");
      clipped_length_inputs.append(std::move(length_field));
      break;
  }

  auto length_clip_fn = std::make_unique<ClipInCurveFunction>(
      geometry_set, ClipInCurveFunction::Mode::ClipLength);
  auto length_clip_op = FieldOperation::Create(std::move(length_clip_fn),
                                               std::move(clipped_length_inputs));
  Field<float> clipped_length = Field<float>(length_clip_op, 0);

  std::shared_ptr<FieldOperation> shift_for_curve_op = FieldOperation::Create(
      std::make_unique<ShiftForCurveFunction>(geometry_set),
      {std::move(clipped_length), index_field, std::move(unit_distances)});
  Field<float> shift_for_curve(shift_for_curve_op, 0);

  switch (mode) {
    case GEO_NODE_SHIFT_FOR_CURVE_FACTOR: {
      auto length_to_factor_fn = std::make_unique<CurveFactorOrLengthFunction>(
          geometry_set, CurveFactorOrLengthFunction::Mode::LengthToFactor);
      auto length_to_factor_op = FieldOperation::Create(std::move(length_to_factor_fn),
                                                        {index_field, std::move(shift_for_curve)});
      params.set_output("Factor", Field<float>(length_to_factor_op, 0));
      break;
    }
    case GEO_NODE_SHIFT_FOR_CURVE_LENGTH:
      params.set_output("Length", std::move(shift_for_curve));
      break;
  }

  params.set_output("Valid", Field<bool>(shift_for_curve_op, 1));
}

}  // namespace blender::nodes::node_geo_shift_for_curve_cc

void register_node_type_geo_shift_for_curve()
{
  namespace file_ns = blender::nodes::node_geo_shift_for_curve_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SHIFT_FOR_CURVE, "Shift for Curve", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.updatefunc = file_ns::node_update;
  ntype.draw_buttons = file_ns::node_layout;
  nodeRegisterType(&ntype);
}
