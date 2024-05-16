/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"

#include "BLI_task.hh"

#include "GEO_curve_plane.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_plane_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Vector>("Min")
      .default_value(float3(float2(-1.0f), 0.0f))
      .description("Minimum boundary of volume");
  b.add_input<decl::Vector>("Max")
      .default_value(float3(float2(1.0f), 0.0f))
      .description("Maximum boundary of volume");

  b.add_input<decl::Int>("Resolution X").default_value(32).min(2);
  b.add_input<decl::Int>("Resolution Y").default_value(32).min(2);

  b.add_input<decl::Bool>("Bitmap")
      .description("True or false is a pixel")
      .supports_field()
      .hide_value();

  b.add_output<decl::Geometry>("Curve");
  b.add_output<decl::Int>("Parent Curve").description("Index of curve around").field_on_all();
}

#ifdef WITH_POTRACE

class ImageBitMapFieldContext : public FieldContext {
 private:
  int2 resolution_;
  const float2 min_point_;
  const float2 max_point_;

 public:
  ImageBitMapFieldContext(const int2 resolution, const float2 min_point, const float2 max_point)
      : resolution_(resolution), min_point_(min_point), max_point_(max_point)
  {
  }

  int64_t points_num() const
  {
    return int64_t(resolution_.x) * int64_t(resolution_.y);
  }

  GVArray get_varray_for_input(const FieldInput &field_input,
                               const IndexMask & /*mask*/,
                               ResourceScope & /*scope*/) const
  {
    const bke::AttributeFieldInput *attribute_field_input =
        dynamic_cast<const bke::AttributeFieldInput *>(&field_input);
    if (attribute_field_input == nullptr) {
      return {};
    }
    if (attribute_field_input->attribute_name() != "position") {
      return {};
    }

    Array<float3> positions(this->points_num());

    threading::parallel_for(
        IndexRange(resolution_.y),
        4096,
        [&](const IndexRange range) {
          for (const int y_index : range) {
            int64_t start_offset = y_index * resolution_.x;
            const float y_factor = bke::attribute_math::mix2(
                float(y_index) / float(resolution_.y - 1), min_point_[1], max_point_[1]);
            for (const int64_t x_index : IndexRange(resolution_.x)) {
              const float x_factor = bke::attribute_math::mix2(
                  float(x_index) / float(resolution_.x - 1), min_point_[0], max_point_[0]);
              positions[start_offset + x_index] = float3(x_factor, y_factor, 0.0f);
            }
          }
        },
        threading::accumulated_task_sizes(
            [&](const IndexRange range) { return range.size() * resolution_.x; }));

    return VArray<float3>::ForContainer(std::move(positions));
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  const float2 min_point = params.extract_input<float3>("Min").xy();
  const float2 max_point = params.extract_input<float3>("Max").xy();
  const int2 resolution = int2(params.extract_input<int>("Resolution X"),
                               params.extract_input<int>("Resolution Y"));

  if (resolution.x < 2 || resolution.y < 2) {
    params.error_message_add(NodeWarningType::Error, TIP_("Resolution must be greater than 1"));
    params.set_default_remaining_outputs();
    return;
  }

  if (min_point.x == max_point.x || min_point.y == max_point.y) {
    params.error_message_add(NodeWarningType::Error, TIP_("Plane area must be greater than 0"));
    params.set_default_remaining_outputs();
    return;
  }

  const ImageBitMapFieldContext context(resolution, min_point, max_point);

  FieldEvaluator evaluator(context, context.points_num());

  Field<bool> input_field = params.extract_input<Field<bool>>("Bitmap");
  Array<bool> byte_map(context.points_num());
  evaluator.add_with_destination(std::move(input_field), byte_map.as_mutable_span());
  evaluator.evaluate();

  AnonymousAttributeIDPtr parent_curve_id = params.get_output_anonymous_attribute_id_if_needed(
      "Parent Curve");
  std::optional<Curves *> curve = geometry::plane_to_curve(
      resolution, byte_map, min_point, max_point, parent_curve_id.get());
  if (!curve.has_value()) {
    params.error_message_add(NodeWarningType::Warning, TIP_("Can not generate curve"));
    params.set_default_remaining_outputs();
    return;
  }

  params.set_output("Curve", GeometrySet::from_curves(*curve));
}

#else

static void node_geo_exec(GeoNodeExecParams params)
{
  params.error_message_add(NodeWarningType::Warning, TIP_("Build without Potrace"));
  params.set_default_remaining_outputs();
}

#endif

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_CURVE_PLANE, "Curve Plane", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_plane_cc
