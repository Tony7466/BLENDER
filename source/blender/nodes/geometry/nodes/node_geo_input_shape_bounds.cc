/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_shape_bounds_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Vector>("Min").field_source();
  b.add_output<decl::Vector>("Max").field_source();
}

class ShapeBoundsMinFieldInput final : public bke::PhysicsFieldInput {
 public:
  ShapeBoundsMinFieldInput() : bke::PhysicsFieldInput(CPPType::get<float3>(), "Shape Bounds Min")
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const Span<bke::CollisionShapePtr> shapes = physics.state().shapes();
    VArray<float3> mins = VArray<float3>::ForFunc(
        physics.shapes_num(), [&](const int index) -> float3 {
          const bke::CollisionShapePtr &shape = shapes[index];
          return shape ? shape->local_bounds().min : float3(0.0f);
        });
    return physics.attributes().adapt_domain<float3>(
        std::move(mins), AttrDomain::Instance, domain);
  }

  uint64_t hash() const override
  {
    /* Some random constant hash. */
    return 2706059866;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    return dynamic_cast<const ShapeBoundsMinFieldInput *>(&other) != nullptr;
  }

  std::optional<AttrDomain> preferred_domain(
      const bke::PhysicsGeometry & /*physics*/) const override
  {
    return AttrDomain::Instance;
  }
};

class ShapeBoundsMaxFieldInput final : public bke::PhysicsFieldInput {
 public:
  ShapeBoundsMaxFieldInput() : bke::PhysicsFieldInput(CPPType::get<float3>(), "Shape Bounds Max")
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    const Span<bke::CollisionShapePtr> shapes = physics.state().shapes();
    VArray<float3> mins = VArray<float3>::ForFunc(
        physics.shapes_num(), [&](const int index) -> float3 {
          const bke::CollisionShapePtr &shape = shapes[index];
          return shape ? shape->local_bounds().max : float3(0.0f);
        });
    return physics.attributes().adapt_domain<float3>(
        std::move(mins), AttrDomain::Instance, domain);
  }

  uint64_t hash() const override
  {
    /* Some random constant hash. */
    return 2459420134;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    return dynamic_cast<const ShapeBoundsMaxFieldInput *>(&other) != nullptr;
  }

  std::optional<AttrDomain> preferred_domain(
      const bke::PhysicsGeometry & /*physics*/) const override
  {
    return AttrDomain::Instance;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  if (params.output_is_required("Min")) {
    Field<float3> min_field{std::make_shared<ShapeBoundsMinFieldInput>()};
    params.set_output("Min", std::move(min_field));
  }
  if (params.output_is_required("Max")) {
    Field<float3> max_field{std::make_shared<ShapeBoundsMaxFieldInput>()};
    params.set_output("Max", std::move(max_field));
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(&ntype, GEO_NODE_INPUT_SHAPE_BOUNDS, "Shape Bounds", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shape_bounds_cc
