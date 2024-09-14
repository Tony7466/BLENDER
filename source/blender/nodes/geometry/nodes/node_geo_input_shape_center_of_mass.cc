/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_shape_center_of_mass_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Vector>("Center of Mass").field_source();
}

class ShapeCenterOfMassFieldInput final : public bke::PhysicsFieldInput {
 public:
  ShapeCenterOfMassFieldInput()
      : bke::PhysicsFieldInput(CPPType::get<float3>(), "Shape Center of Mass")
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 const AttrDomain domain,
                                 const IndexMask & /*mask*/) const final
  {
    VArray<float3> mins = VArray<float3>::ForFunc(
        physics.shapes_num(), [&](const int index) -> float3 {
          const Span<bke::CollisionShapePtr> shapes = physics.state().shapes();
          const bke::CollisionShapePtr &shape = shapes[index];
          return shape ? shape->center_of_mass() : float3(0.0f);
        });
    return physics.attributes().adapt_domain<float3>(
        std::move(mins), AttrDomain::Instance, domain);
  }

  uint64_t hash() const override
  {
    /* Some random constant hash. */
    return 2624040564;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    return dynamic_cast<const ShapeCenterOfMassFieldInput *>(&other) != nullptr;
  }

  std::optional<AttrDomain> preferred_domain(
      const bke::PhysicsGeometry & /*physics*/) const override
  {
    return AttrDomain::Instance;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  if (params.output_is_required("Center of Mass")) {
    Field<float3> com_field{std::make_shared<ShapeCenterOfMassFieldInput>()};
    params.set_output("Center of Mass", std::move(com_field));
  }
}

static void node_register()
{
  static blender::bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_SHAPE_CENTER_OF_MASS, "Shape Center of Mass", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shape_center_of_mass_cc
