/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "BKE_collision_shape.hh"

namespace blender::nodes::node_geo_collision_shape_inertia_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Vector>("Inertia").field_source();
}

class ShapeInertiaFieldInput : public bke::PhysicsFieldInput {
 public:
  ShapeInertiaFieldInput() : PhysicsFieldInput(CPPType::get<float3>(), "Shape Inertia node") {}

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 AttrDomain domain,
                                 const IndexMask & /*mask*/) const override
  {
    if (domain == bke::AttrDomain::Instance) {
      const Span<bke::CollisionShapePtr> shapes = physics.shapes();
      return VArray<float3>::ForFunc(shapes.size(), [shapes](const int index) {
        const bke::CollisionShapePtr &shape = shapes[index];
        return shape->calculate_local_inertia(1.0f);
      });
    }
    return {};
  }

  uint64_t hash() const override
  {
    return 2420616760;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    return dynamic_cast<const ShapeInertiaFieldInput *>(&other) != nullptr;
  }

  std::optional<bke::AttrDomain> preferred_domain(const bke::PhysicsGeometry &curves) const final
  {
    return bke::AttrDomain::Instance;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<float3> inertia_field{std::make_shared<ShapeInertiaFieldInput>()};
  params.set_output("Inertia", std::move(inertia_field));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_COLLISION_SHAPE_INERTIA, "Collision Shape Inertia", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_inertia_cc
