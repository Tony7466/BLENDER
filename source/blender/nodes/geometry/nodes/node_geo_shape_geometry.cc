/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array_utils.hh"

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_instances.hh"
#include "BKE_physics_geometry.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_shape_geometry_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Geometry").propagate_all().align_with_previous();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  /* Share geometry for instances if the collision shapes are the same. */
  Map<const bke::CollisionShape *, GeometrySet> shape_geometry_map;

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (!geometry_set.has_physics()) {
      geometry_set = {};
      return;
    }

    const bke::PhysicsGeometry &physics = *geometry_set.get_physics();
    Span<bke::CollisionShapePtr> shapes = physics.state().shapes();

    bke::Instances *instances = new bke::Instances();
    instances->resize(shapes.size());

    MutableSpan<int> handles = instances->reference_handles_for_write();
    MutableSpan<float4x4> transforms = instances->transforms_for_write();
    for (const int i : shapes.index_range()) {
      const bke::CollisionShape *shape = shapes[i].get();

      const GeometrySet &shape_geometry = shape_geometry_map.lookup_or_add_cb(
          shape, [&]() { return shape->create_geometry(); });
      handles[i] = instances->add_reference(bke::InstanceReference{shape_geometry});
      transforms[i] = float4x4::identity();
    }

    bke::AttributeAccessor src_attributes = physics.attributes();
    bke::AttributeFilter attribute_filter = bke::AttributeFilterFromFunc(
        [&](const StringRef name) {
          if (src_attributes.is_builtin(name)) {
            return bke::AttributeFilter::Result::AllowSkip;
          }
          return bke::AttributeFilter::Result::Process;
        });

    bke::copy_attributes(physics.attributes(),
                         bke::AttrDomain::Instance,
                         bke::AttrDomain::Instance,
                         attribute_filter,
                         instances->attributes_for_write());

    geometry_set = GeometrySet::from_instances(instances);
  });

  params.set_output("Geometry", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SHAPE_GEOMETRY, "Shape Geometry", NODE_CLASS_CONVERTER);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_shape_geometry_cc
