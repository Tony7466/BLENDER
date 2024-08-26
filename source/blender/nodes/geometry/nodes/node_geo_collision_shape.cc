/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_instances.hh"
#include "BKE_physics_geometry.hh"

#include "DNA_curves_types.h"
#include "DNA_mesh_types.h"
#include "DNA_pointcloud_types.h"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_collision_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Scale").default_value(1.0f);
  b.add_input<decl::Vector>("Size", "SizeVector").default_value(float3(1.0f));
  b.add_input<decl::Float>("Radius").default_value(1.0f);
  b.add_input<decl::Float>("Height").default_value(1.0f);
  b.add_input<decl::Vector>("Plane Normal").default_value(float3(0, 0, 1));
  b.add_input<decl::Float>("Plane Constant").default_value(0.0f);
  b.add_input<decl::Vector>("Point", "Point0");
  b.add_input<decl::Vector>("Point", "Point1");
  b.add_input<decl::Vector>("Point", "Point2");
  b.add_input<decl::Geometry>("Geometry")
      .supported_type({GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::Curve,
                       GeometryComponent::Type::PointCloud});
  b.add_input<decl::Geometry>("Mesh").supported_type(GeometryComponent::Type::Mesh);
  b.add_input<decl::Geometry>("Child Shape").supported_type(GeometryComponent::Type::Physics);
  b.add_output<decl::Geometry>("Shape").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "type", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  node->custom1 = int(ShapeType::Box);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  const ShapeType type = ShapeType(node->custom1);

  bNodeSocket *socket_iter = static_cast<bNodeSocket *>(node->inputs.first);
  auto next_socket = [&socket_iter]() {
    bNodeSocket *result = socket_iter;
    BLI_assert(result != nullptr);
    socket_iter = socket_iter->next;
    return result;
  };

  bNodeSocket *scale_socket = next_socket();
  bNodeSocket *size_vector_socket = next_socket();
  bNodeSocket *radius_socket = next_socket();
  bNodeSocket *height_socket = next_socket();
  bNodeSocket *plane_normal_socket = next_socket();
  bNodeSocket *plane_constant_socket = next_socket();
  bNodeSocket *point0_socket = next_socket();
  bNodeSocket *point1_socket = next_socket();
  bNodeSocket *point2_socket = next_socket();
  bNodeSocket *geometry_socket = next_socket();
  bNodeSocket *mesh_socket = next_socket();
  bNodeSocket *child_shape_socket = next_socket();

  bke::node_set_socket_availability(tree, scale_socket, ELEM(type, ShapeType::UniformScaling));
  bke::node_set_socket_availability(tree, size_vector_socket, ELEM(type, ShapeType::Box));
  bke::node_set_socket_availability(
      tree,
      radius_socket,
      ELEM(type, ShapeType::Sphere, ShapeType::Cylinder, ShapeType::Cone, ShapeType::Capsule));
  bke::node_set_socket_availability(
      tree, height_socket, ELEM(type, ShapeType::Cylinder, ShapeType::Cone, ShapeType::Capsule));
  bke::node_set_socket_availability(tree, plane_normal_socket, ELEM(type, ShapeType::StaticPlane));
  bke::node_set_socket_availability(
      tree, plane_constant_socket, ELEM(type, ShapeType::StaticPlane));
  const bool use_points = ELEM(type, ShapeType::Triangle);
  bke::node_set_socket_availability(tree, point0_socket, use_points);
  bke::node_set_socket_availability(tree, point1_socket, use_points);
  bke::node_set_socket_availability(tree, point2_socket, use_points);
  bke::node_set_socket_availability(tree, geometry_socket, ELEM(type, ShapeType::ConvexHull));
  bke::node_set_socket_availability(tree, mesh_socket, ELEM(type, ShapeType::TriangleMesh));
  bke::node_set_socket_availability(
      tree, child_shape_socket, ELEM(type, ShapeType::UniformScaling, ShapeType::Compound));
}

static VArray<float3> gather_points(const GeometrySet &geometry_set)
{
  int span_count = 0;
  int count = 0;
  int total_num = 0;

  Span<float3> positions_span;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    count++;
    if (const VArray positions = *mesh->attributes().lookup<float3>("position")) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    count++;
    if (const VArray positions = *points->attributes().lookup<float3>("position")) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    count++;
    span_count++;
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    positions_span = curves.evaluated_positions();
    total_num += positions_span.size();
  }

  if (count == 0) {
    return nullptr;
  }

  /* If there is only one positions virtual array and it is already contiguous, avoid copying
   * all of the positions and instead pass the span directly to the convex hull function. */
  if (span_count == 1 && count == 1) {
    return VArray<float3>::ForSpan(positions_span);
  }

  Array<float3> positions(total_num);
  int offset = 0;

  if (const Mesh *mesh = geometry_set.get_mesh()) {
    if (const VArray varray = *mesh->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const PointCloud *points = geometry_set.get_pointcloud()) {
    if (const VArray varray = *points->attributes().lookup<float3>("position")) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves()) {
    const bke::CurvesGeometry &curves = curves_id->geometry.wrap();
    Span<float3> array = curves.evaluated_positions();
    positions.as_mutable_span().slice(offset, array.size()).copy_from(array);
    offset += array.size();
  }

  return VArray<float3>::ForContainer(positions);
}

static bke::CollisionShape::Ptr get_convex_collision_shape(const bke::GeometrySet &geometry_set)
{
  if (!geometry_set.has_physics()) {
    return nullptr;
  }
  const bke::PhysicsGeometry *physics = geometry_set.get_physics();
  if (physics->shapes_num() == 0) {
    return nullptr;
  }
  const bke::CollisionShape::Ptr child_shape = physics->state().shapes().first();
  if (!child_shape->is_convex()) {
    return nullptr;
  }
  return child_shape;
}

/* Keep in sync with the type_items enum in node_rna. */
static bke::CollisionShape *make_collision_shape_from_type(
    const bke::CollisionShape::ShapeType type, GeoNodeExecParams params)
{
  using ShapeType = bke::CollisionShape::ShapeType;

  switch (type) {
    case ShapeType::Invalid: {
      return nullptr;
    }
    case ShapeType::Empty: {
      return new bke::EmptyCollisionShape();
    }
    case ShapeType::Box: {
      const float3 half_extent = 0.5f * params.extract_input<float3>("SizeVector");
      return new bke::BoxCollisionShape(half_extent);
    }
    case ShapeType::Triangle: {
      const float3 point0 = params.extract_input<float3>("Point0");
      const float3 point1 = params.extract_input<float3>("Point1");
      const float3 point2 = params.extract_input<float3>("Point2");
      return new bke::TriangleCollisionShape(point0, point1, point2);
    }
    case ShapeType::Tetrahedral: {
      return nullptr;
    }
    case ShapeType::ConvexTriangleMesh: {
      return nullptr;
    }
    case ShapeType::ConvexHull: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
      const VArray<float3> points = gather_points(geometry_set);
      return new bke::ConvexHullCollisionShape(points);
    }
    case ShapeType::ConvexPointCloud: {
      return nullptr;
    }
    case ShapeType::Sphere: {
      const float radius = params.extract_input<float>("Radius");
      return new bke::SphereCollisionShape(radius);
    }
    case ShapeType::MultiSphere: {
      return nullptr;
    }
    case ShapeType::Capsule: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return new bke::CapsuleCollisionShape(radius, height);
    }
    case ShapeType::Cone: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return new bke::ConeCollisionShape(radius, height);
    }
    case ShapeType::Cylinder: {
      const float radius = params.extract_input<float>("Radius");
      const float height = params.extract_input<float>("Height");
      return new bke::CylinderCollisionShape(radius, height);
    }
    case ShapeType::UniformScaling: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const float scale = params.extract_input<float>("Scale");
      const bke::CollisionShape::Ptr child_shape = get_convex_collision_shape(geometry_set);
      if (!child_shape) {
        return nullptr;
      }
      return new bke::UniformScalingCollisionShape(child_shape, scale);
    }
    case ShapeType::MinkowskiSum: {
      return nullptr;
    }
    case ShapeType::MinkowskiDifference: {
      return nullptr;
    }
    case ShapeType::Box2D: {
      return nullptr;
    }
    case ShapeType::Convex2D: {
      return nullptr;
    }
    case ShapeType::TriangleMesh: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Mesh");
      if (!geometry_set.has_mesh()) {
        return nullptr;
      }
      return bke::TriangleMeshCollisionShape::from_mesh(*geometry_set.get_mesh());
    }
    case ShapeType::ScaledTriangleMesh: {
      return nullptr;
    }
    case ShapeType::StaticPlane: {
      const float3 plane_normal = params.extract_input<float3>("Plane Normal");
      const float plane_constant = params.extract_input<float>("Plane Constant");
      return new bke::StaticPlaneCollisionShape(plane_normal, plane_constant);
    }
    case ShapeType::Compound: {
      const GeometrySet geometry_set = params.extract_input<GeometrySet>("Child Shape");
      const int num_shapes = geometry_set.has_physics() ?
                                 geometry_set.get_physics()->shapes_num() :
                                 0;
      const int num_instances = geometry_set.has_instances() ?
                                    geometry_set.get_instances()->instances_num() :
                                    0;

      Vector<bke::CollisionShapePtr> child_shapes;
      Vector<float4x4> child_transforms;
      child_shapes.reserve(num_shapes + num_instances);
      child_transforms.reserve(num_shapes + num_instances);
      if (geometry_set.has_physics()) {
        for (const bke::CollisionShapePtr &child_shape :
             geometry_set.get_physics()->state().shapes())
        {
          child_shapes.append(child_shape);
          child_transforms.append(float4x4::identity());
        }
      }
      if (geometry_set.has_instances()) {
        const bke::Instances &instances = *geometry_set.get_instances();
        const Span<bke::InstanceReference> references = instances.references();
        const Span<float4x4> transforms = instances.transforms();
        for (const int ref_index : instances.reference_handles()) {
          const GeometrySet &ref_geometry_set = references[ref_index].geometry_set();
          const float4x4 &transform = transforms[ref_index];
          if (ref_geometry_set.has_physics()) {
            for (const bke::CollisionShapePtr &child_shape :
                 ref_geometry_set.get_physics()->state().shapes())
            {
              child_shapes.append(child_shape);
              child_transforms.append(transform);
            }
          }
        }
      }
      return new bke::CompoundCollisionShape(VArray<bke::CollisionShapePtr>::ForSpan(child_shapes),
                                             VArray<float4x4>::ForSpan(child_transforms));
    }
  }
  return nullptr;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const auto shape_type = bke::CollisionShape::ShapeType(params.node().custom1);

  bke::PhysicsGeometry *physics = new bke::PhysicsGeometry(0, 0, 1);
  physics->state_for_write().shapes_for_write().first() = bke::CollisionShapePtr(
      make_collision_shape_from_type(shape_type, params));

  params.set_output("Shape", GeometrySet::from_physics(physics));
}

static void node_rna(StructRNA *srna)
{
  using ShapeType = bke::CollisionShape::ShapeType;

  /* Make sure this matches implemented types in make_collision_shape_from_type. */
  static EnumPropertyItem type_items[] = {
      {int(ShapeType::Invalid), "INVALID", 0, "Invalid", ""},
      {int(ShapeType::Empty), "EMPTY", 0, "Empty", ""},
      {int(ShapeType::Box), "BOX", 0, "Box", ""},
      {int(ShapeType::Triangle), "TRIANGLE", 0, "Triangle", ""},
      // {int(ShapeType::Tetrahedral), "TETRAHEDRAL", 0, "Tetrahedral", ""},
      // {int(ShapeType::ConvexTriangleMesh), "CONVEX_TRIANGLE_MESH", 0, "Convex Triangle Mesh",
      // ""},
      {int(ShapeType::ConvexHull), "CONVEX_HULL", 0, "ConvexHull", ""},
      // {int(ShapeType::ConvexPointCloud), "CONVEX_POINT_CLOUD", 0, "Convex Point Cloud", ""},
      {int(ShapeType::Sphere), "SPHERE", 0, "Sphere", ""},
      // {int(ShapeType::MultiSphere), "MULTI_SPHERE", 0, "Multi Sphere", ""},
      {int(ShapeType::Capsule), "CAPSULE", 0, "Capsule", ""},
      {int(ShapeType::Cone), "CONE", 0, "Cone", ""},
      {int(ShapeType::Cylinder), "CYLINDER", 0, "Cylinder", ""},
      {int(ShapeType::UniformScaling), "UNIFORM_SCALING", 0, "Uniform Scaling", ""},
      // {int(ShapeType::MinkowskiSum), "MINKOWSKI_SUM", 0, "Minkowski Sum", ""},
      // {int(ShapeType::MinkowskiDifference), "MINKOWSKI_DIFFERENCE", 0, "Minkowski Difference",
      // ""},
      // {int(ShapeType::Box2D), "BOX_2D", 0, "Box 2D", ""}, {int(ShapeType::Convex2D),
      // "CONVEX_2D", 0, "Convex 2D", ""},
      {int(ShapeType::TriangleMesh), "TRIANGLE_MESH", 0, "Triangle Mesh", ""},
      {int(ShapeType::ScaledTriangleMesh), "SCALED_TRIANGLE_MESH", 0, "Scaled Triangle Mesh", ""},
      {int(ShapeType::StaticPlane), "STATIC_PLANE", 0, "Static Plane", ""},
      {int(ShapeType::Compound), "COMPOUND", 0, "Compound", ""},
      {0, nullptr, 0, nullptr, nullptr},
  };

  RNA_def_node_enum(srna,
                    "type",
                    "Type",
                    "",
                    type_items,
                    NOD_inline_enum_accessors(custom1),
                    int(ShapeType::Box));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_COLLISION_SHAPE, "Collision Shape", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.updatefunc = node_update;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_cc
