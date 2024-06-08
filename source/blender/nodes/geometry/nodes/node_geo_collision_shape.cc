/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_collision_shape_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Float>("Size").default_value(1.0f);
  b.add_input<decl::Vector>("Size", "SizeVector").default_value(float3(1.0f));
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

  bNodeSocket *size_socket = next_socket();
  bNodeSocket *size_vector_socket = next_socket();

  bke::nodeSetSocketAvailability(tree, size_socket, ELEM(type, ShapeType::Sphere));
  node_sock_label(size_socket, ELEM(type, ShapeType::Sphere) ? "Radius" : "");
  bke::nodeSetSocketAvailability(tree, size_vector_socket, ELEM(type, ShapeType::Box));
}

static bke::CollisionShape *make_collision_shape(GeoNodeExecParams params)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  const ShapeType type = ShapeType(params.node().custom1);

  switch (type) {
    case ShapeType::Invalid:
      return nullptr;
    case ShapeType::Empty:
      return nullptr;
    case ShapeType::Box: {
      const float3 half_extent = params.extract_input<float3>("SizeVector");
      return new bke::BoxCollisionShape(half_extent);
    }
    case ShapeType::Triangle:
    case ShapeType::Tetrahedral:
    case ShapeType::ConvexTriangleMesh:
    case ShapeType::ConvexHull:
    case ShapeType::ConvexPointCloud:
    case ShapeType::Sphere: {
      const float radius = params.extract_input<float>("Size");
      return new bke::SphereCollisionShape(radius);
    }
    case ShapeType::MultiSphere:
    case ShapeType::Capsule:
    case ShapeType::Cone:
    case ShapeType::Convex:
    case ShapeType::Cylinder:
    case ShapeType::UniformScaling:
    case ShapeType::MinkowskiSum:
    case ShapeType::MinkowskiDifference:
    case ShapeType::Box2D:
    case ShapeType::Convex2D:
    case ShapeType::TriangleMesh:
    case ShapeType::ScaledTriangleMesh:
    case ShapeType::StaticPlane:
    case ShapeType::Compound:
      return nullptr;
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  bke::CollisionShape *shape = make_collision_shape(params);
  bke::PhysicsGeometry *physics = new bke::PhysicsGeometry(0, 0, 1);
  physics->shapes_for_write()[0] = bke::CollisionShape::Ptr(shape);

  params.set_output("Shape", GeometrySet::from_physics(physics));
}

static void node_rna(StructRNA *srna)
{
  using ShapeType = bke::CollisionShape::ShapeType;

  static EnumPropertyItem type_items[] = {
      {int(ShapeType::Invalid), "INVALID", 0, "Invalid", ""},
      {int(ShapeType::Empty), "EMPTY", 0, "Empty", ""},
      {int(ShapeType::Box), "BOX", 0, "Box", ""},
      {int(ShapeType::Triangle), "TRIANGLE", 0, "Triangle", ""},
      {int(ShapeType::Tetrahedral), "TETRAHEDRAL", 0, "Tetrahedral", ""},
      {int(ShapeType::ConvexTriangleMesh), "CONVEX_TRIANGLE_MESH", 0, "Convex Triangle Mesh", ""},
      {int(ShapeType::ConvexHull), "CONVEX_HULL", 0, "ConvexHull", ""},
      {int(ShapeType::ConvexPointCloud), "CONVEX_POINT_CLOUD", 0, "Convex Point Cloud", ""},
      {int(ShapeType::Sphere), "SPHERE", 0, "Sphere", ""},
      {int(ShapeType::MultiSphere), "MULTI_SPHERE", 0, "Multi Sphere", ""},
      {int(ShapeType::Capsule), "CAPSULE", 0, "Capsule", ""},
      {int(ShapeType::Cone), "CONE", 0, "Cone", ""},
      {int(ShapeType::Convex), "CONVEX", 0, "Convex", ""},
      {int(ShapeType::Cylinder), "CYLINDER", 0, "Cylinder", ""},
      {int(ShapeType::UniformScaling), "UNIFORM_SCALING", 0, "Uniform Scaling", ""},
      {int(ShapeType::MinkowskiSum), "MINKOWSKI_SUM", 0, "Minkowski Sum", ""},
      {int(ShapeType::MinkowskiDifference), "MINKOWSKI_DIFFERENCE", 0, "Minkowski Difference", ""},
      {int(ShapeType::Box2D), "BOX_2D", 0, "Box 2D", ""},
      {int(ShapeType::Convex2D), "CONVEX_2D", 0, "Convex 2D", ""},
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
  blender::bke::nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_collision_shape_cc
