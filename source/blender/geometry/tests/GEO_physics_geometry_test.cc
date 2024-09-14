/* SPDX-FileCopyrightText: 2020 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_geometry_set.hh"
#include "BKE_idtype.hh"
#include "BKE_instances.hh"
#include "BKE_jolt_physics.hh"
#include "BKE_lib_id.hh"
#include "BKE_mesh.h"
#include "BKE_mesh.hh"
#include "BKE_physics_geometry.hh"

#include "BLI_array_utils.hh"
#include "BLI_math_rotation.hh"
#include "BLI_memory_utils.hh"
#include "BLI_utildefines.h"

#include "GEO_join_geometries.hh"
#include "GEO_realize_instances.hh"

#include "CLG_log.h"

#include "testing/testing.h"

namespace blender::bke::tests {

class PhysicsGeometryTest : public testing::Test {
 protected:
  PhysicsGeometryTest() {}

  static void SetUpTestSuite()
  {
    /* BKE_id_free() hits a code path that uses CLOG, which crashes if not initialized properly. */
    CLG_init();

    /* To make id_can_have_animdata() and friends work, the `id_types` array needs to be set up. */
    BKE_idtype_init();

    blender::bke::jolt_physics_init();
  }

  static void TearDownTestSuite()
  {
    blender::bke::jolt_physics_exit();

    CLG_exit();
  }

  void test_data(PhysicsGeometry &physics,
                 const bool has_world,
                 const int bodies_num,
                 const int constraints_num,
                 const int shapes_num)
  {
    bke::PhysicsWorldState &state = physics.state_for_write();

    EXPECT_EQ(state.has_world_data(), has_world);
    EXPECT_EQ(state.bodies_num(), bodies_num);
    EXPECT_EQ(state.constraints_num(), constraints_num);
    EXPECT_EQ(state.shapes_num(), shapes_num);

    // Strict shape index validity isn't guaranteed and not really necessary either.
    // const IndexRange shapes_range = physics.shapes_range();
    // VArraySpan<int> body_shapes = physics.body_shapes();
    // for (const int i : body_shapes.index_range()) {
    //  const int shape_index = body_shapes[i];
    //  EXPECT_TRUE(shape_index == -1 || shapes_range.contains(shape_index));
    //}

    EXPECT_TRUE(state.validate_world_data());
  }

  void add_value_attribute(bke::PhysicsGeometry &physics,
                           const bke::AttrDomain domain,
                           const int value)
  {
    bke::MutableAttributeAccessor attributes = physics.attributes_for_write();
    const VArray<int> varray = VArray<int>::ForSingle(value, attributes.domain_size(domain));
    attributes.add<int>("test_value", domain, AttributeInitVArray(varray));
  }

  void test_attribute(const bke::PhysicsGeometry &physics,
                      const bke::AttrDomain domain,
                      const VArray<int> &varray)
  {
    bke::AttributeAccessor attributes = physics.attributes();
    const int size = attributes.domain_size(domain);
    const VArray<int> data = *attributes.lookup<int>("test_value");
    EXPECT_EQ(data.size(), varray.size());
    for (const int i : IndexRange(size)) {
      EXPECT_EQ(data[i], varray[i]);
    }
  }

  void set_positions(bke::PhysicsGeometry &physics, const float3 &position)
  {
    AttributeWriter<float3> positions = physics.body_positions_for_write();
    EXPECT_TRUE(bool(positions.varray));
    for (const int i : positions.varray.index_range()) {
      positions.varray.set(i, position);
    }
    positions.finish();
  }

  void test_positions(const bke::PhysicsGeometry &physics, const VArray<float3> &varray)
  {
    const VArray<float3> positions = physics.body_positions();
    EXPECT_EQ(positions.size(), varray.size());
    for (const int i : positions.index_range()) {
      EXPECT_EQ(positions[i], varray[i]);
    }
  }
};

static Mesh *create_test_quad_mesh()
{
  static const Array<float3> positions = {{0.453125, 0.0703125, 0.9296875},
                                          {0.453125, -0.234375, 0.8515625},
                                          {0.4609375, -0.4296875, 0.5234375},
                                          {0.7265625, -0.3359375, 0.40625},
                                          {0.6328125, -0.28125, 0.453125},
                                          {0.640625, -0.0546875, 0.703125},
                                          {0.796875, -0.125, 0.5625},
                                          {0.796875, 0.1171875, 0.6171875},
                                          {0.640625, 0.1953125, 0.75}};
  static const Array<int2> edges = {{0, 1},
                                    {1, 2},
                                    {3, 4},
                                    {4, 5},
                                    {6, 7},
                                    {7, 8},
                                    {0, 3},
                                    {1, 4},
                                    {2, 5},
                                    {3, 6},
                                    {4, 7},
                                    {5, 8}};
  static const Array<int> face_offsets = {0, 4, 8, 12, 16};
  static const Array<int> corner_verts = {0, 3, 4, 1, 1, 4, 5, 2, 3, 6, 7, 4, 4, 7, 8, 5};
  static const Array<int> corner_edges = {6, 2, 7, 0, 7, 3, 8, 1, 9, 4, 10, 2, 10, 5, 11, 3};

  Mesh *mesh = BKE_mesh_new_nomain(
      positions.size(), edges.size(), face_offsets.size() - 1, corner_verts.size());
  mesh->vert_positions_for_write().copy_from(positions);
  mesh->edges_for_write().copy_from(edges);
  mesh->face_offsets_for_write().copy_from(face_offsets);
  mesh->corner_verts_for_write().copy_from(corner_verts);
  mesh->corner_edges_for_write().copy_from(corner_edges);

  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();

  return mesh;
}

static Mesh *create_test_triangle_mesh()
{
  static const Array<float3> positions = {{0.453125, 0.0703125, 0.9296875},
                                          {0.453125, -0.234375, 0.8515625},
                                          {0.4609375, -0.4296875, 0.5234375},
                                          {0.7265625, -0.3359375, 0.40625}};
  static const Array<int2> edges = {{0, 1}, {1, 2}, {2, 0}, {0, 3}, {2, 3}};
  static const Array<int> face_offsets = {0, 3, 6};
  static const Array<int> corner_verts = {0, 1, 2, 2, 3, 0};
  static const Array<int> corner_edges = {0, 1, 2, 4, 3, 2};

  Mesh *mesh = BKE_mesh_new_nomain(
      positions.size(), edges.size(), face_offsets.size() - 1, corner_verts.size());
  mesh->vert_positions_for_write().copy_from(positions);
  mesh->edges_for_write().copy_from(edges);
  mesh->face_offsets_for_write().copy_from(face_offsets);
  mesh->corner_verts_for_write().copy_from(corner_verts);
  mesh->corner_edges_for_write().copy_from(corner_edges);

  mesh->tag_loose_verts_none();
  mesh->tag_loose_edges_none();
  mesh->tag_overlapping_none();

  return mesh;
}

struct AllShapesData {
  Vector<CollisionShapePtr> shapes;

  const BoxCollisionShape *box_shape = nullptr;
  const SphereCollisionShape *sphere_shape = nullptr;
  const CylinderCollisionShape *cylinder_shape = nullptr;
  const CapsuleCollisionShape *capsule_shape = nullptr;
  const TriangleCollisionShape *triangle_shape = nullptr;
  const ConvexHullCollisionShape *convex_hull_shape = nullptr;
  const MeshCollisionShape *mesh_shape = nullptr;
  const ScaledCollisionShape *scaled_shape = nullptr;
  const MutableCompoundCollisionShape *compound_shape = nullptr;

  Mesh *empty_mesh = nullptr;
  Mesh *quad_mesh = nullptr;
  Mesh *triangle_mesh = nullptr;

  AllShapesData()
  {
    empty_mesh = BKE_mesh_new_nomain(0, 0, 0, 0);
    quad_mesh = create_test_quad_mesh();
    triangle_mesh = create_test_triangle_mesh();
    BLI_SCOPED_DEFER([&]() {
      BKE_id_free(nullptr, empty_mesh);
      BKE_id_free(nullptr, quad_mesh);
      BKE_id_free(nullptr, triangle_mesh);
    });

    Array<float3> points = {
        float3(1, 0.5f, -1), float3(-1, 0, 0), float3(-0.5f, 1, 0), float3(0, -1, -0.5f)};

    box_shape = new BoxCollisionShape(float3(1, 2, 3));
    sphere_shape = new SphereCollisionShape(1.5f);
    cylinder_shape = new CylinderCollisionShape(0.5f, 1.0f);
    capsule_shape = new CapsuleCollisionShape(0.5f, 1.5f);
    triangle_shape = new TriangleCollisionShape(float3(1, 0, 0), float3(0, 1, 0), float3(0, 0, 1));
    convex_hull_shape = new ConvexHullCollisionShape(VArray<float3>::ForSpan(points));
    mesh_shape = new bke::MeshCollisionShape(*triangle_mesh);
    cylinder_shape->add_user();
    scaled_shape = new ScaledCollisionShape(CollisionShapePtr(cylinder_shape), float3(2.5f));
    capsule_shape->add_user();
    mesh_shape->add_user();
    capsule_shape->add_user();
    Array<CollisionShapePtr> compound_child_shapes = {CollisionShapePtr(capsule_shape),
                                                      CollisionShapePtr(mesh_shape),
                                                      CollisionShapePtr(capsule_shape)};
    Array<float4x4> compound_child_transforms = {
        float4x4::identity(), float4x4::identity(), float4x4::identity()};
    compound_shape = new MutableCompoundCollisionShape(compound_child_shapes,
                                                       compound_child_transforms);

    shapes.reserve(20);
    shapes.append_as(box_shape);
    shapes.append_as(sphere_shape);
    shapes.append_as(cylinder_shape);
    shapes.append_as(capsule_shape);
    shapes.append_as(triangle_shape);
    shapes.append_as(convex_hull_shape);
    shapes.append_as(mesh_shape);
    shapes.append_as(scaled_shape);
    shapes.append_as(compound_shape);
  }
};

TEST_F(PhysicsGeometryTest, construct)
{
  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry();
  test_data(geo1, false, 0, 0, 0);

  bke::PhysicsGeometry geo2 = bke::PhysicsGeometry(1, 0, 0);
  test_data(geo2, false, 1, 0, 0);

  bke::PhysicsGeometry geo3 = bke::PhysicsGeometry(0, 1, 0);
  test_data(geo3, false, 0, 1, 0);

  bke::PhysicsGeometry geo4 = bke::PhysicsGeometry(0, 0, 1);
  test_data(geo4, false, 0, 0, 1);

  bke::PhysicsGeometry geo5 = bke::PhysicsGeometry(1, 1, 1);
  test_data(geo5, false, 1, 1, 1);
}

/* Get a non-default attribute value for testing. */
static const void *attribute_value_by_type(const eCustomDataType type)
{
  switch (type) {
    case CD_PROP_FLOAT: {
      static const float v = 123.0f;
      return &v;
    }
    case CD_PROP_INT32: {
      static const int v = 123;
      return &v;
    }
    case CD_PROP_BOOL: {
      static const bool v = true;
      return &v;
    }
    case CD_PROP_FLOAT3: {
      static const float3 v = float3(11.1f, 22.2f, 33.3f);
      return &v;
    }
    case CD_PROP_QUATERNION: {
      static const math::Quaternion v = math::to_quaternion(math::EulerXYZ(10.0f, 20.0f, 30.0f));
      return &v;
    }
    case CD_PROP_FLOAT4X4: {
      static const float4x4 v = float4x4(
          float4(11.0f), float4(22.0f), float4(33.0f), float4(44.0f));
      return &v;
    }

    default:
      BLI_assert_unreachable();
      return nullptr;
  }
}

static const void *expected_attribute_value(const PhysicsBodyAttribute attribute)
{
  const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
  const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

  switch (attribute) {
    /* Motion type gets clamped to valid enum value. */
    case PhysicsBodyAttribute::motion_type: {
      static const int default_value = int(bke::PhysicsMotionType::Dynamic);
      return &default_value;
    }
    /* Force attributes are only changed by adding forces, writes are ignored. */
    case PhysicsBodyAttribute::total_force:
    case PhysicsBodyAttribute::total_torque: {
      static const float3 zero_vec = float3(0.0f);
      return &zero_vec;
    }

    default:
      return attribute_value_by_type(type);
  }
  BLI_assert_unreachable();
  return nullptr;
}

static const void *expected_attribute_value(const PhysicsConstraintAttribute attribute)
{
  const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
  const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

  switch (attribute) {
    case PhysicsConstraintAttribute::type: {
      static const int expected_type = int(PhysicsConstraintType::Fixed);
      return &expected_type;
    }

    default:
      return attribute_value_by_type(type);
  }
  BLI_assert_unreachable();
  return nullptr;
}

static void test_state_attributes(bke::PhysicsGeometry &geo)
{
  using BodyAttribute = PhysicsGeometry::BodyAttribute;
  using ConstraintAttribute = PhysicsGeometry::ConstraintAttribute;

  {
    bke::MutableAttributeAccessor attributes = geo.attributes_for_write();
    for (const BodyAttribute attribute : PhysicsGeometry::all_body_attributes()) {
      const StringRef name = bke::PhysicsGeometry::attribute_name(attribute);
      const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
      const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

      const void *value = attribute_value_by_type(type);
      bke::GAttributeWriter writer = attributes.lookup_or_add_for_write(
          name, bke::AttrDomain::Point, type);
      writer.varray.fill(value);

      BUFFER_FOR_CPP_TYPE_VALUE(cpptype, buffer);
      writer.varray.get(0, buffer);
      EXPECT_TRUE(cpptype.is_equal(expected_attribute_value(attribute), buffer));

      writer.finish();
    }
    for (const ConstraintAttribute attribute : PhysicsGeometry::all_constraint_attributes()) {
      const StringRef name = bke::PhysicsGeometry::attribute_name(attribute);
      const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
      const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

      const void *value = attribute_value_by_type(type);
      bke::GAttributeWriter writer = attributes.lookup_or_add_for_write(
          name, bke::AttrDomain::Edge, type);
      writer.varray.fill(value);

      BUFFER_FOR_CPP_TYPE_VALUE(cpptype, buffer);
      writer.varray.get(0, buffer);
      EXPECT_TRUE(cpptype.is_equal(expected_attribute_value(attribute), buffer));

      writer.finish();
    }
  }

  geo.state().ensure_read_cache();
  {
    bke::AttributeAccessor attributes = geo.attributes();
    for (const BodyAttribute attribute : PhysicsGeometry::all_body_attributes()) {
      const StringRef name = bke::PhysicsGeometry::attribute_name(attribute);
      const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
      const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

      GVArray varray = *attributes.lookup_or_default(
          name, bke::AttrDomain::Point, type, PhysicsGeometry::attribute_default_value(attribute));

      BUFFER_FOR_CPP_TYPE_VALUE(cpptype, buffer);
      varray.get(0, buffer);
      EXPECT_TRUE(cpptype.is_equal(expected_attribute_value(attribute), buffer));
    }
    for (const ConstraintAttribute attribute : PhysicsGeometry::all_constraint_attributes()) {
      const StringRef name = bke::PhysicsGeometry::attribute_name(attribute);
      const CPPType &cpptype = bke::PhysicsGeometry::attribute_type(attribute);
      const eCustomDataType type = cpp_type_to_custom_data_type(cpptype);

      GVArray varray = *attributes.lookup_or_default(
          name, bke::AttrDomain::Edge, type, PhysicsGeometry::attribute_default_value(attribute));

      BUFFER_FOR_CPP_TYPE_VALUE(cpptype, buffer);
      varray.get(0, buffer);
      EXPECT_TRUE(cpptype.is_equal(expected_attribute_value(attribute), buffer));
    }
  }
}

TEST_F(PhysicsGeometryTest, inactive_state_attributes)
{
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(1, 1, 0);
  test_state_attributes(geo);
  test_data(geo, false, 1, 1, 0);
}

TEST_F(PhysicsGeometryTest, active_state_attributes)
{
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(1, 1, 0);
  geo.state_for_write().create_world();
  test_state_attributes(geo);
  test_data(geo, true, 1, 1, 0);
}

/* Bodies should be deactivated when made static. */
TEST_F(PhysicsGeometryTest, deactivate_when_static)
{
  const static StringRef is_active_id = PhysicsGeometry::attribute_name(
      PhysicsBodyAttribute::is_active);
  const static StringRef motion_type_id = PhysicsGeometry::attribute_name(
      PhysicsBodyAttribute::motion_type);

  /* Test physics with and without world data. */
  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry(1, 0, 0);
  bke::PhysicsGeometry geo2 = bke::PhysicsGeometry(1, 0, 0);
  geo2.state_for_write().create_world();

  {
    const VArray<bool> is_active1 = *geo1.state().attributes().lookup<bool>(
        is_active_id, bke::AttrDomain::Point);
    const VArray<bool> is_active2 = *geo2.state().attributes().lookup<bool>(
        is_active_id, bke::AttrDomain::Point);
    const VArray<int> motion_type1 = *geo1.state().attributes().lookup<int>(
        motion_type_id, bke::AttrDomain::Point);
    const VArray<int> motion_type2 = *geo2.state().attributes().lookup<int>(
        motion_type_id, bke::AttrDomain::Point);
    EXPECT_EQ(PhysicsMotionType::Dynamic, PhysicsMotionType(motion_type1.get(0)));
    EXPECT_EQ(PhysicsMotionType::Dynamic, PhysicsMotionType(motion_type2.get(0)));
    EXPECT_FALSE(is_active1.get(0));
    EXPECT_FALSE(is_active2.get(0));
  }

  {
    bke::AttributeWriter writer1 = geo1.body_motion_types_for_write();
    bke::AttributeWriter writer2 = geo2.body_motion_types_for_write();
    writer1.varray.set(0, int(PhysicsMotionType::Static));
    writer2.varray.set(0, int(PhysicsMotionType::Static));
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(writer1.varray.get(0)));
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(writer2.varray.get(0)));
    writer1.finish();
    writer2.finish();
  }

  {
    const VArray<bool> is_active1 = *geo1.state().attributes().lookup<bool>(
        is_active_id, bke::AttrDomain::Point);
    const VArray<bool> is_active2 = *geo2.state().attributes().lookup<bool>(
        is_active_id, bke::AttrDomain::Point);
    const VArray<int> motion_type1 = *geo1.state().attributes().lookup<int>(
        motion_type_id, bke::AttrDomain::Point);
    const VArray<int> motion_type2 = *geo2.state().attributes().lookup<int>(
        motion_type_id, bke::AttrDomain::Point);
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_type1.get(0)));
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_type2.get(0)));
    EXPECT_FALSE(is_active1.get(0));
    EXPECT_FALSE(is_active2.get(0));
  }
}

TEST_F(PhysicsGeometryTest, custom_data_body_shapes)
{
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(3, 2, 0);
  test_data(geo, false, 3, 2, 0);

  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    body_shapes.varray.set_all({0, 1, -1});
    body_shapes.finish();
    geo.state_for_write().compute_local_inertia(geo.bodies_range());
  }
}

TEST_F(PhysicsGeometryTest, copy_immutable_physics_geometry)
{
  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo = bke::PhysicsGeometry(3, 2, 1);
  geo.state_for_write().create_world();
  add_value_attribute(geo, bke::AttrDomain::Point, 1);
  all_shapes_data.box_shape->add_user();
  geo.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape)});
  geo.state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    body_shapes.varray.set_all({0, 1, -1});
    body_shapes.finish();
  }

  /* Second geometry to make physics data immutable. */
  bke::PhysicsGeometry geo_ref = bke::PhysicsGeometry(geo);

  bke::PhysicsGeometry geo_copy = bke::PhysicsGeometry(geo);
  /* Force copy-on-write. */
  geo_copy.attributes_for_write();

  /* Attribute caches should copy over dirty flags, so that the cache still gets updated correctly
   * after the copy. */
  EXPECT_EQ(1, geo_copy.state().shapes().size());
  EXPECT_EQ(all_shapes_data.box_shape, geo_copy.state().shapes()[0]);
  EXPECT_EQ(3, geo_copy.body_shapes().size());
  EXPECT_EQ_ARRAY(Span<int>{0, 1, -1}.data(),
                  VArraySpan(geo_copy.body_shapes()).data(),
                  geo_copy.body_shapes().size());
  test_data(geo_copy, true, 3, 2, 1);
  test_attribute(geo_copy, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{1, 1, 1}));
}

TEST_F(PhysicsGeometryTest, create_and_remove_world)
{
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(5, 2, 3);
  test_data(geo, false, 5, 2, 3);

  geo.state_for_write().create_world();
  test_data(geo, true, 5, 2, 3);

  geo.state_for_write().destroy_world();
  test_data(geo, false, 5, 2, 3);
}

TEST_F(PhysicsGeometryTest, create_collision_shapes)
{
  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo = bke::PhysicsGeometry(0, 0, all_shapes_data.shapes.size());
  test_data(geo, false, 0, 0, all_shapes_data.shapes.size());
}

TEST_F(PhysicsGeometryTest, assign_collision_shapes)
{
  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo1 = bke::PhysicsGeometry(4, 0, 2);
  test_data(geo1, false, 4, 0, 2);
  {
    const VArray<int> body_shapes = geo1.body_shapes();
    EXPECT_EQ(-1, body_shapes[0]);
    EXPECT_EQ(-1, body_shapes[1]);
    EXPECT_EQ(-1, body_shapes[2]);
    EXPECT_EQ(-1, body_shapes[3]);
  }

  /* Set actual shape pointers. */
  all_shapes_data.box_shape->add_user();
  all_shapes_data.convex_hull_shape->add_user();
  geo1.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.convex_hull_shape)});
  geo1.state_for_write().tag_shapes_changed();
  test_data(geo1, false, 4, 0, 2);
  {
    const VArray<int> body_shapes = geo1.body_shapes();
    EXPECT_EQ(-1, body_shapes[0]);
    EXPECT_EQ(-1, body_shapes[1]);
    EXPECT_EQ(-1, body_shapes[2]);
    EXPECT_EQ(-1, body_shapes[3]);
  }

  /* Assign shapes to bodies. */
  {
    AttributeWriter<int> body_shapes = geo1.body_shapes_for_write();
    body_shapes.varray.set(1, 1);
    body_shapes.varray.set(3, 0);
    body_shapes.finish();
    geo1.state_for_write().compute_local_inertia(geo1.bodies_range());
  }
  test_data(geo1, false, 4, 0, 2);
  {
    const VArray<int> body_shapes = geo1.body_shapes();
    EXPECT_EQ(-1, body_shapes[0]);
    EXPECT_EQ(1, body_shapes[1]);
    EXPECT_EQ(-1, body_shapes[2]);
    EXPECT_EQ(0, body_shapes[3]);
  }

  /* Should remain valid when creating a world. */
  geo1.state_for_write().create_world();
  test_data(geo1, true, 4, 0, 2);
  {
    const VArray<int> body_shapes = geo1.body_shapes();
    EXPECT_EQ(-1, body_shapes[0]);
    EXPECT_EQ(1, body_shapes[1]);
    EXPECT_EQ(-1, body_shapes[2]);
    EXPECT_EQ(0, body_shapes[3]);
  }

  /* Should remain valid when destroying the world. */
  geo1.state_for_write().destroy_world();
  test_data(geo1, false, 4, 0, 2);
  {
    const VArray<int> body_shapes = geo1.body_shapes();
    EXPECT_EQ(-1, body_shapes[0]);
    EXPECT_EQ(1, body_shapes[1]);
    EXPECT_EQ(-1, body_shapes[2]);
    EXPECT_EQ(0, body_shapes[3]);
  }
}

TEST_F(PhysicsGeometryTest, realize_instances)
{
  AllShapesData all_shapes_data;

  const Array<float4x4> CoM_values = Array<float4x4>(
      {math::from_rotation<float4x4>(math::EulerXYZ(0, 30, -10)),
       math::from_scale<float4x4>(float3(1.5f, 1.0f, 2.0f)),
       math::from_location<float4x4>(float3(-3, 1, 1)),
       float4x4::identity(),
       math::from_location<float4x4>(float3(2, 1, 0)),
       float4x4::identity(),
       float4x4::identity()});

  bke::PhysicsGeometry *geo1 = new bke::PhysicsGeometry(5, 2, 3);
  test_data(*geo1, false, 5, 2, 3);
  add_value_attribute(*geo1, bke::AttrDomain::Point, 1);
  /* Set actual shape pointers. */
  all_shapes_data.box_shape->add_user();
  all_shapes_data.box_shape->add_user();
  all_shapes_data.box_shape->add_user();
  geo1->state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.box_shape)});
  geo1->state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo1->body_shapes_for_write();
    body_shapes.varray.set_all({2, 0, 2, -1, 1});
    body_shapes.finish();
    geo1->state_for_write().compute_local_inertia(geo1->bodies_range());
  }

  bke::PhysicsGeometry *geo2 = new bke::PhysicsGeometry(0, 0, 0);
  geo2->state_for_write().create_world();
  test_data(*geo2, true, 0, 0, 0);
  add_value_attribute(*geo2, bke::AttrDomain::Point, 2);

  bke::PhysicsGeometry *geo3 = new bke::PhysicsGeometry(2, 1, 1);
  geo3->state_for_write().create_world();
  test_data(*geo3, true, 2, 1, 1);
  add_value_attribute(*geo3, bke::AttrDomain::Point, 3);
  all_shapes_data.sphere_shape->add_user();
  geo3->state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.sphere_shape)});
  geo3->state_for_write().tag_shapes_changed();
  test_data(*geo3, true, 2, 1, 1);
  /* Invalid shape index should be handled fine. */
  {
    AttributeWriter<int> body_shapes = geo3->body_shapes_for_write();
    body_shapes.varray.set_all({0, 100});
    body_shapes.finish();
    geo3->state_for_write().compute_local_inertia(geo3->bodies_range());
  }
  test_data(*geo3, true, 2, 1, 1);

  std::unique_ptr<bke::Instances> instances = std::make_unique<bke::Instances>();
  instances->resize(3);
  instances->transforms_for_write().fill(float4x4::identity());
  MutableSpan<int> handles = instances->reference_handles_for_write();
  handles[0] = instances->add_reference(bke::InstanceReference{GeometrySet::from_physics(geo1)});
  handles[1] = instances->add_reference(bke::InstanceReference{GeometrySet::from_physics(geo2)});
  handles[2] = instances->add_reference(bke::InstanceReference{GeometrySet::from_physics(geo3)});
  /* Keep this variable so the original geometry is not destroyed and can be tested at the end.
   */
  const GeometrySet instances_geo = GeometrySet::from_instances(instances.release());

  geometry::RealizeInstancesOptions options;
  options.keep_original_ids = true;
  options.realize_instance_attributes = false;
  GeometrySet result = geometry::realize_instances(instances_geo, options);

  EXPECT_TRUE(result.has_physics());
  bke::PhysicsGeometry &geo_result = *result.get_physics_for_write();
  test_data(geo_result, false, 7, 3, 4);
  /* Custom attribute stitched together from different input geometries. */
  test_attribute(
      geo_result, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{1, 1, 1, 1, 1, 3, 3}));
  EXPECT_EQ(geo_result.state().shapes().size(), 4);
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[0].get());
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[1].get());
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[2].get());
  EXPECT_EQ(all_shapes_data.sphere_shape, geo_result.state().shapes()[3].get());
  const VArray<int> result_body_shapes = geo_result.body_shapes();
  EXPECT_EQ(7, result_body_shapes.size());
  EXPECT_EQ(2, result_body_shapes[0]);
  EXPECT_EQ(0, result_body_shapes[1]);
  EXPECT_EQ(2, result_body_shapes[2]);
  EXPECT_EQ(-1, result_body_shapes[3]);
  EXPECT_EQ(1, result_body_shapes[4]);
  /* This shape index shifted to start at 3. */
  EXPECT_EQ(3, result_body_shapes[5]);
  /* Starts as 100, all out-of-bounds indices become -1. */
  EXPECT_EQ(-1, result_body_shapes[6]);

  /* Original geometries should be unmodified. */
  test_data(*geo1, false, 5, 2, 3);
  test_attribute(*geo1, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{1, 1, 1, 1, 1}));
  test_data(*geo2, true, 0, 0, 0);
  test_attribute(*geo2, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{}));
  test_data(*geo3, true, 2, 1, 1);
  test_attribute(*geo3, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{3, 3}));
}

TEST_F(PhysicsGeometryTest, join_geometry)
{
  using ConstraintType = bke::PhysicsGeometry::ConstraintType;

  AllShapesData all_shapes_data;

  const Array<float4x4> frame1 = Array<float4x4>(
      {math::from_rotation<float4x4>(math::EulerXYZ(0, 30, -10)),
       float4x4::identity(),
       math::from_location<float4x4>(float3(-3, 1, 1))});
  const Array<float4x4> frame2 = Array<float4x4>(
      {math::from_location<float4x4>(float3(2, 1, 0)),
       math::from_scale<float4x4>(float3(1.5f, 1.0f, 2.0f)),
       float4x4::identity()});

  bke::PhysicsGeometry *geo1 = new bke::PhysicsGeometry(5, 2, 3);
  add_value_attribute(*geo1, bke::AttrDomain::Point, 1);
  /* Set actual shape pointers. */
  all_shapes_data.box_shape->add_user();
  all_shapes_data.box_shape->add_user();
  all_shapes_data.box_shape->add_user();
  geo1->state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.box_shape)});
  geo1->state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo1->body_shapes_for_write();
    AttributeWriter<float> masses = geo1->body_masses_for_write();
    AttributeWriter<int> constraint_types = geo1->constraint_types_for_write();
    AttributeWriter<int> constraint_body1 = geo1->constraint_body1_for_write();
    AttributeWriter<int> constraint_body2 = geo1->constraint_body2_for_write();
    AttributeWriter<float4x4> constraint_frame1 = geo1->constraint_frame1_for_write();
    AttributeWriter<float4x4> constraint_frame2 = geo1->constraint_frame2_for_write();
    body_shapes.varray.set_all({2, 0, 2, -1, 1});
    masses.varray.set_all({5, 15, 25, 35, 45});
    constraint_types.varray.set_all({int(ConstraintType::Point), int(ConstraintType::Slider)});
    constraint_body1.varray.set_all({2, 0});
    constraint_body2.varray.set_all({1, 2});
    constraint_frame1.varray.set_all(frame1.as_span().slice(0, 2));
    constraint_frame2.varray.set_all(frame2.as_span().slice(0, 2));
    body_shapes.finish();
    masses.finish();
    constraint_types.finish();
    constraint_body1.finish();
    constraint_body2.finish();
    constraint_frame1.finish();
    constraint_frame2.finish();
    geo1->state_for_write().compute_local_inertia(geo1->bodies_range());
  }
  test_data(*geo1, false, 5, 2, 3);

  bke::PhysicsGeometry *geo2 = new bke::PhysicsGeometry(0, 0, 0);
  geo2->state_for_write().create_world();
  add_value_attribute(*geo2, bke::AttrDomain::Point, 2);
  test_data(*geo2, true, 0, 0, 0);

  bke::PhysicsGeometry *geo3 = new bke::PhysicsGeometry(2, 1, 1);
  geo3->state_for_write().create_world();
  add_value_attribute(*geo3, bke::AttrDomain::Point, 3);
  all_shapes_data.sphere_shape->add_user();
  geo3->state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.sphere_shape)});
  geo3->state_for_write().tag_shapes_changed();
  /* Invalid shape index should be handled fine. */
  {
    AttributeWriter<int> body_shapes = geo3->body_shapes_for_write();
    AttributeWriter<float> masses = geo3->body_masses_for_write();
    AttributeWriter<int> constraint_types = geo3->constraint_types_for_write();
    AttributeWriter<int> constraint_body1 = geo3->constraint_body1_for_write();
    AttributeWriter<int> constraint_body2 = geo3->constraint_body2_for_write();
    AttributeWriter<float4x4> constraint_frame1 = geo3->constraint_frame1_for_write();
    AttributeWriter<float4x4> constraint_frame2 = geo3->constraint_frame2_for_write();
    body_shapes.varray.set_all({0, 100});
    masses.varray.set_all({3, 33});
    constraint_types.varray.set_all({int(ConstraintType::Fixed)});
    constraint_body1.varray.set_all({-1});
    constraint_body2.varray.set_all({1});
    constraint_frame1.varray.set_all(frame1.as_span().slice(2, 1));
    constraint_frame2.varray.set_all(frame2.as_span().slice(2, 1));
    body_shapes.finish();
    masses.finish();
    constraint_types.finish();
    constraint_body1.finish();
    constraint_body2.finish();
    constraint_frame1.finish();
    constraint_frame2.finish();
    geo3->state_for_write().compute_local_inertia(geo3->bodies_range());
  }
  test_data(*geo3, true, 2, 1, 1);

  Array<bke::GeometrySet> geometry_sets = {bke::GeometrySet::from_physics(geo1),
                                           bke::GeometrySet::from_physics(geo2),
                                           bke::GeometrySet::from_physics(geo3)};
  GeometrySet result = geometry::join_geometries(geometry_sets, {});

  EXPECT_TRUE(result.has_physics());
  bke::PhysicsGeometry &geo_result = *result.get_physics_for_write();
  test_data(geo_result, true, 7, 3, 4);
  /* Custom attribute stitched together from different input geometries. */
  test_attribute(
      geo_result, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{1, 1, 1, 1, 1, 3, 3}));
  EXPECT_EQ(geo_result.state().shapes().size(), 4);
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[0].get());
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[1].get());
  EXPECT_EQ(all_shapes_data.box_shape, geo_result.state().shapes()[2].get());
  EXPECT_EQ(all_shapes_data.sphere_shape, geo_result.state().shapes()[3].get());
  const VArray<int> result_body_shapes = geo_result.body_shapes();
  EXPECT_EQ(7, result_body_shapes.size());
  EXPECT_EQ(2, result_body_shapes[0]);
  EXPECT_EQ(0, result_body_shapes[1]);
  EXPECT_EQ(2, result_body_shapes[2]);
  EXPECT_EQ(-1, result_body_shapes[3]);
  EXPECT_EQ(1, result_body_shapes[4]);
  /* This shape index shifted to start at 3. */
  EXPECT_EQ(3, result_body_shapes[5]);
  /* Starts as 100, all out-of-bounds indices become -1. */
  EXPECT_EQ(-1, result_body_shapes[6]);
  const VArraySpan<float> masses = geo_result.body_masses();
  EXPECT_EQ_ARRAY(Span<float>{5, 15, 25, 35, 45, 3, 33}.data(), masses.data(), masses.size());
  const VArraySpan<int> constraint_types = geo_result.constraint_types();
  const VArraySpan<int> constraint_body1 = geo_result.constraint_body1();
  const VArraySpan<int> constraint_body2 = geo_result.constraint_body2();
  const VArraySpan<float4x4> constraint_frame1 = geo_result.constraint_frame1();
  const VArraySpan<float4x4> constraint_frame2 = geo_result.constraint_frame2();
  EXPECT_EQ_ARRAY(Span<int>{int(ConstraintType::Point),
                            int(ConstraintType::Slider),
                            int(ConstraintType::Fixed)}
                      .data(),
                  constraint_types.data(),
                  constraint_types.size());
  EXPECT_EQ_ARRAY(Span<int>{2, 0, -1}.data(), constraint_body1.data(), constraint_body1.size());
  EXPECT_EQ_ARRAY(Span<int>{1, 2, 6}.data(), constraint_body2.data(), constraint_body2.size());
  EXPECT_EQ_ARRAY(frame1.data(), constraint_frame1.data(), constraint_frame1.size());
  EXPECT_EQ_ARRAY(frame2.data(), constraint_frame2.data(), constraint_frame2.size());

  /* Original geometries should be unmodified. */
  test_data(*geo1, false, 5, 2, 3);
  test_attribute(*geo1, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{1, 1, 1, 1, 1}));
  /* World data for geo2 has been moved and should no longer exist here. */
  test_data(*geo2, false, 0, 0, 0);
  test_attribute(*geo2, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{}));
  test_data(*geo3, true, 2, 1, 1);
  test_attribute(*geo3, bke::AttrDomain::Point, VArray<int>::ForSpan(Array<int>{3, 3}));
}

TEST_F(PhysicsGeometryTest, body_shapes_update)
{
  AllShapesData all_shapes_data;

  /* Change the body shape indices without(!) updating the internal pointers straight away. This
   * should return the same values when reading from the cache. */
  bke::PhysicsGeometry geo = bke::PhysicsGeometry(8, 0, 1);
  geo.state_for_write().create_world();
  /* Set actual shape pointers. */
  all_shapes_data.box_shape->add_user();
  geo.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape)});
  geo.state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    body_shapes.varray.set_all({0, 0, 0, 0, 0, 0, 0, 0});
    body_shapes.finish();
  }

  /* Trigger a cache update, this should not change shape indices. */
  geo.body_positions_for_write().tag_modified_fn();
  geo.state().ensure_read_cache();

  EXPECT_EQ(geo.state().shapes().size(), 1);
  EXPECT_EQ(all_shapes_data.box_shape, geo.state().shapes()[0].get());
  const VArray<int> result_body_shapes = geo.body_shapes();
  EXPECT_EQ(8, result_body_shapes.size());
  EXPECT_EQ(0, result_body_shapes[0]);
  EXPECT_EQ(0, result_body_shapes[1]);
  EXPECT_EQ(0, result_body_shapes[2]);
  EXPECT_EQ(0, result_body_shapes[3]);
  EXPECT_EQ(0, result_body_shapes[4]);
  EXPECT_EQ(0, result_body_shapes[5]);
  EXPECT_EQ(0, result_body_shapes[6]);
  EXPECT_EQ(0, result_body_shapes[7]);
  /* Note : test_data also invokes shape pointer update, only call after testing indices! */
  test_data(geo, true, 8, 0, 1);
}

TEST_F(PhysicsGeometryTest, update_read_cache)
{
  bke::PhysicsGeometry *geo1 = new bke::PhysicsGeometry(0, 0, 0);
  geo1->state_for_write().create_world();
  test_data(*geo1, true, 0, 0, 0);

  bke::PhysicsGeometry *geo2 = new bke::PhysicsGeometry(3, 2, 0);
  test_data(*geo2, false, 3, 2, 0);
  {
    AttributeWriter<float3> positions = geo2->body_positions_for_write();
    positions.varray.set_all({float3(123), float3(456), float3(789)});
    positions.finish();
    AttributeWriter<int> motion_type = geo2->attributes_for_write().lookup_or_add_for_write<int>(
        "motion_type", AttrDomain::Point);
    motion_type.varray.set_all({int(PhysicsMotionType::Static),
                                int(PhysicsMotionType::Dynamic),
                                int(PhysicsMotionType::Static)});
    motion_type.finish();
  }
  {
    const VArray<float3> positions = geo2->body_positions();
    EXPECT_EQ(3, positions.size());
    EXPECT_EQ(float3(123), positions[0]);
    EXPECT_EQ(float3(456), positions[1]);
    EXPECT_EQ(float3(789), positions[2]);
    const VArray<int> motion_types = geo2->body_motion_types();
    EXPECT_EQ(3, motion_types.size());
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_types[0]));
    EXPECT_EQ(PhysicsMotionType::Dynamic, PhysicsMotionType(motion_types[1]));
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_types[2]));
  }

  Array<bke::GeometrySet> geometry_sets = {bke::GeometrySet::from_physics(geo1),
                                           bke::GeometrySet::from_physics(geo2)};
  GeometrySet result = geometry::join_geometries(geometry_sets, {});

  /*const PhysicsGeometry *geo_result =*/result.get_physics();
  {
    const VArray<float3> positions = geo2->body_positions();
    EXPECT_EQ(3, positions.size());
    EXPECT_EQ(float3(123), positions[0]);
    EXPECT_EQ(float3(456), positions[1]);
    EXPECT_EQ(float3(789), positions[2]);
    const VArray<int> motion_types = geo2->body_motion_types();
    EXPECT_EQ(3, motion_types.size());
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_types[0]));
    EXPECT_EQ(PhysicsMotionType::Dynamic, PhysicsMotionType(motion_types[1]));
    EXPECT_EQ(PhysicsMotionType::Static, PhysicsMotionType(motion_types[2]));
  }
}

/* Some attributes are connected internally. Changing mass, motion type (static/kinematic/dynamic)
 * or the shape is expected to conditionally change the other attributes on the Bullet side.
 * Attributes remain untouched, these read/write from custom data. */
TEST_F(PhysicsGeometryTest, motion_type_attribute_dependencies)
{
  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo = bke::PhysicsGeometry(9, 0, 3);
  geo.state_for_write().create_world();
  test_data(geo, true, 9, 0, 3);

  all_shapes_data.sphere_shape->add_user();
  all_shapes_data.box_shape->add_user();
  all_shapes_data.cylinder_shape->add_user();
  geo.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.cylinder_shape),
       CollisionShapePtr(all_shapes_data.box_shape),
       CollisionShapePtr(all_shapes_data.sphere_shape)});
  geo.state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    AttributeWriter<int> motion_types = geo.body_motion_types_for_write();
    AttributeWriter<float> masses = geo.body_masses_for_write();
    body_shapes.varray.set_all({0, 0, 0, 1, 1, 1, 2, 2, 2});
    motion_types.varray.set_all({int(PhysicsMotionType::Static),
                                 int(PhysicsMotionType::Static),
                                 int(PhysicsMotionType::Static),
                                 int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Dynamic)});
    masses.varray.set_all({0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f});
    body_shapes.finish();
    motion_types.finish();
    masses.finish();
  }
  {
    const VArraySpan<int> body_shapes = geo.body_shapes();
    const VArraySpan<int> motion_types = geo.body_motion_types();
    const VArraySpan<float> masses = geo.body_masses();
    EXPECT_EQ_ARRAY(Span<int>{0, 0, 0, 1, 1, 1, 2, 2, 2}.data(), body_shapes.data(), 9);
    EXPECT_EQ_ARRAY(Span<int>{int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic)}
                        .data(),
                    motion_types.data(),
                    9);
    EXPECT_EQ_ARRAY(Span<float>{0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f}.data(),
                    masses.data(),
                    9);
  }

  /* Change shape:
   * Switching a dynamic/kinematic body to a non-moveable shape makes it static.
   * Switching shape of a static body keeps it static. */
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    body_shapes.varray.set_all({1, 0, 0, 0, 1, 1, 0, 2, 2});
    body_shapes.finish();
  }
  {
    const VArraySpan<int> body_shapes = geo.body_shapes();
    const VArraySpan<int> motion_types = geo.body_motion_types();
    const VArraySpan<float> masses = geo.body_masses();
    EXPECT_EQ_ARRAY(Span<int>{1, 0, 0, 0, 1, 1, 0, 2, 2}.data(), body_shapes.data(), 9);
    EXPECT_EQ_ARRAY(Span<int>{int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Dynamic)}
                        .data(),
                    motion_types.data(),
                    9);
    EXPECT_EQ_ARRAY(Span<float>{0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f}.data(),
                    masses.data(),
                    9);
  }

  /* Change 'static' flag:
   * Only change to dynamic if body has a moveable shape.
   * Making a body static enforces zero mass.
   * Making a body dynamic enforces non-zero mass (1.0 by default).
   */
  {
    AttributeWriter<int> body_motion_types = geo.body_motion_types_for_write();
    body_motion_types.varray.set_all({int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Dynamic),
                                      int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Dynamic),
                                      int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Static),
                                      int(PhysicsMotionType::Dynamic)});
    body_motion_types.finish();
  }
  {
    const VArraySpan<int> body_shapes = geo.body_shapes();
    const VArraySpan<int> motion_types = geo.body_motion_types();
    const VArraySpan<float> masses = geo.body_masses();
    EXPECT_EQ_ARRAY(Span<int>{1, 0, 0, 0, 1, 1, 0, 2, 2}.data(), body_shapes.data(), 9);
    EXPECT_EQ_ARRAY(Span<int>{int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic)}
                        .data(),
                    motion_types.data(),
                    9);
    EXPECT_EQ_ARRAY(Span<float>{0.0f, 0.0f, 0.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f}.data(),
                    masses.data(),
                    9);
  }

  /* Change mass:
   * Remains zero if body is static or the shape is non-moving.
   */
  {
    AttributeWriter<float> body_masses = geo.body_masses_for_write();
    body_masses.varray.set_all({0.0f, 0.0f, 10.0f, 5.0f, 5.0f, 10.0f, 5.0f, 5.0f, 10.0f});
    body_masses.finish();
  }
  {
    const VArraySpan<int> body_shapes = geo.body_shapes();
    const VArraySpan<int> motion_types = geo.body_motion_types();
    const VArraySpan<float> masses = geo.body_masses();
    EXPECT_EQ_ARRAY(Span<int>{1, 0, 0, 0, 1, 1, 0, 2, 2}.data(), body_shapes.data(), 9);
    EXPECT_EQ_ARRAY(Span<int>{int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Static),
                              int(PhysicsMotionType::Dynamic)}
                        .data(),
                    motion_types.data(),
                    9);
    EXPECT_EQ_ARRAY(Span<float>{0.0f, 0.0f, 10.0f, 5.0f, 5.0f, 10.0f, 5.0f, 5.0f, 10.0f}.data(),
                    masses.data(),
                    9);
  }
}

TEST_F(PhysicsGeometryTest, change_constraint_types)
{
  using ConstraintType = PhysicsGeometry::ConstraintType;

  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo = bke::PhysicsGeometry(3, 2, 1);
  geo.state_for_write().create_world();
  all_shapes_data.box_shape->add_user();
  geo.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape)});
  geo.state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    AttributeWriter<int> motion_types = geo.body_motion_types_for_write();
    AttributeWriter<float> masses = geo.body_masses_for_write();
    AttributeWriter<int> constraint_types = geo.constraint_types_for_write();
    AttributeWriter<int> constraint_body1 = geo.constraint_body1_for_write();
    AttributeWriter<int> constraint_body2 = geo.constraint_body2_for_write();
    body_shapes.varray.set_all({0, 0, 0});
    motion_types.varray.set_all({int(PhysicsMotionType::Dynamic),
                                 int(PhysicsMotionType::Static),
                                 int(PhysicsMotionType::Dynamic)});
    masses.varray.set_all({1.0f, 1.0f, 1.0f});
    constraint_types.varray.set_all({int(ConstraintType::Fixed), int(ConstraintType::Hinge)});
    constraint_body1.varray.set_all({1, 1});
    constraint_body2.varray.set_all({0, 2});
    body_shapes.finish();
    motion_types.finish();
    masses.finish();
    constraint_types.finish();
    constraint_body1.finish();
    constraint_body2.finish();
  }
  test_data(geo, true, 3, 2, 1);

  {
    AttributeWriter<int> constraint_types = geo.constraint_types_for_write();
    constraint_types.varray.set_all({int(ConstraintType::Point), int(ConstraintType::Hinge)});
    constraint_types.finish();
  }
  {
    const VArraySpan<int> constraint_types = geo.constraint_types();
    EXPECT_EQ_ARRAY(Span<int>{int(ConstraintType::Point), int(ConstraintType::Hinge)}.data(),
                    constraint_types.data(),
                    constraint_types.size());
  }
  test_data(geo, true, 3, 2, 1);
}

TEST_F(PhysicsGeometryTest, simple_time_step)
{
  AllShapesData all_shapes_data;

  bke::PhysicsGeometry geo = bke::PhysicsGeometry(1, 0, 1);
  geo.state_for_write().create_world();
  test_data(geo, true, 1, 0, 1);

  all_shapes_data.box_shape->add_user();
  geo.state_for_write().shapes_for_write().copy_from(
      {CollisionShapePtr(all_shapes_data.box_shape)});
  geo.state_for_write().tag_shapes_changed();
  {
    AttributeWriter<int> body_shapes = geo.body_shapes_for_write();
    AttributeWriter<int> motion_types = geo.body_motion_types_for_write();
    AttributeWriter<float> masses = geo.body_masses_for_write();
    body_shapes.varray.set(0, 0);
    motion_types.varray.set(0, int(PhysicsMotionType::Dynamic));
    masses.varray.set(0, 1.0f);
    body_shapes.finish();
    motion_types.finish();
    masses.finish();
  }

  {
    geo.state().ensure_read_cache();
    const VArray<float3> positions = geo.body_positions();
    const VArray<float3> velocities = geo.body_velocities();
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), positions[0]);
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), velocities[0]);
  }

  constexpr float g = -9.81f;
  geo.state_for_write().set_gravity(float3(0, 0, g));

  constexpr float delta_time = 0.1f;
  geo.state_for_write().step_simulation(delta_time);

  {
    geo.state().ensure_read_cache();
    const VArray<float3> positions = geo.body_positions();
    const VArray<float3> velocities = geo.body_velocities();
    /* Not enough accuracy to meaningfully test for equality. */
    // const float dv = g * delta_time;
    // const float dx = 0.5f * g * delta_time * delta_time;
    // EXPECT_V3_NEAR(float3(0.0f, 0.0f, dx), positions[0], 1e-2f);
    // EXPECT_V3_NEAR(float3(0.0f, 0.0f, dv), velocities[0], 1e-2f);
    EXPECT_LT(positions[0].z, 0.0f);
    EXPECT_LT(velocities[0].z, 0.0f);
  }
}

}  // namespace blender::bke::tests
