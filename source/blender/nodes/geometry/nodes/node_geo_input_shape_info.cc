/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_attribute.hh"
#include "BKE_collision_shape.hh"
#include "BKE_physics_geometry.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_shape_info_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  using ShapeType = bke::CollisionShape::ShapeType;
  using ShapeAttribute = bke::PhysicsShapeAttribute;
  const bNode *node = b.node_or_null();
  if (node == nullptr) {
    return;
  }

  const ShapeType shape_type = ShapeType(node->custom1);

  auto add_param_output = [&](const ShapeAttribute attribute) {
    if (bke::physics_attributes::physics_shape_attribute_valid(shape_type, attribute)) {
      const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(
          bke::physics_attributes::physics_attribute_type(attribute));
      const StringRef identifier = bke::physics_attributes::physics_attribute_name(attribute);
      const StringRef name = bke::physics_attributes::physics_shape_attribute_label(shape_type,
                                                                                    attribute);
      b.add_output(data_type, name, identifier);
    }
  };

  b.add_output<decl::Bool>("Selection")
      .field_source()
      .description("Selection of shapes that match the chosen type");

  add_param_output(ShapeAttribute::translation);
  add_param_output(ShapeAttribute::rotation);
  add_param_output(ShapeAttribute::scale);
  add_param_output(ShapeAttribute::size);
  add_param_output(ShapeAttribute::radius);
  add_param_output(ShapeAttribute::radius2);
  add_param_output(ShapeAttribute::height);
  add_param_output(ShapeAttribute::point0);
  add_param_output(ShapeAttribute::point1);
  add_param_output(ShapeAttribute::point2);
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

class ShapeInfoFieldInput : public bke::PhysicsFieldInput {
 public:
  using ShapeType = bke::CollisionShape::ShapeType;
  using ShapeAttribute = bke::PhysicsShapeAttribute;

  ShapeType shape_type_;
  ShapeAttribute attribute_;

  ShapeInfoFieldInput(const ShapeType shape_type, const ShapeAttribute attribute)
      : bke::PhysicsFieldInput(bke::physics_attributes::physics_attribute_type(attribute),
                               "ShapeInfoFieldInput:" +
                                   bke::physics_attributes::physics_attribute_name(attribute)),
        shape_type_(shape_type),
        attribute_(attribute)
  {
  }

  static fn::GField Create(const ShapeType shape_type, const ShapeAttribute attribute)
  {
    return fn::GField(std::make_shared<ShapeInfoFieldInput>(shape_type, attribute));
  }
  template<typename T>
  static fn::Field<T> Create(const ShapeType shape_type, const ShapeAttribute attribute)
  {
    return fn::Field<T>(std::make_shared<ShapeInfoFieldInput>(shape_type, attribute));
  }

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 AttrDomain domain,
                                 const IndexMask & /*mask*/) const override
  {
    if (domain == AttrDomain::Instance) {
      return bke::physics_attributes::physics_attribute_varray(attribute_,
                                                               physics.state().shapes());
    }
    return {};
  }
};

class ShapeMatchFieldInput : public bke::PhysicsFieldInput {
 public:
  using ShapeType = bke::CollisionShape::ShapeType;

  ShapeType shape_type_;

  ShapeMatchFieldInput(const ShapeType shape_type)
      : bke::PhysicsFieldInput(CPPType::get<bool>(), "ShapeMatchFieldInput"),
        shape_type_(shape_type)
  {
  }

  static fn::Field<bool> Create(const ShapeType shape_type)
  {
    return fn::Field<bool>(std::make_shared<ShapeMatchFieldInput>(shape_type));
  }

  GVArray get_varray_for_context(const bke::PhysicsGeometry &physics,
                                 AttrDomain domain,
                                 const IndexMask &mask) const override
  {
    if (domain == AttrDomain::Instance) {
      const Span<bke::CollisionShapePtr> shapes = physics.state().shapes();
      Array<bool> matches(physics.state().shapes().size());
      mask.foreach_index(GrainSize(1024), [&](const int index) {
        matches[index] = (shapes[index] ? shapes[index]->type() == shape_type_ : false);
      });
      return VArray<bool>::ForContainer(matches);
    }
    return {};
  }
};

template<typename T>
static void set_attribute_field(GeoNodeExecParams params,
                                const bke::CollisionShape::ShapeType shape_type,
                                const bke::PhysicsShapeAttribute attribute)
{
  if (bke::physics_attributes::physics_shape_attribute_valid(shape_type, attribute)) {
    params.set_output(bke::physics_attributes::physics_attribute_name(attribute),
                      Field<T>(ShapeInfoFieldInput::Create(shape_type, attribute)));
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  using ShapeAttribute = bke::PhysicsShapeAttribute;
  using ShapeType = bke::CollisionShape::ShapeType;
  const ShapeType shape_type = ShapeType(params.node().custom1);

  params.set_output("Selection", ShapeMatchFieldInput::Create(shape_type));

  set_attribute_field<float3>(params, shape_type, ShapeAttribute::translation);
  set_attribute_field<math::Quaternion>(params, shape_type, ShapeAttribute::rotation);
  set_attribute_field<float3>(params, shape_type, ShapeAttribute::scale);
  set_attribute_field<float3>(params, shape_type, ShapeAttribute::size);
  set_attribute_field<float>(params, shape_type, ShapeAttribute::radius);
  set_attribute_field<float>(params, shape_type, ShapeAttribute::radius2);
  set_attribute_field<float>(params, shape_type, ShapeAttribute::height);
  set_attribute_field<float3>(params, shape_type, ShapeAttribute::point0);
  set_attribute_field<float3>(params, shape_type, ShapeAttribute::point1);
  set_attribute_field<float3>(params, shape_type, ShapeAttribute::point2);
}

static void node_rna(StructRNA *srna)
{
  using ShapeType = bke::CollisionShape::ShapeType;

  /* Make sure this matches implemented types in make_collision_shape_from_type. */
  static EnumPropertyItem type_items[] = {
      {int(ShapeType::Sphere), "SPHERE", 0, "Sphere", ""},
      {int(ShapeType::Box), "BOX", 0, "Box", ""},
      {int(ShapeType::Triangle), "TRIANGLE", 0, "Triangle", ""},
      {int(ShapeType::Capsule), "CAPSULE", 0, "Capsule", ""},
      {int(ShapeType::TaperedCapsule), "TAPERED_CAPSULE", 0, "TaperedCapsule", ""},
      {int(ShapeType::Cylinder), "CYLINDER", 0, "Cylinder", ""},
      {int(ShapeType::ConvexHull), "CONVEX_HULL", 0, "ConvexHull", ""},
      {int(ShapeType::StaticCompound), "STATIC_COMPOUND", 0, "StaticCompound", ""},
      {int(ShapeType::MutableCompound), "MUTABLE_COMPOUND", 0, "MutableCompound", ""},
      {int(ShapeType::RotatedTranslated), "ROTATED_TRANSLATED", 0, "RotatedTranslated", ""},
      {int(ShapeType::Scaled), "SCALED", 0, "Scaled", ""},
      {int(ShapeType::OffsetCenterOfMass), "OFFSET_CENTER_OF_MASS", 0, "OffsetCenterOfMass", ""},
      {int(ShapeType::Mesh), "MESH", 0, "Mesh", ""},
      {int(ShapeType::HeightField), "HEIGHT_FIELD", 0, "HeightField", ""},
      {int(ShapeType::SoftBody), "SOFT_BODY", 0, "SoftBody", ""},
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

  geo_node_type_base(&ntype, GEO_NODE_INPUT_SHAPE_INFO, "Shape Info", NODE_CLASS_INPUT);
  ntype.declare = node_declare;
  ntype.initfunc = node_init;
  ntype.draw_buttons = node_layout;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_input_shape_info_cc
