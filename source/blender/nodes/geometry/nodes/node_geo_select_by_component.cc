/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.h"

#include "BLI_index_mask.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_select_by_component_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Bool>(N_("Is Mesh")).field_source();
  b.add_output<decl::Bool>(N_("Is Curves")).field_source();
  b.add_output<decl::Bool>(N_("Is Point Cloud")).field_source();
  b.add_output<decl::Bool>(N_("Is Instances")).field_source();
}

class SelectByComponentFieldInput final : public bke::GeometryFieldInput {
 private:
  const GeometryComponentType type_;

 public:
  SelectByComponentFieldInput(const GeometryComponentType type)
      : bke::GeometryFieldInput(CPPType::get<bool>(), "Select by Component"), type_(type)
  {
  }

  GVArray get_varray_for_context(const bke::GeometryFieldContext &context,
                                 const IndexMask &mask) const final
  {
    const GeometryComponentType context_component_type = context.type();
    return VArray<bool>::ForSingle(context_component_type == type_, mask.size());
  }

  uint64_t hash() const override
  {
    return uint64_t(type_);
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const SelectByComponentFieldInput *other_mask =
            dynamic_cast<const SelectByComponentFieldInput *>(&other))
    {
      return other_mask->type_ == this->type_;
    }
    return false;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  if (params.output_is_required("Is Mesh")) {
    params.set_output(
        "Is Mesh",
        Field<bool>{std::make_shared<SelectByComponentFieldInput>(GEO_COMPONENT_TYPE_MESH)});
  }
  if (params.output_is_required("Is Curves")) {
    params.set_output(
        "Is Curves",
        Field<bool>{std::make_shared<SelectByComponentFieldInput>(GEO_COMPONENT_TYPE_CURVE)});
  }
  if (params.output_is_required("Is Point Cloud")) {
    params.set_output("Is Point Cloud",
                      Field<bool>{std::make_shared<SelectByComponentFieldInput>(
                          GEO_COMPONENT_TYPE_POINT_CLOUD)});
  }
  if (params.output_is_required("Is Instances")) {
    params.set_output(
        "Is Instances",
        Field<bool>{std::make_shared<SelectByComponentFieldInput>(GEO_COMPONENT_TYPE_INSTANCES)});
  }
}

}  // namespace blender::nodes::node_geo_select_by_component_cc

void register_node_type_geo_select_by_component()
{
  namespace file_ns = blender::nodes::node_geo_select_by_component_cc;

  static bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_SELECT_BY_COMPONENT, "Select by Component", NODE_CLASS_CONVERTER);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
