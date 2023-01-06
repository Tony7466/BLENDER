/* SPDX-License-Identifier: GPL-2.0-or-later */
#include "GEO_join_geometry.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_join_geometry_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry")).multi_input();
  b.add_output<decl::Geometry>(N_("Geometry")).propagate_all();
}

template<typename Component>
Vector<const Component *> get_components_from_geometry_sets(
    const Span<GeometrySet> src_geometry_sets)
{
  Vector<const Component *> components;
  for (const GeometrySet &geometry_set : src_geometry_sets) {
    const Component *component = geometry_set.get_component_for_read<Component>();
    if (component != nullptr && !component->is_empty()) {
      components.append(component);
    }
  }
  return components;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  Vector<GeometrySet> geometry_sets = params.extract_input<Vector<GeometrySet>>("Geometry");

  const AnonymousAttributePropagationInfo &propagation_info = params.get_output_propagation_info(
      "Geometry");

  GeometrySet geometry_set_result;
  geometry::GEO_join_component_type<MeshComponent>(
      get_components_from_geometry_sets<MeshComponent>(geometry_sets),
      geometry_set_result,
      propagation_info);
  geometry::GEO_join_component_type<PointCloudComponent>(
      get_components_from_geometry_sets<PointCloudComponent>(geometry_sets),
      geometry_set_result,
      propagation_info);
  geometry::GEO_join_component_type<InstancesComponent>(
      get_components_from_geometry_sets<InstancesComponent>(geometry_sets),
      geometry_set_result,
      propagation_info);
  geometry::GEO_join_component_type<VolumeComponent>(
      get_components_from_geometry_sets<VolumeComponent>(geometry_sets),
      geometry_set_result,
      propagation_info);
  geometry::GEO_join_component_type<CurveComponent>(
      get_components_from_geometry_sets<CurveComponent>(geometry_sets),
      geometry_set_result,
      propagation_info);
  geometry::GEO_join_component_type<GeometryComponentEditData>(
      get_components_from_geometry_sets<GeometryComponentEditData>(geometry_sets),
      geometry_set_result,
      propagation_info);

  params.set_output("Geometry", std::move(geometry_set_result));
}
}  // namespace blender::nodes::node_geo_join_geometry_cc

void register_node_type_geo_join_geometry()
{
  namespace file_ns = blender::nodes::node_geo_join_geometry_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_JOIN_GEOMETRY, "Join Geometry", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.declare = file_ns::node_declare;
  nodeRegisterType(&ntype);
}
