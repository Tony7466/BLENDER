/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"

#include "BKE_curve_to_mesh.hh"
#include "BKE_grease_pencil.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "GEO_join_geometries.hh"
#include "GEO_randomize.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curve_to_mesh_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Curve").supported_type(
      {GeometryComponent::Type::Curve, GeometryComponent::Type::GreasePencil});
  b.add_input<decl::Geometry>("Profile Curve")
      .only_realized_data()
      .supported_type(GeometryComponent::Type::Curve);
  b.add_input<decl::Bool>("Fill Caps")
      .description(
          "If the profile spline is cyclic, fill the ends of the generated mesh with N-gons");
  b.add_output<decl::Geometry>("Mesh").propagate_all();
}

static Mesh *geometry_set_curve_to_mesh_doit(
    const bke::CurvesGeometry &curves,
    const GeometrySet &profile_set,
    const bool fill_caps,
    const AnonymousAttributePropagationInfo &propagation_info)
{
  Mesh *mesh;
  if (profile_set.has_curves()) {
    const Curves *profile_curves = profile_set.get_curves();
    mesh = bke::curve_to_mesh_sweep(
        curves, profile_curves->geometry.wrap(), fill_caps, propagation_info);
  }
  else {
    mesh = bke::curve_to_wire_mesh(curves, propagation_info);
  }
  geometry::debug_randomize_mesh_order(mesh);
  return mesh;
}

static void geometry_set_curve_to_mesh(GeometrySet &geometry_set,
                                       const GeometrySet &profile_set,
                                       const bool fill_caps,
                                       const AnonymousAttributePropagationInfo &propagation_info)
{
  bke::GeometryComponentEditData::remember_deformed_positions_if_necessary(geometry_set);
  Vector<GeometrySet> mesh_geometry_sets;

  if (geometry_set.has_curves()) {
    const Curves &curves = *geometry_set.get_curves();
    Mesh *mesh = geometry_set_curve_to_mesh_doit(
        curves.geometry.wrap(), profile_set, fill_caps, propagation_info);
    mesh_geometry_sets.append(GeometrySet::from_mesh(mesh));
  }

  if (geometry_set.has_grease_pencil()) {
    using namespace blender::bke::greasepencil;
    const GreasePencil &grease_pencil = *geometry_set.get_grease_pencil();
    for (const int layer_index : grease_pencil.layers().index_range()) {
      const Drawing *drawing = get_eval_grease_pencil_layer_drawing(grease_pencil, layer_index);
      if (drawing == nullptr) {
        continue;
      }
      const bke::CurvesGeometry &curves = drawing->strokes();
      /* TODO: Need to propagate layer attributes as well. */
      Mesh *mesh = geometry_set_curve_to_mesh_doit(
          curves, profile_set, fill_caps, propagation_info);
      mesh_geometry_sets.append(GeometrySet::from_mesh(mesh));
    }
  }

  BLI_assert(!mesh_geometry_sets.is_empty());
  GeometrySet all_geometry = geometry::join_geometries(mesh_geometry_sets, propagation_info);
  Mesh *mesh = all_geometry.get_component_for_write<MeshComponent>().release();
  geometry_set.remove_geometry_during_modify();
  geometry_set.replace_mesh(mesh);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet curve_set = params.extract_input<GeometrySet>("Curve");
  GeometrySet profile_set = params.extract_input<GeometrySet>("Profile Curve");
  const bool fill_caps = params.extract_input<bool>("Fill Caps");

  curve_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    if (geometry_set.has_curves() || geometry_set.has_grease_pencil()) {
      geometry_set_curve_to_mesh(
          geometry_set, profile_set, fill_caps, params.get_output_propagation_info("Mesh"));
    }
    geometry_set.keep_only_during_modify({GeometryComponent::Type::Mesh});
  });

  params.set_output("Mesh", std::move(curve_set));
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_CURVE_TO_MESH, "Curve to Mesh", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curve_to_mesh_cc
