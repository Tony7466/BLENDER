/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "GEO_mesh_primitive_cuboid.hh"

#include "BLI_bounds.hh"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_bounding_box_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_output<decl::Geometry>("Bounding Box");
  b.add_output<decl::Vector>("Min");
  b.add_output<decl::Vector>("Max");
}

static Mesh *create_mesh_from_boundbox(const Bounds<float3> &bounds)
{
  const float3 scale = bounds.max - bounds.min;
  const float3 center = bounds.min + scale / 2.0f;
  Mesh *mesh = geometry::create_cuboid_mesh(scale, 2, 2, 2, "uv_map");
  transform_mesh(*mesh, center, float3(0), float3(1));
  return mesh;
}

static bool boundbox_from_grease_pencil(GeometrySet &geometry_set)
{
  using namespace blender::bke::greasepencil;

  const GreasePencil &grease_pencil = *geometry_set.get_grease_pencil();
  Array<Mesh *> mesh_by_layer(grease_pencil.layers().size(), nullptr);

  for (const int layer_index : grease_pencil.layers().index_range()) {
    const Drawing *drawing = get_eval_grease_pencil_layer_drawing(grease_pencil, layer_index);
    if (drawing == nullptr) {
      continue;
    }
    const bke::CurvesGeometry &curves = drawing->strokes();
    if (curves.curves_num() == 0) {
      continue;
    }
    const std::optional<Bounds<float3>> bounds = curves.bounds_min_max();
    if (!bounds) {
      continue;
    }
    mesh_by_layer[layer_index] = create_mesh_from_boundbox(bounds.value());
  }

  if (mesh_by_layer.is_empty()) {
    return false;
  }

  InstancesComponent &instances_component =
      geometry_set.get_component_for_write<InstancesComponent>();
  bke::Instances *instances = instances_component.get_for_write();
  if (instances == nullptr) {
    instances = new bke::Instances();
    instances_component.replace(instances);
  }
  for (Mesh *mesh : mesh_by_layer) {
    if (!mesh) {
      /* Add an empty reference so the number of layers and instances match.
       * This makes it easy to reconstruct the layers afterwards and keep their attributes.
       * Although in this particular case we don't propagate the attributes. */
      const int handle = instances->add_reference(bke::InstanceReference());
      instances->add_instance(handle, float4x4::identity());
      continue;
    }
    GeometrySet temp_set = GeometrySet::from_mesh(mesh);
    const int handle = instances->add_reference(bke::InstanceReference{temp_set});
    instances->add_instance(handle, float4x4::identity());
  }
  geometry_set.replace_grease_pencil(nullptr);
  return true;
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");

  /* Compute the min and max of all realized geometry for the two
   * vector outputs, which are only meant to consider real geometry.
   *
   * Grease Pencil are a bit of an exception where we want them to create
   * separate bounding box for each layer. But to include their calculation
   * in the Min/Max.
   *
   * The reason we don't do this for the Instances too is that it can get expensive. */
  const std::optional<Bounds<float3>> bounds = geometry_set.compute_boundbox_without_instances();
  std::optional<Bounds<float3>> bounds_with_grease_pencil = bounds;
  if (const GreasePencil *grease_pencil = geometry_set.get_grease_pencil()) {
    bounds_with_grease_pencil = bounds::merge(bounds_with_grease_pencil,
                                              grease_pencil->bounds_min_max());
  }
  if (!bounds_with_grease_pencil) {
    params.set_output("Min", float3(0));
    params.set_output("Max", float3(0));
  }
  else {
    params.set_output("Min", bounds_with_grease_pencil->min);
    params.set_output("Max", bounds_with_grease_pencil->max);
  }

  /* Generate the bounding box meshes inside each unique geometry set (including individually for
   * every instance). Because geometry components are reference counted anyway, we can just
   * repurpose the original geometry sets for the output. */
  if (params.output_is_required("Bounding Box")) {
    geometry_set.modify_geometry_sets([&](GeometrySet &sub_geometry) {
      std::optional<Bounds<float3>> sub_bounds;
      bool has_boundbox = false;

      /* Reuse the min and max calculation if this is the main "real" geometry set. */
      if (&sub_geometry == &geometry_set) {
        sub_bounds = bounds;
      }
      else {
        sub_bounds = sub_geometry.compute_boundbox_without_instances();
      }

      if (sub_bounds) {
        Mesh *mesh = create_mesh_from_boundbox(*sub_bounds);
        sub_geometry.replace_mesh(mesh);
        sub_geometry.keep_only_during_modify({GeometryComponent::Type::Mesh});
        has_boundbox = true;
      }
      if (geometry_set.has_grease_pencil()) {
        if (boundbox_from_grease_pencil(geometry_set)) {
          has_boundbox = true;
        }
      }

      if (!has_boundbox) {
        sub_geometry.remove_geometry_during_modify();
      }
    });

    params.set_output("Bounding Box", std::move(geometry_set));
  }
}

static void node_register()
{
  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_BOUNDING_BOX, "Bounding Box", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_bounding_box_cc
