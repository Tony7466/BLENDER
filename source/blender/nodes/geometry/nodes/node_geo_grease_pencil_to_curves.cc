/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_grease_pencil_to_curves_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Grease Pencil")
      .supported_type(bke::GeometryComponent::Type::GreasePencil);
  b.add_output<decl::Geometry>("Instances");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet grease_pencil_geometry = params.extract_input<GeometrySet>("Grease Pencil");
  const GreasePencil *grease_pencil = grease_pencil_geometry.get_grease_pencil();
  if (!grease_pencil) {
    params.set_default_remaining_outputs();
    return;
  }

  const Span<const bke::greasepencil::Layer *> layers = grease_pencil->layers();

  bke::Instances *instances = new bke::Instances();
  std::optional<int> empty_geometry_handle;

  for (const int layer_i : layers.index_range()) {
    const bke::greasepencil::Layer &layer = *layers[layer_i];
    const bke::greasepencil::Drawing *drawing = grease_pencil->get_eval_drawing(layer);
    const float4x4 transform = layer.local_transform();
    if (!drawing) {
      if (!empty_geometry_handle.has_value()) {
        empty_geometry_handle = instances->add_reference(bke::InstanceReference());
      }
      instances->add_instance(*empty_geometry_handle, transform);
      continue;
    }
    const bke::CurvesGeometry &layer_strokes = drawing->strokes();
    Curves *curves_id = bke::curves_new_nomain(layer_strokes);
    curves_id->mat = static_cast<Material **>(MEM_dupallocN(grease_pencil->material_array));
    curves_id->totcol = grease_pencil->material_array_num;
    const int handle = instances->add_reference(GeometrySet::from_curves(curves_id));
    instances->add_instance(handle, transform);
  }

  params.set_output("Instances", GeometrySet::from_instances(instances));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_GREASE_PENCIL_TO_CURVES, "Grease Pencil to Curves", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  bke::node_type_size(&ntype, 160, 100, 320);

  bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_grease_pencil_to_curves_cc
