/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_curves_to_grease_pencil_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Instances").only_instances();
  b.add_output<decl::Geometry>("Grease Pencil");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet instances_geometry = params.extract_input<GeometrySet>("Instances");
  const bke::Instances *instances = instances_geometry.get_instances();
  if (!instances) {
    params.set_default_remaining_outputs();
    return;
  }
  const Span<int> reference_handles = instances->reference_handles();
  const Span<bke::InstanceReference> references = instances->references();
  const int instances_num = instances->instances_num();

  GreasePencil *grease_pencil = BKE_grease_pencil_new_nomain();
  grease_pencil->add_empty_drawings(instances_num);

  for (const int instance_i : IndexRange(instances->instances_num())) {
    std::string name = std::to_string(instance_i);
    const bke::InstanceReference &reference = references[reference_handles[instance_i]];
    if (reference.type() != bke::InstanceReference::Type::GeometrySet) {
      /* TODO */
      continue;
    }
    GeometrySet instance_geometry = reference.geometry_set();
    const Curves *instance_curves = instance_geometry.get_curves();
    if (!instance_curves) {
      /* TODO: Create empty layer? */
      continue;
    }

    bke::greasepencil::Drawing &drawing =
        reinterpret_cast<GreasePencilDrawing *>(grease_pencil->drawing(instance_i))->wrap();
    drawing.strokes_for_write() = instance_curves->geometry.wrap();

    bke::greasepencil::Layer &layer = grease_pencil->add_layer(std::move(name));
    layer.add_frame(0)->drawing_index = instance_i;
    drawing.add_user();
  }

  params.set_output("Grease Pencil", GeometrySet::from_grease_pencil(grease_pencil));
}

static void node_register()
{
  static bke::bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_CURVES_TO_GREASE_PENCIL, "Curves to Grease Pencil", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  bke::node_type_size(&ntype, 160, 100, 320);

  bke::nodeRegisterType(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_curves_to_grease_pencil_cc
