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

  VectorSet<Material *> all_materials;

  for (const int instance_i : IndexRange(instances->instances_num())) {
    std::string name = std::to_string(instance_i);
    const bke::InstanceReference &reference = references[reference_handles[instance_i]];

    bke::greasepencil::Drawing &drawing =
        reinterpret_cast<GreasePencilDrawing *>(grease_pencil->drawing(instance_i))->wrap();
    bke::greasepencil::Layer &layer = grease_pencil->add_layer(std::move(name));
    layer.add_frame(0)->drawing_index = instance_i;
    drawing.add_user();

    GeometrySet instance_geometry;
    reference.to_geometry_set(instance_geometry);
    const Curves *instance_curves = instance_geometry.get_curves();
    if (!instance_curves) {
      continue;
    }

    bke::CurvesGeometry &strokes = drawing.strokes_for_write();
    strokes = instance_curves->geometry.wrap();

    Vector<int> new_material_indices;
    for (Material *material : Span{instance_curves->mat, instance_curves->totcol}) {
      new_material_indices.append(all_materials.index_of_or_add(material));
    }

    /* Remap material indices. */
    bke::SpanAttributeWriter<int> material_indices =
        strokes.attributes_for_write().lookup_or_add_for_write_span<int>("material_index",
                                                                         bke::AttrDomain::Curve);
    for (int &material_index : material_indices.span) {
      if (material_index >= 0 && material_index < new_material_indices.size()) {
        material_index = new_material_indices[material_index];
      }
    }
    material_indices.finish();
  }

  grease_pencil->material_array_num = all_materials.size();
  grease_pencil->material_array = MEM_cnew_array<Material *>(all_materials.size(), __func__);
  initialized_copy_n(all_materials.data(), all_materials.size(), grease_pencil->material_array);

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
