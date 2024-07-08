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
  b.add_input<decl::Geometry>("Curve Instances").only_instances();
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value().field_on_all();
  b.add_output<decl::Geometry>("Grease Pencil");
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet instances_geometry = params.extract_input<GeometrySet>("Curve Instances");
  const bke::Instances *instances = instances_geometry.get_instances();
  if (!instances) {
    params.set_default_remaining_outputs();
    return;
  }
  const Span<int> reference_handles = instances->reference_handles();
  const Span<bke::InstanceReference> references = instances->references();
  const Span<float4x4> transforms = instances->transforms();

  const int instances_num = instances->instances_num();
  if (instances_num == 0) {
    params.set_default_remaining_outputs();
    return;
  }

  const bke::InstancesFieldContext field_context{*instances};
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  FieldEvaluator evaluator{field_context, instances_num};
  evaluator.set_selection(selection_field);
  evaluator.evaluate();
  const IndexMask instance_selection = evaluator.get_evaluated_selection_as_mask();

  const int layer_num = instance_selection.size();
  if (layer_num == 0) {
    params.set_default_remaining_outputs();
    return;
  }

  GreasePencil *grease_pencil = BKE_grease_pencil_new_nomain();
  grease_pencil->add_layers_with_empty_drawings_for_eval(layer_num);

  VectorSet<Material *> all_materials;

  instance_selection.foreach_index([&](const int instance_i, const int layer_i) {
    const bke::InstanceReference &reference = references[reference_handles[instance_i]];
    const std::string name = reference.name();

    bke::greasepencil::Layer *layer = grease_pencil->layer(layer_i);
    BLI_assert(layer);
    layer->set_name(name);
    layer->set_local_transform(transforms[instance_i]);

    bke::greasepencil::Drawing *drawing = grease_pencil->get_eval_drawing(*layer);
    BLI_assert(drawing);

    GeometrySet instance_geometry;
    reference.to_geometry_set(instance_geometry);
    const Curves *instance_curves = instance_geometry.get_curves();
    if (!instance_curves) {
      return;
    }

    bke::CurvesGeometry &strokes = drawing->strokes_for_write();
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
  });

  grease_pencil->material_array_num = all_materials.size();
  grease_pencil->material_array = MEM_cnew_array<Material *>(all_materials.size(), __func__);
  initialized_copy_n(all_materials.data(), all_materials.size(), grease_pencil->material_array);

  const bke::AttributeAccessor instances_attributes = instances->attributes();
  bke::MutableAttributeAccessor grease_pencil_attributes = grease_pencil->attributes_for_write();
  instances_attributes.for_all([&](const AttributeIDRef &attribute_id,
                                   const AttributeMetaData &meta_data) {
    if (instances_attributes.is_builtin(attribute_id) &&
        !grease_pencil_attributes.is_builtin(attribute_id))
    {
      return true;
    }
    const GAttributeReader src_attribute = instances_attributes.lookup(attribute_id);
    if (!src_attribute) {
      return true;
    }
    if (instance_selection.size() == instances_num && src_attribute.varray.is_span() &&
        src_attribute.sharing_info)
    {
      /* Try reusing existing attribute array. */
      grease_pencil_attributes.add(
          attribute_id,
          AttrDomain::Layer,
          meta_data.data_type,
          bke::AttributeInitShared{src_attribute.varray.get_internal_span().data(),
                                   *src_attribute.sharing_info});
      return true;
    }
    if (!grease_pencil_attributes.add(
            attribute_id, AttrDomain::Layer, meta_data.data_type, bke::AttributeInitConstruct()))
    {
      return true;
    }
    bke::GSpanAttributeWriter dst_attribute = grease_pencil_attributes.lookup_for_write_span(
        attribute_id);
    array_utils::gather(src_attribute.varray, instance_selection, dst_attribute.span);
    dst_attribute.finish();
    return true;
  });

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
