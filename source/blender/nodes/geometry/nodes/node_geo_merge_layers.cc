/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "GEO_merge_layers.hh"

#include "BKE_grease_pencil.hh"

namespace blender::nodes::node_geo_merge_layers_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Grease Pencil")
      .supported_type(GeometryComponent::Type::GreasePencil);
  b.add_input<decl::Bool>("Selection").default_value(true).hide_value();
  b.add_output<decl::Geometry>("Grease Pencil").propagate_all();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry = params.extract_input<GeometrySet>("Grease Pencil");
  Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");

  const NodeAttributeFilter attribute_filter = params.get_attribute_filter("Grease Pencil");

  const GreasePencil *src_grease_pencil = geometry.get_grease_pencil();
  if (src_grease_pencil) {
    using namespace bke::greasepencil;
    const int layers_num = src_grease_pencil->layers().size();

    bke::GreasePencilFieldContext field_context{*src_grease_pencil};
    FieldEvaluator field_evaluator{field_context, layers_num};
    field_evaluator.add(selection_field);
    field_evaluator.evaluate();
    const VArray<bool> selection = field_evaluator.get_evaluated<bool>(0);

    Vector<Vector<int>> layers_map;
    Map<StringRef, int> new_layer_index_by_name;

    for (const int layer_i : IndexRange(layers_num)) {
      const bool is_selected = selection[layer_i];
      if (!is_selected) {
        layers_map.append({layer_i});
        continue;
      }

      const Layer *layer = src_grease_pencil->layer(layer_i);
      const int new_layer_index = new_layer_index_by_name.lookup_or_add_cb(
          layer->name(), [&]() { return layers_map.append_and_get_index_as(); });
      layers_map[new_layer_index].append(layer_i);
    }

    GreasePencil *new_grease_pencil = geometry::merge_layers(
        *src_grease_pencil, layers_map, attribute_filter);
    geometry.replace_grease_pencil(new_grease_pencil);
  }
  params.set_output("Grease Pencil", std::move(geometry));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_MERGE_LAYERS, "Merge Layers", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_merge_layers_cc
