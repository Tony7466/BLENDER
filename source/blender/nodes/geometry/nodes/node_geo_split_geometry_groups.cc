/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "GEO_mesh_copy_selection.hh"
#include "GEO_randomize.hh"

#include "BKE_instances.hh"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "RNA_enum_types.hh"

namespace blender::nodes::node_geo_split_geometry_groups_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry");
  b.add_input<decl::Bool>("Selection").supports_field().hide_value().default_value(true);
  b.add_input<decl::Int>("Group ID").supports_field().hide_value();
  b.add_output<decl::Geometry>("Geometry").propagate_all();
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  const bNode &node = params.node();
  const eAttrDomain domain = eAttrDomain(node.custom1);

  GeometrySet src_geometry = params.extract_input<GeometrySet>("Geometry");
  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const Field<int> group_id_field = params.extract_input<Field<int>>("Group ID");

  const AnonymousAttributePropagationInfo &propagation_info = params.get_output_propagation_info(
      "Geometry");

  bke::Instances *dst_instances = new bke::Instances();
  GeometrySet dst_geometry = GeometrySet::from_instances(dst_instances);

  if (src_geometry.has_mesh()) {
    const MeshComponent &component = *src_geometry.get_component<MeshComponent>();
    const Mesh &src_mesh = *component.get();
    const int domain_size = component.attribute_domain_size(domain);
    const bke::MeshFieldContext field_context{src_mesh, domain};
    FieldEvaluator field_evaluator{field_context, domain_size};
    field_evaluator.set_selection(selection_field);
    field_evaluator.add(group_id_field);
    field_evaluator.evaluate();
    const IndexMask selection = field_evaluator.get_evaluated_selection_as_mask();
    const VArraySpan<int> group_ids = field_evaluator.get_evaluated<int>(0);

    MultiValueMap<int, int> indices_by_group;
    selection.foreach_index([&](const int i) { indices_by_group.add(group_ids[i], i); });

    Array<bool> group_selection(domain_size, false);
    const VArray<bool> group_selection_varray = VArray<bool>::ForSpan(group_selection);
    for (auto item : indices_by_group.items()) {
      const int group_id = item.key;
      const Span<int> elements = item.value;
      for (int i : elements) {
        group_selection[i] = true;
      }

      std::optional<Mesh *> group_mesh_opt = geometry::mesh_copy_selection(
          src_mesh, group_selection_varray, domain, propagation_info);
      GeometrySet group_geometry;
      if (group_mesh_opt.has_value()) {
        if (Mesh *group_mesh = *group_mesh_opt) {
          group_geometry = GeometrySet::from_mesh(group_mesh);
        }
      }
      else {
        group_geometry.add(component);
      }
      const int handle = dst_instances->add_reference(std::move(group_geometry));
      dst_instances->add_instance(handle, float4x4::identity());

      for (int i : elements) {
        group_selection[i] = false;
      }
    }
  }

  geometry::debug_randomize_instance_order(dst_instances);

  params.set_output("Geometry", std::move(dst_geometry));
}

static void node_rna(StructRNA *srna)
{
  RNA_def_node_enum(srna,
                    "domain",
                    "Domain",
                    "Attribute domain for the selection and group id inputs",
                    rna_enum_attribute_domain_without_corner_items,
                    NOD_inline_enum_accessors(custom1),
                    ATTR_DOMAIN_POINT);
}

static void node_register()
{
  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_SPLIT_GEOMETRY_GROUPS, "Split Geometry Groups", NODE_CLASS_GEOMETRY);
  ntype.geometry_node_execute = node_geo_exec;
  ntype.declare = node_declare;
  ntype.draw_buttons = node_layout;
  nodeRegisterType(&ntype);

  node_rna(ntype.rna_ext.srna);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_split_geometry_groups_cc
