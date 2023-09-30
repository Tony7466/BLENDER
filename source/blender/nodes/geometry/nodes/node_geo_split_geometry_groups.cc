/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "GEO_mesh_copy_selection.hh"
#include "GEO_randomize.hh"

#include "BKE_instances.hh"
#include "BKE_mesh.hh"

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
  b.add_output<decl::Int>("Group ID").field_on_all();
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

  AnonymousAttributeIDPtr dst_group_id_attribute_id =
      params.get_output_anonymous_attribute_id_if_needed("Group ID");

  Map<int, int> dst_index_by_group_id;

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
    const int groups_num = indices_by_group.size();

    Vector<int> groups_ids_ordered;
    groups_ids_ordered.extend(indices_by_group.keys().begin(), indices_by_group.keys().end());

    Vector<GeometrySet> group_geometries(groups_num);

    threading::EnumerableThreadSpecific<Array<bool>> group_selection_per_thread{
        [&]() { return Array<bool>(domain_size, false); }};

    threading::parallel_for(groups_ids_ordered.index_range(), 16, [&](const IndexRange range) {
      /* Need task isolation because of the thread local variable. */
      threading::isolate_task([&]() {
        MutableSpan<bool> group_selection = group_selection_per_thread.local();
        const VArray<bool> group_selection_varray = VArray<bool>::ForSpan(group_selection);
        for (const int group_index : range) {
          const int group_id = groups_ids_ordered[group_index];
          const Span<int> elements = indices_by_group.lookup(group_id);
          for (int i : elements) {
            group_selection[i] = true;
          }

          /* Using #mesh_copy_selection here is not ideal, because it can lead to O(n^2) behavior
           * when there are many groups. */
          std::optional<Mesh *> group_mesh_opt = geometry::mesh_copy_selection(
              src_mesh, group_selection_varray, domain, propagation_info);
          GeometrySet &group_geometry = group_geometries[group_index];
          if (group_mesh_opt.has_value()) {
            if (Mesh *group_mesh = *group_mesh_opt) {
              group_geometry = GeometrySet::from_mesh(group_mesh);
            }
          }
          else {
            group_geometry.add(component);
          }

          for (int i : elements) {
            group_selection[i] = false;
          }
        }
      });
    });

    for (const int group_index : IndexRange(groups_num)) {
      const int group_id = groups_ids_ordered[group_index];
      dst_index_by_group_id.add(group_id, group_index);
      GeometrySet &group_geometry = group_geometries[group_index];
      const int handle = dst_instances->add_reference(std::move(group_geometry));
      dst_instances->add_instance(handle, float4x4::identity());
    }
  }

  if (dst_group_id_attribute_id) {
    SpanAttributeWriter dst_group_id =
        dst_instances->attributes_for_write().lookup_or_add_for_write_span<int>(
            *dst_group_id_attribute_id, ATTR_DOMAIN_INSTANCE);
    for (auto item : dst_index_by_group_id.items()) {
      dst_group_id.span[item.value] = item.key;
    }
    dst_group_id.finish();
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
