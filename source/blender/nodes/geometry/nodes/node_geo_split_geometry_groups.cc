/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_geometry_util.hh"

#include "GEO_mesh_copy_selection.hh"
#include "GEO_randomize.hh"

#include "BKE_curves.hh"
#include "BKE_instances.hh"
#include "BKE_mesh.hh"
#include "BKE_pointcloud.h"

#include "NOD_rna_define.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "DNA_pointcloud_types.h"

#include "RNA_enum_types.hh"

#include "BLI_array_utils.hh"

namespace blender::nodes::node_geo_split_geometry_groups_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Geometry")
      .supported_type({GeometryComponent::Type::Mesh,
                       GeometryComponent::Type::PointCloud,
                       GeometryComponent::Type::Curve});
  b.add_input<decl::Bool>("Selection").supports_field().hide_value().default_value(true);
  b.add_input<decl::Int>("Group ID").supports_field().hide_value();
  b.add_output<decl::Geometry>("Geometry")
      .propagate_all()
      .description("All geometry groups as separate instances");
  b.add_output<decl::Int>("Group ID")
      .field_on_all()
      .description("The group id of each group instance");
}

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiLayoutSetPropSep(layout, true);
  uiLayoutSetPropDecorate(layout, false);
  uiItemR(layout, ptr, "domain", UI_ITEM_NONE, "", ICON_NONE);
}

static void ensure_group_geometries(Map<int, std::unique_ptr<GeometrySet>> &geometry_by_group_id,
                                    const Span<int> group_ids)
{
  for (const int group_id : group_ids) {
    geometry_by_group_id.lookup_or_add_cb(group_id,
                                          []() { return std::make_unique<GeometrySet>(); });
  }
}

static void split_mesh_groups(const MeshComponent &component,
                              const eAttrDomain domain,
                              const Field<bool> &selection_field,
                              const Field<int> &group_id_field,
                              const AnonymousAttributePropagationInfo &propagation_info,
                              Map<int, std::unique_ptr<GeometrySet>> &geometry_by_group_id)
{
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

  Vector<int> group_ids_ordered;
  group_ids_ordered.extend(indices_by_group.keys().begin(), indices_by_group.keys().end());
  ensure_group_geometries(geometry_by_group_id, group_ids_ordered);

  threading::EnumerableThreadSpecific<Array<bool>> group_selection_per_thread{
      [&]() { return Array<bool>(domain_size, false); }};

  threading::parallel_for(group_ids_ordered.index_range(), 16, [&](const IndexRange range) {
    /* Need task isolation because of the thread local variable. */
    threading::isolate_task([&]() {
      MutableSpan<bool> group_selection = group_selection_per_thread.local();
      const VArray<bool> group_selection_varray = VArray<bool>::ForSpan(group_selection);
      for (const int group_index : range) {
        const int group_id = group_ids_ordered[group_index];
        const Span<int> elements = indices_by_group.lookup(group_id);
        for (int i : elements) {
          group_selection[i] = true;
        }

        /* Using #mesh_copy_selection here is not ideal, because it can lead to O(n^2) behavior
         * when there are many groups. */
        std::optional<Mesh *> group_mesh_opt = geometry::mesh_copy_selection(
            src_mesh, group_selection_varray, domain, propagation_info);
        GeometrySet &group_geometry = *geometry_by_group_id.lookup(group_id);
        if (group_mesh_opt.has_value()) {
          if (Mesh *group_mesh = *group_mesh_opt) {
            group_geometry.replace_mesh(group_mesh);
          }
          else {
            group_geometry.replace_mesh(nullptr);
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
}

static void split_pointcloud_groups(const PointCloudComponent &component,
                                    const Field<bool> &selection_field,
                                    const Field<int> &group_id_field,
                                    const AnonymousAttributePropagationInfo &propagation_info,
                                    Map<int, std::unique_ptr<GeometrySet>> &geometry_by_group_id)
{
  const PointCloud &src_pointcloud = *component.get();
  const int points_num = src_pointcloud.totpoint;

  const bke::PointCloudFieldContext field_context{src_pointcloud};
  FieldEvaluator field_evaluator{field_context, points_num};
  field_evaluator.set_selection(selection_field);
  field_evaluator.add(group_id_field);
  field_evaluator.evaluate();
  const IndexMask selection = field_evaluator.get_evaluated_selection_as_mask();
  const VArraySpan<int> group_ids = field_evaluator.get_evaluated<int>(0);

  MultiValueMap<int, int> indices_by_group;
  selection.foreach_index([&](const int i) { indices_by_group.add(group_ids[i], i); });
  const int groups_num = indices_by_group.size();

  Vector<int> group_ids_ordered;
  group_ids_ordered.extend(indices_by_group.keys().begin(), indices_by_group.keys().end());
  ensure_group_geometries(geometry_by_group_id, group_ids_ordered);

  threading::parallel_for(group_ids_ordered.index_range(), 16, [&](const IndexRange range) {
    for (const int group_index : range) {
      const int group_id = group_ids_ordered[group_index];
      const Span<int> point_indices = indices_by_group.lookup(group_id);
      const int group_points_num = point_indices.size();
      PointCloud *group_pointcloud = BKE_pointcloud_new_nomain(group_points_num);

      const AttributeAccessor src_attributes = src_pointcloud.attributes();
      MutableAttributeAccessor dst_attributes = group_pointcloud->attributes_for_write();
      bke::gather_attributes(
          src_attributes, ATTR_DOMAIN_POINT, propagation_info, {}, point_indices, dst_attributes);

      GeometrySet &group_geometry = *geometry_by_group_id.lookup(group_id);
      group_geometry.replace_pointcloud(group_pointcloud);
    }
  });
}

static void split_curve_groups(const bke::CurveComponent &component,
                               const eAttrDomain domain,
                               const Field<bool> &selection_field,
                               const Field<int> &group_id_field,
                               const AnonymousAttributePropagationInfo &propagation_info,
                               Map<int, std::unique_ptr<GeometrySet>> &geometry_by_group_id)
{
  const bke::CurvesGeometry &src_curves = component.get()->geometry.wrap();
  const int domain_size = src_curves.attributes().domain_size(domain);

  const bke::CurvesFieldContext field_context{src_curves, domain};
  FieldEvaluator field_evaluator{field_context, domain_size};
  field_evaluator.set_selection(selection_field);
  field_evaluator.add(group_id_field);
  field_evaluator.evaluate();
  const IndexMask selection = field_evaluator.get_evaluated_selection_as_mask();
  const VArraySpan<int> group_ids = field_evaluator.get_evaluated<int>(0);

  MultiValueMap<int, int> indices_by_group;
  selection.foreach_index([&](const int i) { indices_by_group.add(group_ids[i], i); });
  const int groups_num = indices_by_group.size();

  Vector<int> group_ids_ordered;
  group_ids_ordered.extend(indices_by_group.keys().begin(), indices_by_group.keys().end());
  ensure_group_geometries(geometry_by_group_id, group_ids_ordered);

  threading::parallel_for(group_ids_ordered.index_range(), 16, [&](const IndexRange range) {
    IndexMaskMemory memory;
    for (const int group_index : range) {
      const int group_id = group_ids_ordered[group_index];
      const Span<int> elements = indices_by_group.lookup(group_id);
      const IndexMask elements_mask = IndexMask::from_indices(elements, memory);
      std::optional<bke::CurvesGeometry> group_curves;
      if (domain == ATTR_DOMAIN_POINT) {
        group_curves = bke::curves_copy_point_selection(
            src_curves, elements_mask, propagation_info);
      }
      else {
        group_curves = bke::curves_copy_curve_selection(
            src_curves, elements_mask, propagation_info);
      }
      Curves *group_curves_id = bke::curves_new_nomain(std::move(*group_curves));
      GeometrySet &group_geometry = *geometry_by_group_id.lookup(group_id);
      group_geometry.replace_curves(group_curves_id);
    }
  });
}

static void split_instance_groups(const InstancesComponent &component,
                                  const Field<bool> &selection_field,
                                  const Field<int> &group_id_field,
                                  const AnonymousAttributePropagationInfo &propagation_info,
                                  Map<int, std::unique_ptr<GeometrySet>> &geometry_by_group_id)
{
  const bke::Instances &src_instances = *component.get();
  const int instances_num = src_instances.instances_num();

  const bke::InstancesFieldContext field_context{src_instances};
  FieldEvaluator field_evaluator{field_context, instances_num};
  field_evaluator.set_selection(selection_field);
  field_evaluator.add(group_id_field);
  field_evaluator.evaluate();
  const IndexMask selection = field_evaluator.get_evaluated_selection_as_mask();
  const VArraySpan<int> group_ids = field_evaluator.get_evaluated<int>(0);

  MultiValueMap<int, int> indices_by_group;
  selection.foreach_index([&](const int i) { indices_by_group.add(group_ids[i], i); });
  const int groups_num = indices_by_group.size();

  Vector<int> group_ids_ordered;
  group_ids_ordered.extend(indices_by_group.keys().begin(), indices_by_group.keys().end());
  ensure_group_geometries(geometry_by_group_id, group_ids_ordered);

  threading::parallel_for(group_ids_ordered.index_range(), 16, [&](const IndexRange range) {
    IndexMaskMemory memory;
    for (const int group_index : range) {
      const int group_id = group_ids_ordered[group_index];
      const Span<int> instance_indices = indices_by_group.lookup(group_id);

      bke::Instances *group_instances = new bke::Instances();
      group_instances->resize(instance_indices.size());

      for (const bke::InstanceReference &reference : src_instances.references()) {
        group_instances->add_reference(reference);
      }

      array_utils::gather(
          src_instances.transforms(), instance_indices, group_instances->transforms());
      array_utils::gather(src_instances.reference_handles(),
                          instance_indices,
                          group_instances->reference_handles());
      bke::gather_attributes(src_instances.attributes(),
                             ATTR_DOMAIN_INSTANCE,
                             propagation_info,
                             {},
                             instance_indices,
                             group_instances->attributes_for_write());
      group_instances->remove_unused_references();

      GeometrySet &group_geometry = *geometry_by_group_id.lookup(group_id);
      group_geometry.replace_instances(group_instances);
    }
  });
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

  Map<int, std::unique_ptr<GeometrySet>> geometry_by_group_id;

  if (src_geometry.has_mesh() &&
      ELEM(domain, ATTR_DOMAIN_POINT, ATTR_DOMAIN_EDGE, ATTR_DOMAIN_FACE)) {
    const auto &component = *src_geometry.get_component<MeshComponent>();
    split_mesh_groups(component,
                      domain,
                      selection_field,
                      group_id_field,
                      propagation_info,
                      geometry_by_group_id);
  }
  if (src_geometry.has_pointcloud() && domain == ATTR_DOMAIN_POINT) {
    const auto &component = *src_geometry.get_component<PointCloudComponent>();
    split_pointcloud_groups(
        component, selection_field, group_id_field, propagation_info, geometry_by_group_id);
  }
  if (src_geometry.has_curves() && ELEM(domain, ATTR_DOMAIN_POINT, ATTR_DOMAIN_CURVE)) {
    const auto &component = *src_geometry.get_component<bke::CurveComponent>();
    split_curve_groups(component,
                       domain,
                       selection_field,
                       group_id_field,
                       propagation_info,
                       geometry_by_group_id);
  }
  if (src_geometry.has_instances() && domain == ATTR_DOMAIN_INSTANCE) {
    const auto &component = *src_geometry.get_component<bke::InstancesComponent>();
    split_instance_groups(
        component, selection_field, group_id_field, propagation_info, geometry_by_group_id);
  }

  bke::Instances *dst_instances = new bke::Instances();
  GeometrySet dst_geometry = GeometrySet::from_instances(dst_instances);
  const int total_groups_num = geometry_by_group_id.size();
  dst_instances->resize(total_groups_num);

  AnonymousAttributeIDPtr dst_group_id_attribute_id =
      params.get_output_anonymous_attribute_id_if_needed("Group ID");
  SpanAttributeWriter<int> dst_group_id;
  if (dst_group_id_attribute_id) {
    dst_group_id = dst_instances->attributes_for_write().lookup_or_add_for_write_span<int>(
        *dst_group_id_attribute_id, ATTR_DOMAIN_INSTANCE);
  }

  dst_instances->transforms().fill(float4x4::identity());
  MutableSpan<int> dst_instance_handles = dst_instances->reference_handles();
  std::iota(dst_instance_handles.begin(), dst_instance_handles.end(), 0);

  int index = 0;
  for (auto item : geometry_by_group_id.items()) {
    const int group_id = item.key;
    std::unique_ptr<GeometrySet> &group_geometry = item.value;
    dst_instances->add_reference(std::move(group_geometry));
    if (dst_group_id) {
      dst_group_id.span[index] = group_id;
    }
    index++;
  }

  if (dst_group_id) {
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
