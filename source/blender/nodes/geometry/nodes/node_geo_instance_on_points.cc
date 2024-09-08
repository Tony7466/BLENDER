/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <atomic>

#include "BLI_array_utils.hh"
#include "BLI_linear_allocator.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

#include "GEO_join_geometries.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_instance_on_points_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>("Points").description("Points to instance on");
  b.add_input<decl::Bool>("Selection").default_value(true).field_on({0}).hide_value();
  b.add_input<decl::Geometry>("Instance").description("Geometry that is instanced on the points");
  b.add_input<decl::Bool>("Pick Instance")
      .field_on({0})
      .description(
          "Choose instances from the \"Instance\" input at each point instead of instancing the "
          "entire geometry");
  b.add_input<decl::Int>("Instance Index")
      .implicit_field_on(implicit_field_inputs::id_or_index, {0})
      .description(
          "Index of the instance used for each point. This is only used when Pick Instances "
          "is on. By default the point index is used");
  b.add_input<decl::Rotation>("Rotation").field_on({0}).description("Rotation of the instances");
  b.add_input<decl::Vector>("Scale")
      .default_value({1.0f, 1.0f, 1.0f})
      .subtype(PROP_XYZ)
      .field_on({0})
      .description("Scale of the instances");

  b.add_output<decl::Geometry>("Instances").propagate_all();
}

static void wrap_indices(const int total, MutableSpan<int> indices)
{
  threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      indices[i] = mod_i(indices[i], total);
    }
  });
}

static void fill_by_single_instance(const GeometrySet &instance,
                                    const float4x4 &transform,
                                    const IndexMask &mask,
                                    bke::Instances &instances)
{
  index_mask::masked_fill<int>(
      instances.reference_handles_for_write(), instances.add_reference(instance), mask);
  index_mask::masked_fill<float4x4>(instances.transforms_for_write(), transform, mask);
}

static GeometrySet instances_on_domain(const VArray<bool> &pick_instance,
                                       const VArray<int> &indices,
                                       const IndexMask &selection,
                                       const GeometrySet &instances,
                                       bool &ignore_realized_data)
{
  IndexMaskMemory memory;
  const IndexMask mapped_instance_selection = IndexMask::from_bools(
      selection, pick_instance, memory);
  const IndexMask single_instance_selection = mapped_instance_selection.complement(selection,
                                                                                   memory);

  ignore_realized_data |= single_instance_selection.is_empty() && instances.has_realized_data();

  bke::Instances *dst_instances = new bke::Instances();
  dst_instances->resize(selection.size());
  MutableSpan<int> dst_handles = dst_instances->reference_handles_for_write();
  MutableSpan<float4x4> dst_transforms = dst_instances->transforms_for_write();

  if (!single_instance_selection.is_empty()) {
    fill_by_single_instance(
        instances, float4x4::identity(), single_instance_selection, *dst_instances);
  }

  if (mapped_instance_selection.is_empty()) {
    return GeometrySet::from_instances(dst_instances);
  }

  const bke::Instances *src_instances = instances.get_instances();
  if (src_instances == nullptr) {
    fill_by_single_instance(
        GeometrySet(), float4x4::identity(), mapped_instance_selection, *dst_instances);
    return GeometrySet::from_instances(dst_instances);
  }

  const int instances_num = src_instances->instances_num();
  const Span<bke::InstanceReference> src_instance_handlers = src_instances->references();
  const Span<int> src_handles = src_instances->reference_handles();
  const Span<float4x4> src_transforms = src_instances->transforms();
  if (src_handles.is_empty()) {
    fill_by_single_instance(
        GeometrySet(), float4x4::identity(), mapped_instance_selection, *dst_instances);
    return GeometrySet::from_instances(dst_instances);
  }

  Array<int> gathered_indices(mapped_instance_selection.size());
  indices.materialize_compressed(mapped_instance_selection, gathered_indices.as_mutable_span());
  wrap_indices(instances_num, gathered_indices.as_mutable_span());

  Array<int> mapped_to_all_mapping(selection.min_array_size(), -1);
  index_mask::build_reverse_map(selection, mapped_to_all_mapping.as_mutable_span());
  mapped_instance_selection.foreach_index_optimized<int>(
      GrainSize(4096), [&](const int i, const int pos) {
        dst_transforms[mapped_to_all_mapping[i]] = src_transforms[gathered_indices[pos]];
      });

  array_utils::gather(src_handles, gathered_indices.as_span(), gathered_indices.as_mutable_span());
  VectorSet<int> unique_handlers(gathered_indices.as_span());
  threading::parallel_for(gathered_indices.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      gathered_indices[i] = unique_handlers.index_of(gathered_indices[i]);
    }
  });

  Array<int> unique_handlers_indices(unique_handlers.size());
  for (const int unique_handler_i : unique_handlers.index_range()) {
    const int unique_handler_index = unique_handlers[unique_handler_i];
    const bke::InstanceReference &reference = src_instance_handlers[unique_handler_index];
    unique_handlers_indices[unique_handler_i] = dst_instances->add_reference(reference);
  }

  array_utils::gather(unique_handlers_indices.as_span(),
                      gathered_indices.as_span(),
                      gathered_indices.as_mutable_span());
  // array_utils::scatter(gathered_indices.as_span(), mapped_instance_selection, dst_handles);

  mapped_instance_selection.foreach_index_optimized<int>(
      GrainSize(4096), [&](const int i, const int pos) {
        dst_handles[mapped_to_all_mapping[i]] = gathered_indices[pos];
      });

  return GeometrySet::from_instances(dst_instances);
}

static void apply_transform(const VArray<float3> positions,
                            const VArray<math::Quaternion> &rotations,
                            const VArray<float3> &scales,
                            const IndexMask &selection,
                            MutableSpan<float4x4> transformations)
{
  selection.foreach_index(GrainSize(4096), [&](const int src_index, const int dst_pos) {
    const float4x4 to_apply = math::from_loc_rot_scale<float4x4>(
        positions[src_index], rotations[src_index], scales[src_index]);
    transformations[dst_pos] = to_apply * transformations[dst_pos];
  });
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  GeometrySet instance = params.extract_input<GeometrySet>("Instance");
  instance.ensure_owns_direct_data();
  const NodeAttributeFilter &attribute_filter = params.get_attribute_filter("Instances");

  const Field<bool> selection_field = params.extract_input<Field<bool>>("Selection");
  const Field<bool> pick_instance_field = params.extract_input<Field<bool>>("Pick Instance");
  const Field<int> indices_field = params.extract_input<Field<int>>("Instance Index");
  const Field<math::Quaternion> rotations_field = params.extract_input<Field<math::Quaternion>>(
      "Rotation");
  const Field<float3> scales_field = params.extract_input<Field<float3>>("Scale");

  std::atomic<bool> has_skiped_realized_instances = false;

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    /* It's important not to invalidate the existing #InstancesComponent because it owns references
     * to other geometry sets that are processed by this node. */
    InstancesComponent &instances_component =
        geometry_set.get_component_for_write<InstancesComponent>();
    bke::Instances *dst_instances = instances_component.release();

    Vector<GeometrySet, 4> instances_set;
    if (dst_instances != nullptr) {
      instances_set.append(GeometrySet::from_instances(dst_instances));
    }

    const static std::array<GeometryComponent::Type, 3> types = {
        GeometryComponent::Type::Mesh,
        GeometryComponent::Type::PointCloud,
        GeometryComponent::Type::Curve};

    Map<StringRef, AttributeKind> attributes_to_propagate;
    geometry_set.gather_attributes_for_propagation(types,
                                                   GeometryComponent::Type::Instance,
                                                   false,
                                                   attribute_filter,
                                                   attributes_to_propagate);
    attributes_to_propagate.remove("position");
    attributes_to_propagate.remove(".reference_index");

    for (const GeometryComponent::Type type : types) {
      if (!geometry_set.has(type)) {
        continue;
      }

      const GeometryComponent &component = *geometry_set.get_component(type);
      const bke::AttributeAccessor src_attributes = *component.attributes();
      const bke::GeometryFieldContext field_context(component, AttrDomain::Point);
      fn::FieldEvaluator evaluator(field_context, src_attributes.domain_size(AttrDomain::Point));

      evaluator.set_selection(selection_field);
      evaluator.add(pick_instance_field);
      evaluator.add(indices_field);
      evaluator.add(rotations_field);
      evaluator.add(scales_field);

      evaluator.evaluate();

      const VArray<bool> pick_instance = evaluator.get_evaluated<bool>(0);
      const VArray<int> indices = evaluator.get_evaluated<int>(1);
      const VArray<math::Quaternion> rotations = evaluator.get_evaluated<math::Quaternion>(2);
      const VArray<float3> scales = evaluator.get_evaluated<float3>(3);
      const IndexMask selection = evaluator.get_evaluated_selection_as_mask();

      if (selection.is_empty()) {
        continue;
      }

      bool ignore_realized_data = false;
      GeometrySet instances_on_component = instances_on_domain(
          pick_instance, indices, selection, instance, ignore_realized_data);
      if (ignore_realized_data) {
        has_skiped_realized_instances = true;
      }

      bke::Instances &dst_instances = *instances_on_component.get_instances_for_write();
      const VArray<float3> src_positions = *src_attributes.lookup<float3>("position");
      apply_transform(
          src_positions, rotations, scales, selection, dst_instances.transforms_for_write());

      // Need to exclude skiped attributes from filter to do not copy them.
      // bke::copy_attributes(src_attributes, AttrDomain::Point, AttrDomain::Instance,
      // attribute_filter, dst_instances.attributes_for_write());

      instances_set.append(std::move(instances_on_component));
    }

    if (geometry_set.has_grease_pencil()) {
      /*
      using namespace bke::greasepencil;
      const GreasePencil &grease_pencil = *geometry_set.get_grease_pencil();

      bke::Instances *gp_instances = instances_component.get_for_write();
      for (const int layer_index : grease_pencil.layers().index_range()) {
        const Drawing *drawing = grease_pencil.get_eval_drawing(*grease_pencil.layer(layer_index));
        if (drawing == nullptr) {
          continue;
        }
        const bke::CurvesGeometry &src_curves = drawing->strokes();
        if (src_curves.curves_num() == 0) {
          /* Add an empty reference so the number of layers and instances match.
           * This makes it easy to reconstruct the layers afterwards and keep their attributes.
           * Although in this particular case we don't propagate the attributes. *//*
          const int handle = gp_instances->add_reference(bke::InstanceReference());
          gp_instances->add_instance(handle, float4x4::identity());

          continue;
        }
        /* TODO: Attributes are not propagating from the curves or the points. */ /*
        bke::Instances *instances = new bke::Instances();
        const bke::GreasePencilLayerFieldContext field_context(
            grease_pencil, AttrDomain::Point, layer_index);
        add_instances_from_component(*instances,
                                     src_curves.attributes(),
                                     instance,
                                     field_context,
                                     params,
                                     attributes_to_propagate);
        GeometrySet temp_set = GeometrySet::from_instances(instances);
        const int handle = gp_instances->add_reference(bke::InstanceReference{temp_set});
        gp_instances->add_instance(handle, float4x4::identity());
      }
      */
    }

    GeometrySet all_instances = geometry::join_geometries(instances_set.as_span(),
                                                          attribute_filter);
    // *dst_instances = std::move(*(new bke::Instances()));
    // *dst_instances =
    // std::move(*all_instances.get_component_for_write<bke::InstancesComponent>().release());
    instances_component.replace(
        all_instances.get_component_for_write<bke::InstancesComponent>().release());
    geometry_set.remove_geometry_during_modify();
  });

  if (has_skiped_realized_instances) {
    params.error_message_add(NodeWarningType::Info,
                             TIP_("Realized geometry is not used when pick instances is true"));
  }

  /* Unused references may have been added above. Remove those now so that other nodes don't
   * process them needlessly.
   * This should eventually be moved into the loop above, but currently this is quite tricky
   * because it might remove references that the loop still wants to iterate over. */
  if (bke::Instances *instances = geometry_set.get_instances_for_write()) {
    instances->remove_unused_references();
  }

  params.set_output("Instances", std::move(geometry_set));
}

static void node_register()
{
  static blender::bke::bNodeType ntype;

  geo_node_type_base(
      &ntype, GEO_NODE_INSTANCE_ON_POINTS, "Instance on Points", NODE_CLASS_GEOMETRY);
  ntype.declare = node_declare;
  ntype.geometry_node_execute = node_geo_exec;
  blender::bke::node_register_type(&ntype);
}
NOD_REGISTER_NODE(node_register)

}  // namespace blender::nodes::node_geo_instance_on_points_cc
