/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <atomic>

#include "BLI_array_utils.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"

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

void gather_attributes(const bke::AttributeAccessor src_attributes,
                       const AttrDomain src_domain,
                       const AttrDomain dst_domain,
                       const Map<AttributeIDRef, AttributeKind> &attributes_to_propagate,
                       const Span<int> dst_to_src_mapping,
                       bke::MutableAttributeAccessor dst_attributes)
{
  for (const auto item : attributes_to_propagate.items()) {
    const AttributeIDRef &id = item.key;
    const eCustomDataType data_type = item.value.data_type;

    const GAttributeReader src = src_attributes.lookup(id, src_domain, data_type);
    if (!src) {
      continue;
    }
    bke::GSpanAttributeWriter dst = dst_attributes.lookup_or_add_for_write_only_span(id, dst_domain, data_type);
    if (!dst) {
      continue;
    }
    bke::attribute_math::convert_to_static_type(src.varray.type(), [&](auto dymmu) {
      using T = decltype(dymmu);
      array_utils::gather<T>(src.varray.typed<T>(), dst_to_src_mapping, dst.span.typed<T>().take_back(dst_to_src_mapping.size()));
    });
    dst.finish();
    continue;
  }
}

static void gather_transform_apply(const Span<float4x4> to_apply, const Span<int> indices, MutableSpan<float4x4> original)
{
  BLI_assert(indices.size() == original.size());
  threading::parallel_for(original.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      original[i] *= to_apply[indices[i]];
    }
  });
}

static void wrap_indices(const int total, MutableSpan<int> indices)
{
  threading::parallel_for(indices.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      indices[i] = mod_i(indices[i], total);
    }
  });
}

static void added_mapped_instances_from_component(const GeometrySet &instance,
                                                  VArray<int> indices,
                                                  const IndexMask &pick_instance_mask,
                                                  const IndexMask &top_level_instances_mask,
                                                  bke::Instances &dst_component,
                                                  Array<int> &r_dst_to_src_mapping)
{
  const bke::Instances *src_instances = instance.get_instances();
  const bool has_picked_instance = src_instances != nullptr && src_instances->instances_num() > 0;
  const int total_new_instances = top_level_instances_mask.size() + int(has_picked_instance) * pick_instance_mask.size();
  dst_component.resize(dst_component.instances_num() + total_new_instances);
  MutableSpan<int> dst_handles = dst_component.reference_handles_for_write().take_back(total_new_instances);
  MutableSpan<float4x4> dst_transforms = dst_component.transforms_for_write().take_back(total_new_instances);

  if (!top_level_instances_mask.is_empty()) {
    const int single_handler_i = dst_component.add_reference(instance);
    dst_handles.take_back(top_level_instances_mask.size()).fill(single_handler_i);
    dst_transforms.take_back(top_level_instances_mask.size()).fill(float4x4::identity());
  }

  r_dst_to_src_mapping.reinitialize(total_new_instances);
  top_level_instances_mask.to_indices(r_dst_to_src_mapping.as_mutable_span().take_back(top_level_instances_mask.size()));
  if (!has_picked_instance) {
    return;
  }
  pick_instance_mask.to_indices(r_dst_to_src_mapping.as_mutable_span().take_front(pick_instance_mask.size()));

  Array<int> src_in_dst_mapping(pick_instance_mask.size());
  array_utils::gather(indices, pick_instance_mask, src_in_dst_mapping.as_mutable_span());
  wrap_indices(src_instances->instances_num(), src_in_dst_mapping);
  array_utils::gather(src_instances->reference_handles(), src_in_dst_mapping.as_span(), src_in_dst_mapping.as_mutable_span());

  const Span<bke::InstanceReference> src_references = src_instances->references();
  VectorSet<int> unique_handlers(src_in_dst_mapping);
  Array<int> unique_handler_indices(unique_handlers.size());
  for (const int unique_handler_i : unique_handlers.index_range()) {
    const int unique_handler_index = unique_handlers[unique_handler_i];
    const bke::InstanceReference &reference = src_references[unique_handler_index];
    unique_handler_indices[unique_handler_i] = dst_component.add_reference(reference);
  }

  pick_instance_mask.foreach_index(GrainSize(4096), [&](const int /*index*/, const int pos) {
    dst_handles[pos] = unique_handler_indices[unique_handlers.index_of(src_in_dst_mapping[pos])];
  });

  const Span<float4x4> src_transforms = src_instances->transforms();
  array_utils::gather<float4x4>(src_transforms, src_in_dst_mapping.as_span(), dst_transforms.take_front(pick_instance_mask.size()));
}

static bool add_instances_from_points(const GeometryComponent &src_component,
                                      const GeometrySet &instance,
                                      const Map<AttributeIDRef, AttributeKind> &attributes_to_propagate,
                                      const Field<bool> &selection_field,
                                      const Field<bool> &pick_instance_field,
                                      const Field<int> &indices_field,
                                      const Field<math::Quaternion> &rotations_field,
                                      const Field<float3> &scales_field,
                                      bke::Instances &dst_component)
{
  const bke::GeometryFieldContext field_context(src_component, AttrDomain::Point);
  fn::FieldEvaluator evaluator(field_context, src_component.attribute_domain_size(AttrDomain::Point));

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
    return false;
  }

  IndexMaskMemory memory;
  const IndexMask picked_instances = IndexMask::from_bools(selection, pick_instance, memory);
  const IndexMask top_level_instances = picked_instances.complement(selection, memory);

  Array<int> dst_to_src_mapping;
  added_mapped_instances_from_component(instance, indices, picked_instances, top_level_instances, dst_component, dst_to_src_mapping);

  gather_attributes(*src_component.attributes(), AttrDomain::Point, AttrDomain::Instance, attributes_to_propagate, dst_to_src_mapping, dst_component.attributes_for_write());

  MutableSpan<float4x4> dst_transforms = dst_component.transforms_for_write().take_back(selection.size());
  const VArraySpan<float3> positions = *src_component.attributes()->lookup<float3>("position");

  threading::parallel_for(dst_to_src_mapping.index_range(), 4096, [&](const IndexRange range) {
    for (const int dst_index : range) {
      const int src_index = dst_to_src_mapping[dst_index];
      const float4x4 user_transform = math::from_loc_rot_scale<float4x4>(positions[src_index], rotations[src_index], scales[src_index]);
      dst_transforms[dst_index] = user_transform * dst_transforms[dst_index];
    }
  });

  return pick_instance.is_single() && pick_instance.get_internal_single() && instance.has_realized_data();
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");
  GeometrySet instance = params.extract_input<GeometrySet>("Instance");
  instance.ensure_owns_direct_data();
  const AnonymousAttributePropagationInfo &propagation_info = params.get_output_propagation_info("Instances");

  const Field<bool> selection_field = params.get_input<Field<bool>>("Selection");
  const Field<bool> pick_instance_field = params.get_input<Field<bool>>("Pick Instance");
  const Field<int> indices_field = params.get_input<Field<int>>("Instance Index");
  const Field<math::Quaternion> rotations_field = params.get_input<Field<math::Quaternion>>("Rotation");
  const Field<float3> scales_field = params.get_input<Field<float3>>("Scale");

  std::atomic<bool> has_skiped_realized_instances = false;

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    /* It's important not to invalidate the existing #InstancesComponent because it owns references
     * to other geometry sets that are processed by this node. */
    InstancesComponent &instances_component = geometry_set.get_component_for_write<InstancesComponent>();
    bke::Instances *dst_instances = instances_component.get_for_write();
    if (dst_instances == nullptr) {
      dst_instances = new bke::Instances();
      instances_component.replace(dst_instances);
    }

    const static std::array<GeometryComponent::Type, 3> types = {GeometryComponent::Type::Mesh, GeometryComponent::Type::PointCloud, GeometryComponent::Type::Curve};

    Map<AttributeIDRef, AttributeKind> attributes_to_propagate;
    geometry_set.gather_attributes_for_propagation(types, GeometryComponent::Type::Instance, false, propagation_info, attributes_to_propagate);
    attributes_to_propagate.remove("position");
    attributes_to_propagate.remove(".reference_index");

    for (const GeometryComponent::Type type : types) {
      if (geometry_set.has(type)) {
        const GeometryComponent &component = *geometry_set.get_component(type);
        if (add_instances_from_points(component, instance, attributes_to_propagate, selection_field, pick_instance_field, indices_field, rotations_field, scales_field, *dst_instances)) {
          has_skiped_realized_instances = true;
        }
      }
    }
    if (geometry_set.has_grease_pencil()) {
      using namespace bke::greasepencil;
      /*
      const GreasePencil &grease_pencil = *geometry_set.get_grease_pencil();
      for (const int layer_index : grease_pencil.layers().index_range()) {
        const Drawing *drawing = grease_pencil.get_eval_drawing(*grease_pencil.layer(layer_index));
        if (drawing == nullptr) {
          continue;
        }
        const bke::CurvesGeometry &src_curves = drawing->strokes();
        if (src_curves.curves_num() == 0) {
          /* Add an empty reference so the number of layers and instances match.
           * This makes it easy to reconstruct the layers afterwards and keep their attributes.
           * Although in this particular case we don't propagate the attributes. */ /*
          const int handle = dst_instances->add_reference(bke::InstanceReference());
          dst_instances->add_instance(handle, float4x4::identity());
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
        const int handle = dst_instances->add_reference(bke::InstanceReference{temp_set});
        dst_instances->add_instance(handle, float4x4::identity());
      }
      if (geometry_set.has_instances()) {
        GeometrySet::propagate_attributes_from_layer_to_instances(
            geometry_set.get_grease_pencil()->attributes(),
            geometry_set.get_instances_for_write()->attributes_for_write(),
            propagation_info);
      }
      geometry_set.replace_grease_pencil(nullptr);
      
      */
    }
    geometry_set.remove_geometry_during_modify();
  });

  if (has_skiped_realized_instances) {
    params.error_message_add(NodeWarningType::Info, TIP_("Realized geometry is not used when pick instances is true"));
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
