/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include <iostream>

#include "WM_api.hh"
#include "WM_types.hh"

#include "DNA_modifier_types.h"
#include "DNA_node_types.h"

#include "BKE_compute_contexts.hh"
#include "BKE_context.hh"
#include "BKE_geometry_set.hh"
#include "BKE_geometry_set_instances.hh"
#include "BKE_idprop.hh"
#include "BKE_instances.hh"
#include "BKE_modifier.hh"
#include "BKE_node_runtime.hh"
#include "BKE_object.hh"

#include "BLI_math_base_safe.h"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix.hh"
#include "BLI_math_rotation.h"
#include "BLI_math_rotation.hh"
#include "BLI_math_vector.h"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "NOD_geometry_nodes_gizmos.hh"
#include "NOD_geometry_nodes_log.hh"

#include "MOD_nodes.hh"

#include "UI_resources.hh"

#include "ED_gizmo_library.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "view3d_intern.h" /* own include */

namespace blender::ed::view3d {
namespace geo_eval_log = nodes::geo_eval_log;

struct NodeGizmoData {
  wmGizmo *gizmo;
};

struct GeometryNodesGizmoGroup {
  Map<bke::GeoNodesGizmoID, std::unique_ptr<NodeGizmoData>> gizmo_by_node;
};

/**
 * A float property that is controlled by a gizmo.
 */
struct FloatTargetProperty {
  PointerRNA owner;
  PropertyRNA *property;
  std::optional<int> index;

  float get()
  {
    const PropertyType prop_type = RNA_property_type(this->property);
    const bool prop_is_array = RNA_property_array_check(this->property);
    if (prop_is_array) {
      BLI_assert(this->index.has_value());
      if (prop_type == PROP_FLOAT) {
        return RNA_property_float_get_index(&this->owner, this->property, *this->index);
      }
      else if (prop_type == PROP_INT) {
        return RNA_property_int_get_index(&this->owner, this->property, *this->index);
      }
    }
    else {
      if (prop_type == PROP_FLOAT) {
        return RNA_property_float_get(&this->owner, this->property);
      }
      else if (prop_type == PROP_INT) {
        return RNA_property_int_get(&this->owner, this->property);
      }
    }
    BLI_assert_unreachable();
    return 0.0f;
  }

  float clamp(const float value)
  {
    const PropertyType prop_type = RNA_property_type(this->property);
    float softmin = -FLT_MAX;
    float softmax = FLT_MAX;
    if (prop_type == PROP_FLOAT) {
      float step, precision;
      RNA_property_float_ui_range(
          &this->owner, this->property, &softmin, &softmax, &step, &precision);
    }
    else if (prop_type == PROP_INT) {
      int softmin_i, softmax_i, step;
      RNA_property_int_ui_range(&this->owner, this->property, &softmin_i, &softmax_i, &step);
      softmin = float(softmin_i);
      softmax = float(softmax_i);
    }
    const float value_clamped = std::clamp(value, softmin, softmax);
    return value_clamped;
  }

  void set_and_update(bContext *C, const float value)
  {
    const PropertyType prop_type = RNA_property_type(this->property);
    const bool prop_is_array = RNA_property_array_check(this->property);

    if (prop_is_array) {
      BLI_assert(this->index.has_value());
      if (prop_type == PROP_FLOAT) {
        RNA_property_float_set_index(&this->owner, this->property, *this->index, value);
      }
      else if (prop_type == PROP_INT) {
        RNA_property_int_set_index(&this->owner, this->property, *this->index, value);
      }
    }
    else {
      if (prop_type == PROP_FLOAT) {
        RNA_property_float_set(&this->owner, this->property, value);
      }
      else if (prop_type == PROP_INT) {
        RNA_property_int_set(&this->owner, this->property, value);
      }
    }
    RNA_property_update(C, &this->owner, this->property);
    if (Object *object = CTX_data_active_object(C)) {
      /* The recalc is necessary so that the gizmo positions are updated even if they don't affect
       * the final output currently. */
      DEG_id_tag_update(&object->id, ID_RECALC_GEOMETRY);
    }
  }
};

static std::optional<float> apply_reverse_function_to_factor(const float target_factor,
                                                             const NodesModifierData &nmd,
                                                             const ComputeContext &compute_context,
                                                             const bNode &node,
                                                             const nodes::gizmos::ValueElem &elem)
{
  geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(compute_context.hash());
  tree_log.ensure_socket_values();

  switch (node.type) {
    case SH_NODE_MATH: {
      const int mode = node.custom1;
      const bNodeSocket &second_input_socket = node.input_socket(1);
      const float second_value =
          tree_log.find_primitive_socket_value<float>(second_input_socket).value_or(0.0f);
      switch (mode) {
        case NODE_MATH_MULTIPLY: {
          return safe_divide(target_factor, second_value);
        }
        case NODE_MATH_DIVIDE: {
          return target_factor * second_value;
        }
        case NODE_MATH_RADIANS: {
          return RAD2DEG(target_factor);
        }
        case NODE_MATH_DEGREES: {
          return DEG2RAD(target_factor);
        }
        default:
          return std::nullopt;
      }
      break;
    }
    case SH_NODE_VECTOR_MATH: {
      const int mode = node.custom1;
      const bNodeSocket &second_input_socket = node.input_socket(1);
      const bNodeSocket &scale_input_socket = node.input_by_identifier("Scale");
      switch (mode) {
        case NODE_VECTOR_MATH_MULTIPLY: {
          const float factor = tree_log.find_primitive_socket_value<float3>(second_input_socket)
                                   .value_or(float3{0, 0, 0})[*elem.index];
          return safe_divide(target_factor, factor);
        }
        case NODE_VECTOR_MATH_DIVIDE: {
          const float divisor = tree_log.find_primitive_socket_value<float3>(second_input_socket)
                                    .value_or(float3{0, 0, 0})[*elem.index];
          return target_factor * divisor;
        }
        case NODE_VECTOR_MATH_SCALE: {
          const float scale =
              tree_log.find_primitive_socket_value<float>(scale_input_socket).value_or(0.0f);
          return safe_divide(target_factor, scale);
        }
      }
      break;
    }
    case SH_NODE_MAP_RANGE: {
      const eCustomDataType data_type = eCustomDataType(
          static_cast<NodeMapRange *>(node.storage)->data_type);
      switch (data_type) {
        case CD_PROP_FLOAT: {
          const float from_min = tree_log
                                     .find_primitive_socket_value<float>(
                                         node.input_by_identifier("From Min"))
                                     .value_or(0.0f);
          const float from_max = tree_log
                                     .find_primitive_socket_value<float>(
                                         node.input_by_identifier("From Max"))
                                     .value_or(0.0f);
          const float to_min = tree_log
                                   .find_primitive_socket_value<float>(
                                       node.input_by_identifier("To Min"))
                                   .value_or(0.0f);
          const float to_max = tree_log
                                   .find_primitive_socket_value<float>(
                                       node.input_by_identifier("To Max"))
                                   .value_or(0.0f);
          return target_factor * safe_divide(from_max - from_min, to_max - to_min);
        }
        case CD_PROP_FLOAT3: {
          const int index = *elem.index;
          const float from_min = tree_log
                                     .find_primitive_socket_value<float3>(
                                         node.input_by_identifier("From_Min_FLOAT3"))
                                     .value_or(float3{0, 0, 0})[index];
          const float from_max = tree_log
                                     .find_primitive_socket_value<float3>(
                                         node.input_by_identifier("From_Max_FLOAT3"))
                                     .value_or(float3{0, 0, 0})[index];
          const float to_min = tree_log
                                   .find_primitive_socket_value<float3>(
                                       node.input_by_identifier("To_Min_FLOAT3"))
                                   .value_or(float3{0, 0, 0})[index];
          const float to_max = tree_log
                                   .find_primitive_socket_value<float3>(
                                       node.input_by_identifier("To_Max_FLOAT3"))
                                   .value_or(float3{0, 0, 0})[index];
          return target_factor * safe_divide(from_max - from_min, to_max - to_min);
        }
        default:
          return std::nullopt;
      }
      break;
    }
  }
  return std::nullopt;
}

/**
 * Computes the factor that is applied when gizmo-movements are converted to value-changes.
 * For example, a factor of 10 means that when the gizmo is moved by 2 units, the target value
 * changes by 20.
 */
static std::optional<float> compute_target_factor(
    const NodesModifierData &nmd, const nodes::gizmos::PropagationPath &propagation_path)
{
  /* The default factor is 1. */
  float target_factor = 1.0f;
  /* Check the nodes that propagated the gizmo target from right-to-left. */
  for (const nodes::gizmos::PropagationPath::PathElem &path_elem : propagation_path.path) {
    target_factor = apply_reverse_function_to_factor(target_factor,
                                                     nmd,
                                                     *path_elem.compute_context,
                                                     *path_elem.node,
                                                     path_elem.elem)
                        .value_or(0.0f);
    if (target_factor == 0.0f) {
      return std::nullopt;
    }
  }
  return target_factor;
}

/**
 * Get an actual property accessor for the value controlled by the gizmo.
 */
static std::optional<FloatTargetProperty> get_float_target_property(
    const nodes::gizmos::PropagatedGizmoTarget &gizmo_target,
    const Object &object,
    const NodesModifierData &nmd)
{
  if (const auto *ref = std::get_if<nodes::gizmos::InputSocketRef>(&gizmo_target.target)) {
    /* The gizmo controls the value of an input socket. */
    const bNodeTree &tree = ref->input_socket->owner_tree();
    ID *id = const_cast<ID *>(&tree.id);
    FloatTargetProperty target_property;
    target_property.owner = RNA_pointer_create(
        id, &RNA_NodeSocket, const_cast<bNodeSocket *>(ref->input_socket));
    target_property.property = RNA_struct_find_property(&target_property.owner, "default_value");
    target_property.index = ref->elem.index;
    return target_property;
  }
  if (const auto *ref = std::get_if<nodes::gizmos::ValueNodeRef>(&gizmo_target.target)) {
    /* The gizmo controls the value of a value node. */
    const bNodeTree &tree = ref->value_node->owner_tree();
    ID *id = const_cast<ID *>(&tree.id);
    switch (ref->value_node->type) {
      case SH_NODE_VALUE: {
        FloatTargetProperty target_property;
        target_property.owner = RNA_pointer_create(
            id, &RNA_NodeSocket, const_cast<bNodeSocket *>(&ref->value_node->output_socket(0)));
        target_property.property = RNA_struct_find_property(&target_property.owner,
                                                            "default_value");
        return target_property;
      }
      case FN_NODE_INPUT_VECTOR: {
        FloatTargetProperty target_property;
        target_property.owner = RNA_pointer_create(
            id, &RNA_Node, const_cast<bNode *>(ref->value_node));
        target_property.property = RNA_struct_find_property(&target_property.owner, "vector");
        target_property.index = *ref->elem.index;
        return target_property;
      }
      case FN_NODE_INPUT_INT: {
        FloatTargetProperty target_property;
        target_property.owner = RNA_pointer_create(
            id, &RNA_Node, const_cast<bNode *>(ref->value_node));
        target_property.property = RNA_struct_find_property(&target_property.owner, "integer");
        return target_property;
      }
    }
  }
  if (const auto *ref = std::get_if<nodes::gizmos::GroupInputRef>(&gizmo_target.target)) {
    /* The gizmo controls a value stored in the modifier. */
    const bNodeTree &ntree = *nmd.node_group;
    const StringRefNull input_identifier = ntree.interface_inputs()[ref->input_index]->identifier;
    IDProperty *id_property = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                       input_identifier.c_str());
    if (id_property == nullptr) {
      return std::nullopt;
    }
    FloatTargetProperty target_property;
    target_property.owner = RNA_pointer_create(
        const_cast<ID *>(&object.id), &RNA_NodesModifier, const_cast<NodesModifierData *>(&nmd));
    target_property.property = reinterpret_cast<PropertyRNA *>(id_property);
    target_property.index = ref->elem.index;
    return target_property;
  }
  return std::nullopt;
}

/**
 * The target property with a factor that determines how gizmo-changes affect changes in the
 * controlled property.
 */
struct DerivedFloatTargetProperty {
  FloatTargetProperty property;
  float factor;
};

/**
 * Find all properties controlled by a specific gizmo node.
 */
static Vector<DerivedFloatTargetProperty> find_gizmo_target_properties(
    const bNode &gizmo_node,
    const ComputeContext &compute_context,
    const Object &object,
    const NodesModifierData &nmd)
{
  Vector<nodes::gizmos::PropagatedGizmoTarget> gizmo_targets =
      nodes::gizmos::find_propagated_gizmo_targets(compute_context, gizmo_node);
  Vector<DerivedFloatTargetProperty> target_properties;
  for (const nodes::gizmos::PropagatedGizmoTarget &gizmo_target : gizmo_targets) {
    const std::optional<float> target_factor = compute_target_factor(
        nmd, gizmo_target.propagation_path);
    if (!target_factor) {
      continue;
    }
    if (std::optional<FloatTargetProperty> target_property = get_float_target_property(
            gizmo_target, object, nmd))
    {
      target_properties.append({std::move(*target_property), *target_factor});
    }
  }
  return target_properties;
}

static bool WIDGETGROUP_geometry_nodes_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  if (Object *object = CTX_data_active_object(C)) {
    if (ModifierData *md = BKE_object_active_modifier(object)) {
      if (md->type == eModifierType_Nodes) {
        NodesModifierData *nmd = reinterpret_cast<NodesModifierData *>(md);
        return nmd->node_group != nullptr;
      }
    }
  }
  return false;
}

static void WIDGETGROUP_geometry_nodes_setup(const bContext * /*C*/, wmGizmoGroup *gzgroup)
{
  GeometryNodesGizmoGroup *gzgroup_data = MEM_new<GeometryNodesGizmoGroup>(__func__);
  gzgroup->customdata = gzgroup_data;
  gzgroup->customdata_free = [](void *data) {
    auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(data);
    MEM_delete(gzgroup_data);
  };
}

static GeometryNodeGizmoColor get_gizmo_color_id(const bNode &gizmo_node)
{
  switch (gizmo_node.type) {
    case GEO_NODE_GIZMO_ARROW:
      return GeometryNodeGizmoColor(
          static_cast<const NodeGeometryArrowGizmo *>(gizmo_node.storage)->color_id);
    case GEO_NODE_GIZMO_DIAL:
      return GeometryNodeGizmoColor(
          static_cast<const NodeGeometryDialGizmo *>(gizmo_node.storage)->color_id);
    default:
      return GEO_NODE_GIZMO_COLOR_PRIMARY;
  }
}

static ThemeColorID get_gizmo_theme_color_id(const bNode &gizmo_node)
{
  switch (get_gizmo_color_id(gizmo_node)) {
    case GEO_NODE_GIZMO_COLOR_PRIMARY:
      return TH_GIZMO_PRIMARY;
    case GEO_NODE_GIZMO_COLOR_SECONDARY:
      return TH_GIZMO_SECONDARY;
    case GEO_NODE_GIZMO_COLOR_X:
      return TH_AXIS_X;
    case GEO_NODE_GIZMO_COLOR_Y:
      return TH_AXIS_Y;
    case GEO_NODE_GIZMO_COLOR_Z:
      return TH_AXIS_Z;
  }
  return TH_GIZMO_PRIMARY;
}

/** Finds the gizmo transform stored directly in the geometry, ignoring the instances. */
static const float4x4 *find_direct_gizmo_transform(const bke::GeometrySet &geometry,
                                                   const bke::GeoNodesGizmoID &gizmo_id)
{
  if (const auto *edit_data_component = geometry.get_component<bke::GeometryComponentEditData>()) {
    if (const float4x4 *m = edit_data_component->gizmo_transforms_.lookup_ptr(gizmo_id)) {
      return m;
    }
  }
  return nullptr;
}

/**
 * True, if the geometry contains a transform for the given gizmo. Also checks if all instances.
 */
static bool has_nested_gizmo_transform(const bke::GeometrySet &geometry,
                                       const bke::GeoNodesGizmoID &gizmo_id)
{
  if (find_direct_gizmo_transform(geometry, gizmo_id)) {
    return true;
  }
  if (!geometry.has_instances()) {
    return false;
  }
  const bke::Instances *instances = geometry.get_instances();
  for (const bke::InstanceReference &reference : instances->references()) {
    if (reference.type() != bke::InstanceReference::Type::GeometrySet) {
      continue;
    }
    const bke::GeometrySet &reference_geometry = reference.geometry_set();
    if (has_nested_gizmo_transform(reference_geometry, gizmo_id)) {
      return true;
    }
  }
  return false;
}

static std::optional<float4x4> find_gizmo_geometry_transform_recursive(
    const bke::GeometrySet &geometry,
    const bke::GeoNodesGizmoID &gizmo_id,
    const float4x4 &transform)
{
  if (const float4x4 *m = find_direct_gizmo_transform(geometry, gizmo_id)) {
    return transform * *m;
  }
  if (!geometry.has_instances()) {
    return std::nullopt;
  }
  const bke::Instances *instances = geometry.get_instances();
  const Span<bke::InstanceReference> references = instances->references();
  const Span<int> handles = instances->reference_handles();
  const Span<float4x4> transforms = instances->transforms();
  for (const int reference_i : references.index_range()) {
    const bke::InstanceReference &reference = references[reference_i];
    if (reference.type() != bke::InstanceReference::Type::GeometrySet) {
      continue;
    }
    const bke::GeometrySet &reference_geometry = reference.geometry_set();
    if (has_nested_gizmo_transform(reference_geometry, gizmo_id)) {
      const int index = handles.first_index_try(reference_i);
      if (index >= 0) {
        const float4x4 sub_transform = transform * transforms[index];
        if (const std::optional<float4x4> m = find_gizmo_geometry_transform_recursive(
                reference_geometry, gizmo_id, sub_transform))
        {
          return *m;
        }
      }
    }
  }
  return std::nullopt;
}

/**
 * Tries to find a transformation of the gizmo in the given geometry.
 */
static std::optional<float4x4> find_gizmo_geometry_transform(const bke::GeometrySet &geometry,
                                                             const bke::GeoNodesGizmoID &gizmo_id)
{
  const float4x4 identity = float4x4::identity();
  return find_gizmo_geometry_transform_recursive(geometry, gizmo_id, identity);
}

static float4x4 matrix_from_position_and_up_direction(const float3 &position,
                                                      const float3 &direction,
                                                      const math::AxisSigned direction_axis)
{
  BLI_assert(math::is_unit_scale(direction));
  math::Quaternion rotation;
  const float3 base_direction = math::to_vector<float3>(direction_axis);
  rotation_between_vecs_to_quat(&rotation.w, base_direction, direction);
  float4x4 mat = math::from_rotation<float4x4>(rotation);
  mat.location() = position;
  return mat;
}

static std::optional<float4x4> get_arrow_gizmo_base_transform(const bNode &gizmo_node,
                                                              geo_eval_log::GeoTreeLog &tree_log,
                                                              bool &r_missing_socket_logs)
{
  const bNodeSocket &position_input = gizmo_node.input_socket(1);
  const bNodeSocket &direction_input = gizmo_node.input_socket(2);
  const std::optional<float3> position_opt = tree_log.find_primitive_socket_value<float3>(
      position_input);
  const std::optional<float3> direction_opt = tree_log.find_primitive_socket_value<float3>(
      direction_input);
  if (!position_opt || !direction_opt) {
    r_missing_socket_logs = true;
    return std::nullopt;
  }
  const float3 position = *position_opt;
  const float3 direction = math::normalize(*direction_opt);
  if (math::is_zero(direction)) {
    return std::nullopt;
  }
  return matrix_from_position_and_up_direction(position, direction, math::AxisSigned::Z_POS);
}

static std::optional<float4x4> get_dial_gizmo_base_transform(const bNode &gizmo_node,
                                                             geo_eval_log::GeoTreeLog &tree_log,
                                                             bool &r_missing_socket_logs)
{
  const bNodeSocket &position_input = gizmo_node.input_socket(1);
  const bNodeSocket &direction_input = gizmo_node.input_socket(2);
  const std::optional<float3> position_opt = tree_log.find_primitive_socket_value<float3>(
      position_input);
  const std::optional<float3> direction_opt = tree_log.find_primitive_socket_value<float3>(
      direction_input);
  if (!position_opt || !direction_opt) {
    r_missing_socket_logs = true;
    return std::nullopt;
  }
  const float3 position = *position_opt;
  const float3 direction = math::normalize(*direction_opt);
  if (math::is_zero(direction)) {
    return std::nullopt;
  }
  return matrix_from_position_and_up_direction(position, direction, math::AxisSigned::Z_NEG);
}

static void WIDGETGROUP_geometry_nodes_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  auto *gzgroup_data = static_cast<GeometryNodesGizmoGroup *>(gzgroup->customdata);

  Object *ob_orig = CTX_data_active_object(C);
  const NodesModifierData &nmd = *reinterpret_cast<const NodesModifierData *>(
      BKE_object_active_modifier(ob_orig));
  if (!nmd.runtime->eval_log) {
    return;
  }

  const wmWindowManager *wm = CTX_wm_manager(C);
  if (wm == nullptr) {
    return;
  }
  Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  Object *ob_eval = DEG_get_evaluated_object(depsgraph, ob_orig);
  if (ob_eval == nullptr) {
    return;
  }

  const bke::GeometrySet geometry = bke::object_get_evaluated_geometry_set(*ob_eval);

  bNodeTree &ntree = *nmd.node_group;
  ntree.ensure_topology_cache();

  const float4x4 object_to_world{ob_orig->object_to_world};

  /* Rebuild the set of all active gizmos, reusing gizmo data from before if possible. */
  Map<bke::GeoNodesGizmoID, std::unique_ptr<NodeGizmoData>> new_gizmo_by_node;
  nodes::gizmos::foreach_active_gizmo(
      *ob_orig,
      nmd,
      *wm,
      [&](const ComputeContext &compute_context, const bNode &gizmo_node_const) {
        /* Need const-cast because we might actually want to modify data indirectly referenced by
         * the node. */
        bNode &gizmo_node = const_cast<bNode &>(gizmo_node_const);
        const bke::GeoNodesGizmoID gizmo_id = {compute_context.hash(),
                                               gizmo_node_const.identifier};
        if (new_gizmo_by_node.contains(gizmo_id)) {
          /* Already handled. */
          return;
        }

        std::optional<std::unique_ptr<NodeGizmoData>> old_node_gizmo_data =
            gzgroup_data->gizmo_by_node.pop_try(gizmo_id);
        /* While the user interacts with the gizmo, it should not be removed and its base transform
         * should not change. */
        const bool is_interacting = old_node_gizmo_data ?
                                        old_node_gizmo_data->get()->gizmo->interaction_data !=
                                            nullptr :
                                        false;

        const float4x4 geometry_transform =
            find_gizmo_geometry_transform(geometry, gizmo_id).value_or(float4x4::identity());

        geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(
            compute_context.hash());
        tree_log.ensure_socket_values();
        std::optional<float4x4> base_gizmo_transform;
        bool missing_socket_logs = false;
        switch (gizmo_node.type) {
          case GEO_NODE_GIZMO_ARROW: {
            base_gizmo_transform = get_arrow_gizmo_base_transform(
                gizmo_node, tree_log, missing_socket_logs);
            break;
          }
          case GEO_NODE_GIZMO_DIAL: {
            base_gizmo_transform = get_dial_gizmo_base_transform(
                gizmo_node, tree_log, missing_socket_logs);
            break;
          }
        }
        if (!base_gizmo_transform) {
          if (missing_socket_logs) {
            /* TODO: protect against repeated updates */
            DEG_id_tag_update(&ob_orig->id, ID_RECALC_GEOMETRY);
            WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
          }
          if (!is_interacting) {
            return;
          }
        }

        Vector<DerivedFloatTargetProperty> variables = find_gizmo_target_properties(
            gizmo_node, compute_context, *ob_orig, nmd);
        /* The gizmo should be drawn even if there is nothing linked to the value input, because
         * that results in a better initial user experience. It also should not be removed while
         * the user interacts with it. */
        if (variables.is_empty() && gizmo_node.input_socket(0).is_directly_linked() &&
            !is_interacting) {
          return;
        }

        std::unique_ptr<NodeGizmoData> node_gizmo_data;
        if (old_node_gizmo_data.has_value()) {
          node_gizmo_data = std::move(*old_node_gizmo_data);
        }
        else {
          node_gizmo_data = std::make_unique<NodeGizmoData>();
          wmGizmo *gz;
          switch (gizmo_node.type) {
            case GEO_NODE_GIZMO_ARROW: {
              gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, nullptr);
              WM_gizmo_set_line_width(gz, 1.0f);
              break;
            }
            case GEO_NODE_GIZMO_DIAL: {
              gz = WM_gizmo_new("GIZMO_GT_dial_3d", gzgroup, nullptr);
              WM_gizmo_set_flag(gz, WM_GIZMO_DRAW_VALUE, true);
              WM_gizmo_set_line_width(gz, 2.0f);
              break;
            }
            default: {
              BLI_assert_unreachable();
              return;
            }
          }
          gz->flag |= WM_GIZMO_NEEDS_UNDO;
          node_gizmo_data->gizmo = gz;
        }

        wmGizmo *gz = node_gizmo_data->gizmo;

        /* Update gizmo draw styles. */
        const ThemeColorID color_id = get_gizmo_theme_color_id(gizmo_node);
        UI_GetThemeColor3fv(color_id, gz->color);
        UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);
        switch (gizmo_node.type) {
          case GEO_NODE_GIZMO_ARROW: {
            /* Make sure the enum values are in sync. */
            static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_ARROW) ==
                          int(ED_GIZMO_ARROW_STYLE_NORMAL));
            static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_BOX) ==
                          int(ED_GIZMO_ARROW_STYLE_BOX));
            static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_CROSS) ==
                          int(ED_GIZMO_ARROW_STYLE_CROSS));
            static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_PLANE) ==
                          int(ED_GIZMO_ARROW_STYLE_PLANE));
            RNA_enum_set(
                gz->ptr,
                "draw_style",
                static_cast<const NodeGeometryArrowGizmo *>(gizmo_node.storage)->draw_style);
            break;
          }
          case GEO_NODE_GIZMO_DIAL: {
            int draw_options = RNA_enum_get(gz->ptr, "draw_options");
            SET_FLAG_FROM_TEST(draw_options, is_interacting, ED_GIZMO_DIAL_DRAW_FLAG_ANGLE_VALUE);
            RNA_enum_set(gz->ptr, "draw_options", draw_options);
            break;
          }
        }

        struct UserData {
          bContext *C;
          Vector<float> initial_values;
          Vector<DerivedFloatTargetProperty> variables;
          float gizmo_value = 0.0f;
        };

        wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(gz, "offset");
        if (is_interacting) {
          /* Don't update the base transform while interacting with the gizmo, so that its behavior
           * stays more predictable. */
          UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
          for (const int i : user_data->variables.index_range()) {
            const DerivedFloatTargetProperty &new_variable = variables[i];
            DerivedFloatTargetProperty &variable = user_data->variables[i];
            variable.factor = new_variable.factor;
          }
        }
        else {
          /* Update gizmo base transform. */
          const float4x4 current_gizmo_transform = object_to_world * geometry_transform *
                                                   *base_gizmo_transform;
          copy_m4_m4(node_gizmo_data->gizmo->matrix_basis, current_gizmo_transform.ptr());

          UserData *user_data = MEM_new<UserData>(__func__);
          /* The code that calls `value_set_fn` has the context, but its not passed into the
           * callback currently. */
          user_data->C = const_cast<bContext *>(C);
          for (DerivedFloatTargetProperty &variable : variables) {
            user_data->initial_values.append(variable.property.get());
          }
          user_data->variables = std::move(variables);

          wmGizmoPropertyFnParams params{};
          params.user_data = user_data;
          params.free_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop) {
            MEM_delete(static_cast<UserData *>(gz_prop->custom_func.user_data));
          };
          params.value_set_fn = [](const wmGizmo * /*gz*/,
                                   wmGizmoProperty *gz_prop,
                                   const void *value_ptr) {
            UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
            const float new_gizmo_value = *static_cast<const float *>(value_ptr);
            float new_gizmo_value_clamped = new_gizmo_value;
            /* First clamp the value based on all target properties. */
            for (const int i : user_data->variables.index_range()) {
              DerivedFloatTargetProperty &variable = user_data->variables[i];
              const float initial_value = user_data->initial_values[i];
              const float new_value = initial_value + new_gizmo_value_clamped * variable.factor;
              const float new_value_clamped = variable.property.clamp(new_value);
              new_gizmo_value_clamped = (new_value_clamped - initial_value) / variable.factor;
            }
            /* Now apply the same clamped value to all targets. */
            user_data->gizmo_value = new_gizmo_value_clamped;
            for (const int i : user_data->variables.index_range()) {
              DerivedFloatTargetProperty &variable = user_data->variables[i];
              const float initial_value = user_data->initial_values[i];
              const float new_value = initial_value + new_gizmo_value_clamped * variable.factor;
              variable.property.set_and_update(user_data->C, new_value);
            }
          };
          params.value_get_fn =
              [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
                UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
                *static_cast<float *>(value_ptr) = user_data->gizmo_value;
              };
          /* Free the old property data because it's replaced now. */
          if (gz_prop->custom_func.free_fn) {
            gz_prop->custom_func.free_fn(node_gizmo_data->gizmo, gz_prop);
          }
          WM_gizmo_target_property_def_func(node_gizmo_data->gizmo, "offset", &params);
        }

        new_gizmo_by_node.add_new({compute_context.hash(), gizmo_node.identifier},
                                  std::move(node_gizmo_data));
      });

  for (std::unique_ptr<NodeGizmoData> &node_gizmo_data : gzgroup_data->gizmo_by_node.values()) {
    WM_gizmo_unlink(&gzgroup->gizmos,
                    gzgroup->parent_gzmap,
                    node_gizmo_data->gizmo,
                    const_cast<bContext *>(C));
  }

  gzgroup_data->gizmo_by_node = std::move(new_gizmo_by_node);
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext * /*C*/,
                                                    wmGizmoGroup * /*gzgroup*/)
{
}

}  // namespace blender::ed::view3d

void VIEW3D_GGT_geometry_nodes(wmGizmoGroupType *gzgt)
{
  using namespace blender::ed::view3d;

  gzgt->name = "Geometry Nodes Widgets";
  gzgt->idname = "VIEW3D_GGT_geometry_nodes";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D);

  gzgt->poll = WIDGETGROUP_geometry_nodes_poll;
  gzgt->setup = WIDGETGROUP_geometry_nodes_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_geometry_nodes_refresh;
  gzgt->draw_prepare = WIDGETGROUP_geometry_nodes_draw_prepare;
}
