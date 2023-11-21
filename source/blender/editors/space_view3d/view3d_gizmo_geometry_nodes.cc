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

struct SocketItem {
  const bNodeSocket &socket;
  int elem_index = -1;
};

struct FloatValuePath {
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

struct GizmoFloatVariable {
  FloatValuePath path;
  float derivative;
};

static std::optional<float> compute_derivative(
    const NodesModifierData &nmd, const nodes::gizmos::GlobalGizmoSource &gizmo_source)
{
  float derivative = 1.0f;
  for (const nodes::gizmos::PropagationPath::PathElem &path_elem :
       gizmo_source.propagation_path.path)
  {
    geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(
        path_elem.compute_context->hash());
    tree_log.ensure_socket_values();

    const bNode &path_node = *path_elem.node;
    switch (path_node.type) {
      case SH_NODE_MATH: {
        const int mode = path_node.custom1;
        const bNodeSocket &second_input_socket = path_elem.node->input_socket(1);
        const float second_value =
            tree_log.find_primitive_socket_value<float>(second_input_socket).value_or(0.0f);
        switch (mode) {
          case NODE_MATH_MULTIPLY: {
            derivative = safe_divide(derivative, second_value);
            break;
          }
          case NODE_MATH_DIVIDE: {
            derivative *= second_value;
            break;
          }
          case NODE_MATH_RADIANS: {
            derivative = RAD2DEG(derivative);
            break;
          }
          case NODE_MATH_DEGREES: {
            derivative = DEG2RAD(derivative);
            break;
          }
          default:
            return std::nullopt;
        }
        break;
      }
      case SH_NODE_VECTOR_MATH: {
        const int mode = path_node.custom1;
        const bNodeSocket &second_input_socket = path_elem.node->input_socket(1);
        const bNodeSocket &scale_input_socket = path_elem.node->input_by_identifier("Scale");
        switch (mode) {
          case NODE_VECTOR_MATH_MULTIPLY: {
            const float factor = tree_log.find_primitive_socket_value<float3>(second_input_socket)
                                     .value_or(float3{0, 0, 0})[*path_elem.elem.index];
            derivative = safe_divide(derivative, factor);
            break;
          }
          case NODE_VECTOR_MATH_DIVIDE: {
            const float divisor = tree_log.find_primitive_socket_value<float3>(second_input_socket)
                                      .value_or(float3{0, 0, 0})[*path_elem.elem.index];
            derivative *= divisor;
            break;
          }
          case NODE_VECTOR_MATH_SCALE: {
            const float scale =
                tree_log.find_primitive_socket_value<float>(scale_input_socket).value_or(0.0f);
            derivative = safe_divide(derivative, scale);
            break;
          }
        }
        break;
      }
      case SH_NODE_MAP_RANGE: {
        const eCustomDataType data_type = eCustomDataType(
            static_cast<NodeMapRange *>(path_node.storage)->data_type);
        switch (data_type) {
          case CD_PROP_FLOAT: {
            const float from_min = tree_log
                                       .find_primitive_socket_value<float>(
                                           path_node.input_by_identifier("From Min"))
                                       .value_or(0.0f);
            const float from_max = tree_log
                                       .find_primitive_socket_value<float>(
                                           path_node.input_by_identifier("From Max"))
                                       .value_or(0.0f);
            const float to_min = tree_log
                                     .find_primitive_socket_value<float>(
                                         path_node.input_by_identifier("To Min"))
                                     .value_or(0.0f);
            const float to_max = tree_log
                                     .find_primitive_socket_value<float>(
                                         path_node.input_by_identifier("To Max"))
                                     .value_or(0.0f);
            derivative *= safe_divide(from_max - from_min, to_max - to_min);
            break;
          }
          case CD_PROP_FLOAT3: {
            const int index = *path_elem.elem.index;
            const float from_min = tree_log
                                       .find_primitive_socket_value<float3>(
                                           path_node.input_by_identifier("From_Min_FLOAT3"))
                                       .value_or(float3{0, 0, 0})[index];
            const float from_max = tree_log
                                       .find_primitive_socket_value<float3>(
                                           path_node.input_by_identifier("From_Max_FLOAT3"))
                                       .value_or(float3{0, 0, 0})[index];
            const float to_min = tree_log
                                     .find_primitive_socket_value<float3>(
                                         path_node.input_by_identifier("To_Min_FLOAT3"))
                                     .value_or(float3{0, 0, 0})[index];
            const float to_max = tree_log
                                     .find_primitive_socket_value<float3>(
                                         path_node.input_by_identifier("To_Max_FLOAT3"))
                                     .value_or(float3{0, 0, 0})[index];
            derivative *= safe_divide(from_max - from_min, to_max - to_min);
            break;
          }
          default:
            return std::nullopt;
        }
        break;
      }
      default: {
        return std::nullopt;
      }
    }
  }
  if (derivative == 0.0f) {
    return std::nullopt;
  }
  return derivative;
}

static std::optional<FloatValuePath> get_float_value_path(
    const nodes::gizmos::GlobalGizmoSource &gizmo_source,
    const Object &object,
    const NodesModifierData &nmd)
{
  if (const auto *ref = std::get_if<nodes::gizmos::InputSocketRef>(&gizmo_source.source)) {
    const bNodeTree &tree = ref->input_socket->owner_tree();
    ID *id = const_cast<ID *>(&tree.id);
    FloatValuePath value_path;
    value_path.owner = RNA_pointer_create(
        id, &RNA_NodeSocket, const_cast<bNodeSocket *>(ref->input_socket));
    value_path.property = RNA_struct_find_property(&value_path.owner, "default_value");
    value_path.index = ref->elem.index;
    return value_path;
  }
  if (const auto *ref = std::get_if<nodes::gizmos::ValueNodeRef>(&gizmo_source.source)) {
    const bNodeTree &tree = ref->value_node->owner_tree();
    ID *id = const_cast<ID *>(&tree.id);
    switch (ref->value_node->type) {
      case SH_NODE_VALUE: {
        FloatValuePath value_path;
        value_path.owner = RNA_pointer_create(
            id, &RNA_NodeSocket, const_cast<bNodeSocket *>(&ref->value_node->output_socket(0)));
        value_path.property = RNA_struct_find_property(&value_path.owner, "default_value");
        return value_path;
      }
      case FN_NODE_INPUT_VECTOR: {
        FloatValuePath value_path;
        value_path.owner = RNA_pointer_create(id, &RNA_Node, const_cast<bNode *>(ref->value_node));
        value_path.property = RNA_struct_find_property(&value_path.owner, "vector");
        value_path.index = *ref->elem.index;
        return value_path;
      }
      case FN_NODE_INPUT_INT: {
        FloatValuePath value_path;
        value_path.owner = RNA_pointer_create(id, &RNA_Node, const_cast<bNode *>(ref->value_node));
        value_path.property = RNA_struct_find_property(&value_path.owner, "integer");
        return value_path;
      }
    }
  }
  if (const auto *ref = std::get_if<nodes::gizmos::GroupInputRef>(&gizmo_source.source)) {
    const bNodeTree &ntree = *nmd.node_group;
    const StringRefNull input_identifier = ntree.interface_inputs()[ref->input_index]->identifier;
    IDProperty *id_property = IDP_GetPropertyFromGroup(nmd.settings.properties,
                                                       input_identifier.c_str());
    if (id_property == nullptr) {
      return std::nullopt;
    }
    FloatValuePath value_path;
    value_path.owner = RNA_pointer_create(
        const_cast<ID *>(&object.id), &RNA_NodesModifier, const_cast<NodesModifierData *>(&nmd));
    value_path.property = reinterpret_cast<PropertyRNA *>(id_property);
    value_path.index = ref->elem.index;
    return value_path;
  }
  return std::nullopt;
}

static Vector<GizmoFloatVariable> find_gizmo_float_value_paths(
    const bNode &gizmo_node,
    const ComputeContext &compute_context,
    const Object &object,
    const NodesModifierData &nmd)
{
  Vector<nodes::gizmos::GlobalGizmoSource> gizmo_sources =
      nodes::gizmos::find_global_gizmo_sources(compute_context, gizmo_node);
  Vector<GizmoFloatVariable> variables;
  for (const nodes::gizmos::GlobalGizmoSource &gizmo_source : gizmo_sources) {
    const std::optional<float> derivative = compute_derivative(nmd, gizmo_source);
    if (!derivative) {
      continue;
    }
    if (std::optional<FloatValuePath> value_path = get_float_value_path(gizmo_source, object, nmd))
    {
      variables.append({std::move(*value_path), *derivative});
    }
  }
  return variables;
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

static const float4x4 *find_direct_gizmo_transform(const bke::GeometrySet &geometry,
                                                   const ComputeContextHash &compute_context_hash,
                                                   const int gizmo_node_identifier)
{
  if (const auto *edit_data_component = geometry.get_component<bke::GeometryComponentEditData>()) {
    if (const float4x4 *m = edit_data_component->gizmo_transforms_.lookup_ptr(
            {compute_context_hash, gizmo_node_identifier}))
    {
      return m;
    }
  }
  return nullptr;
}

static bool has_nested_gizmo_transform(const bke::GeometrySet &geometry,
                                       const ComputeContextHash &compute_context_hash,
                                       const int gizmo_node_identifier)
{
  if (find_direct_gizmo_transform(geometry, compute_context_hash, gizmo_node_identifier)) {
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
    if (has_nested_gizmo_transform(
            reference_geometry, compute_context_hash, gizmo_node_identifier)) {
      return true;
    }
  }
  return false;
}

static std::optional<float4x4> find_gizmo_crazy_space_transform_recursive(
    const bke::GeometrySet &geometry,
    const ComputeContextHash &compute_context_hash,
    const int gizmo_node_identifier,
    const float4x4 &transform)
{
  if (const float4x4 *m = find_direct_gizmo_transform(
          geometry, compute_context_hash, gizmo_node_identifier))
  {
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
    if (has_nested_gizmo_transform(
            reference_geometry, compute_context_hash, gizmo_node_identifier)) {
      const int index = handles.first_index_try(reference_i);
      if (index >= 0) {
        const float4x4 sub_transform = transform * transforms[index];
        if (const std::optional<float4x4> m = find_gizmo_crazy_space_transform_recursive(
                reference_geometry, compute_context_hash, gizmo_node_identifier, sub_transform))
        {
          return *m;
        }
      }
    }
  }
  return std::nullopt;
}

static std::optional<float4x4> find_gizmo_crazy_space_transform(
    const bke::GeometrySet &geometry,
    const ComputeContextHash &compute_context_hash,
    const int gizmo_node_identifier)
{
  const float4x4 identity = float4x4::identity();
  return find_gizmo_crazy_space_transform_recursive(
      geometry, compute_context_hash, gizmo_node_identifier, identity);
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

  Map<bke::GeoNodesGizmoID, std::unique_ptr<NodeGizmoData>> new_gizmo_by_node;

  nodes::gizmos::foreach_active_gizmo(
      *ob_orig,
      nmd,
      *wm,
      [&](const ComputeContext &compute_context, const bNode &gizmo_node_const) {
        if (new_gizmo_by_node.contains({compute_context.hash(), gizmo_node_const.identifier})) {
          /* Already handled. */
          return;
        }

        bNode &gizmo_node = const_cast<bNode &>(gizmo_node_const);

        geo_eval_log::GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(
            compute_context.hash());
        tree_log.ensure_socket_values();

        bNodeSocket &position_input = gizmo_node.input_socket(1);
        bNodeSocket &direction_input = gizmo_node.input_socket(2);

        const std::optional<float3> position_opt = tree_log.find_primitive_socket_value<float3>(
            position_input);
        const std::optional<float3> direction_opt = tree_log.find_primitive_socket_value<float3>(
            direction_input);
        if (!position_opt || !direction_opt) {
          /* TODO: Add some safety measure to make sure that this does not needlessly cause updates
           * all the time. */
          DEG_id_tag_update(&ob_orig->id, ID_RECALC_GEOMETRY);
          WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
          return;
        }

        const float3 position = *position_opt;
        const float3 direction = math::normalize(*direction_opt);
        if (math::is_zero(direction)) {
          return;
        }

        Vector<GizmoFloatVariable> variables = find_gizmo_float_value_paths(
            gizmo_node, compute_context, *ob_orig, nmd);
        /* The gizmo should be drawn even if there is nothing linked to the value input, because
         * that results in a better initial user experience. */
        if (variables.is_empty() && gizmo_node.input_socket(0).is_directly_linked()) {
          return;
        }

        std::optional<std::unique_ptr<NodeGizmoData>> old_node_gizmo_data =
            gzgroup_data->gizmo_by_node.pop_try({compute_context.hash(), gizmo_node.identifier});

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
              return;
            }
          }
          gz->flag |= WM_GIZMO_NEEDS_UNDO;
          node_gizmo_data->gizmo = gz;
        }

        wmGizmo *gz = node_gizmo_data->gizmo;

        const ThemeColorID color_id = get_gizmo_theme_color_id(gizmo_node);
        UI_GetThemeColor3fv(color_id, gz->color);
        UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);

        if (gizmo_node.type == GEO_NODE_GIZMO_ARROW) {
          static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_ARROW) ==
                        int(ED_GIZMO_ARROW_STYLE_NORMAL));
          static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_BOX) == int(ED_GIZMO_ARROW_STYLE_BOX));
          static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_CROSS) ==
                        int(ED_GIZMO_ARROW_STYLE_CROSS));
          static_assert(int(GEO_NODE_ARROW_GIZMO_DRAW_STYLE_PLANE) ==
                        int(ED_GIZMO_ARROW_STYLE_PLANE));
          RNA_enum_set(
              gz->ptr,
              "draw_style",
              static_cast<const NodeGeometryArrowGizmo *>(gizmo_node.storage)->draw_style);
        }

        const bool is_interacting = gz->interaction_data != nullptr;

        struct UserData {
          bContext *C;
          Vector<float> initial_values;
          Vector<GizmoFloatVariable> variables;
          float gizmo_value = 0.0f;
        };

        wmGizmoProperty *gz_prop = WM_gizmo_target_property_find(gz, "offset");
        if (is_interacting) {
          UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
          for (const int i : user_data->variables.index_range()) {
            const GizmoFloatVariable &new_variable = variables[i];
            GizmoFloatVariable &variable = user_data->variables[i];
            variable.derivative = new_variable.derivative;
          }
        }
        else {
          float4x4 crazy_space_transform = float4x4::identity();
          if (const std::optional<float4x4> m = find_gizmo_crazy_space_transform(
                  geometry, compute_context.hash(), gizmo_node.identifier))
          {
            crazy_space_transform = *m;
          }

          const math::Quaternion rotation = math::from_vector(
              math::normalize(direction),
              gz->type->idname == StringRef("GIZMO_GT_arrow_3d") ? math::AxisSigned::Z_NEG :
                                                                   math::AxisSigned::Z_POS,
              math::Axis::X);
          float4x4 mat = math::from_rotation<float4x4>(rotation);
          mat.location() = position;
          mat = float4x4(ob_orig->object_to_world) * crazy_space_transform * mat;
          copy_m4_m4(node_gizmo_data->gizmo->matrix_basis, mat.ptr());

          UserData *user_data = MEM_new<UserData>(__func__);
          /* The code that calls `value_set_fn` has the context, but its not passed into the
           * callback currently. */
          user_data->C = const_cast<bContext *>(C);
          for (GizmoFloatVariable &variable : variables) {
            user_data->initial_values.append(variable.path.get());
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
            for (const int i : user_data->variables.index_range()) {
              GizmoFloatVariable &variable = user_data->variables[i];
              const float initial_value = user_data->initial_values[i];
              const float new_value = initial_value +
                                      new_gizmo_value_clamped * variable.derivative;
              const float new_value_clamped = variable.path.clamp(new_value);
              new_gizmo_value_clamped = (new_value_clamped - initial_value) / variable.derivative;
            }
            user_data->gizmo_value = new_gizmo_value_clamped;
            for (const int i : user_data->variables.index_range()) {
              GizmoFloatVariable &variable = user_data->variables[i];
              const float initial_value = user_data->initial_values[i];
              const float new_value = initial_value +
                                      new_gizmo_value_clamped * variable.derivative;
              variable.path.set_and_update(user_data->C, new_value);
            }
          };
          params.value_get_fn =
              [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
                UserData *user_data = static_cast<UserData *>(gz_prop->custom_func.user_data);
                *static_cast<float *>(value_ptr) = user_data->gizmo_value;
              };

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
