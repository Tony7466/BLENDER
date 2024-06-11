/* SPDX-FileCopyrightText: 2024 Blender Authors
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
#include "BKE_geometry_nodes_gizmos_transforms.hh"
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
#include "ED_node.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "view3d_intern.hh"

namespace blender::ed::view3d::geometry_nodes_gizmos {
namespace geo_eval_log = nodes::geo_eval_log;
using geo_eval_log::GeoTreeLog;

static bool gizmo_is_interacting(const wmGizmo &gizmo)
{
  return gizmo.interaction_data != nullptr;
}

static ThemeColorID get_gizmo_theme_color_id(const GeometryNodeGizmoColor color_id)
{
  switch (color_id) {
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

static void make_matrix_orthonormal_but_keep_z_axis(float4x4 &m)
{
  /* Without this, the gizmos may be skewed. */
  m.x_axis() = math::normalize(math::cross(m.y_axis(), m.z_axis()));
  m.y_axis() = math::normalize(math::cross(m.z_axis(), m.x_axis()));
  m.z_axis() = math::normalize(m.z_axis());
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

struct UpdateReport {
  bool missing_socket_logs = false;
  bool invalid_transform = false;
};

using ApplyChangeFn = std::function<void(
    StringRef socket_identifier, FunctionRef<void(bke::SocketValueVariant &value)> modify_value)>;

struct GizmosUpdateParams {
  /* Transform of the object and geometry that the gizmo belongs to. */
  float4x4 parent_transform;
  const bNode &gizmo_node;
  GeoTreeLog &tree_log;
  UpdateReport &r_report;
};

class NodeGizmos {
 public:
  ApplyChangeFn apply_change;

  virtual ~NodeGizmos() = default;

  virtual void create_gizmos(wmGizmoGroup &gzgroup) = 0;

  virtual void update_style(const bNode & /*gizmo_node*/) {}

  virtual void update(GizmosUpdateParams & /*params*/) {}

  virtual Vector<wmGizmo *> get_all_gizmos() = 0;
};

class LinearGizmo : public NodeGizmos {
 private:
  wmGizmo *gizmo_ = nullptr;

  struct EditData {
    float factor_from_transform = 1.0f;
    float current_value = 0.0f;
  } edit_data_;

 public:
  void create_gizmos(wmGizmoGroup &gzgroup) override
  {
    gizmo_ = WM_gizmo_new("GIZMO_GT_arrow_3d", &gzgroup, nullptr);
    WM_gizmo_set_line_width(gizmo_, 1.0f);
  }

  void update_style(const bNode &gizmo_node) override
  {
    const auto &storage = *static_cast<const NodeGeometryLinearGizmo *>(gizmo_node.storage);

    /* Make sure the enum values are in sync. */
    static_assert(int(GEO_NODE_LINEAR_GIZMO_DRAW_STYLE_ARROW) == int(ED_GIZMO_ARROW_STYLE_NORMAL));
    static_assert(int(GEO_NODE_LINEAR_GIZMO_DRAW_STYLE_BOX) == int(ED_GIZMO_ARROW_STYLE_BOX));
    static_assert(int(GEO_NODE_LINEAR_GIZMO_DRAW_STYLE_CROSS) == int(ED_GIZMO_ARROW_STYLE_CROSS));
    RNA_enum_set(gizmo_->ptr, "draw_style", storage.draw_style);

    const ThemeColorID color_theme_id = get_gizmo_theme_color_id(
        GeometryNodeGizmoColor(storage.color_id));
    UI_GetThemeColor3fv(color_theme_id, gizmo_->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gizmo_->color_hi);
  }

  void update(GizmosUpdateParams &params) override
  {
    const bNodeSocket &position_socket = params.gizmo_node.input_socket(1);
    const bNodeSocket &direction_socket = params.gizmo_node.input_socket(2);
    const std::optional<float3> position_opt = params.tree_log.find_primitive_socket_value<float3>(
        position_socket);
    const std::optional<float3> direction_opt =
        params.tree_log.find_primitive_socket_value<float3>(direction_socket);
    if (!position_opt || !direction_opt) {
      params.r_report.missing_socket_logs = true;
      return;
    }
    const float3 position = *position_opt;
    const float3 direction = math::normalize(*direction_opt);
    if (math::is_zero(direction)) {
      params.r_report.invalid_transform = true;
      return;
    }
    const float4x4 gizmo_base_transform = matrix_from_position_and_up_direction(
        position, direction, math::AxisSigned::Z_POS);

    const bool is_interacting = gizmo_is_interacting(*gizmo_);
    if (!is_interacting) {
      float4x4 gizmo_transform = params.parent_transform * gizmo_base_transform;
      edit_data_.factor_from_transform = math::length(gizmo_transform.z_axis());
      make_matrix_orthonormal_but_keep_z_axis(gizmo_transform);
      copy_m4_m4(gizmo_->matrix_basis, gizmo_transform.ptr());

      /* Always reset to 0 when not interacting. */
      edit_data_.current_value = 0.0f;

      wmGizmoPropertyFnParams params{};
      params.user_data = this;
      params.value_set_fn =
          [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, const void *value_ptr) {
            LinearGizmo &self = *static_cast<LinearGizmo *>(gz_prop->custom_func.user_data);
            const float new_gizmo_value = *static_cast<const float *>(value_ptr);
            self.edit_data_.current_value = new_gizmo_value;
            self.apply_change("Value", [&](bke::SocketValueVariant &value_variant) {
              value_variant.set(value_variant.get<float>() + new_gizmo_value);
            });
          };
      params.value_get_fn = [](const wmGizmo * /*gz*/, wmGizmoProperty *gz_prop, void *value_ptr) {
        LinearGizmo &self = *static_cast<LinearGizmo *>(gz_prop->custom_func.user_data);
        *static_cast<float *>(value_ptr) = self.edit_data_.current_value;
      };
      WM_gizmo_target_property_def_func(gizmo_, "offset", &params);
    }
  }

  Vector<wmGizmo *> get_all_gizmos() override
  {
    return {gizmo_};
  }
};

class DialGizmo : public NodeGizmos {
 private:
  wmGizmo *gizmo_ = nullptr;

 public:
  void create_gizmos(wmGizmoGroup &gzgroup) override
  {
    gizmo_ = WM_gizmo_new("GIZMO_GT_dial_3d", &gzgroup, nullptr);
    WM_gizmo_set_flag(gizmo_, WM_GIZMO_DRAW_VALUE, true);
    WM_gizmo_set_line_width(gizmo_, 2.0f);
    RNA_boolean_set(gizmo_->ptr, "wrap_angle", false);
  }

  void update_style(const bNode &gizmo_node) override
  {
    const auto &storage = *static_cast<const NodeGeometryDialGizmo *>(gizmo_node.storage);

    const bool is_interacting = gizmo_is_interacting(*gizmo_);
    int draw_options = RNA_enum_get(gizmo_->ptr, "draw_options");
    SET_FLAG_FROM_TEST(draw_options, is_interacting, ED_GIZMO_DIAL_DRAW_FLAG_ANGLE_VALUE);
    RNA_enum_set(gizmo_->ptr, "draw_options", draw_options);

    const ThemeColorID color_theme_id = get_gizmo_theme_color_id(
        GeometryNodeGizmoColor(storage.color_id));
    UI_GetThemeColor3fv(color_theme_id, gizmo_->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gizmo_->color_hi);
  }

  Vector<wmGizmo *> get_all_gizmos() override
  {
    return {gizmo_};
  }
};

class TransformGizmos : public NodeGizmos {
 private:
  std::array<wmGizmo *, 3> translation_gizmos_ = {};

 public:
  void create_gizmos(wmGizmoGroup &gzgroup) override
  {
    for (const int axis : IndexRange(3)) {
      wmGizmo *gizmo = WM_gizmo_new("GIZMO_GT_arrow_3d", &gzgroup, nullptr);
      WM_gizmo_set_line_width(gizmo, 1.0f);
      translation_gizmos_[axis] = gizmo;
    }
  }

  Vector<wmGizmo *> get_all_gizmos() override
  {
    return translation_gizmos_;
  }
};

struct GeometryNodesGizmoGroup {
  Map<bke::GeoNodesGizmoID, std::unique_ptr<NodeGizmos>> gizmos_by_node;
};

static std::unique_ptr<NodeGizmos> create_node_gizmos(const bNode &gizmo_node)
{
  switch (gizmo_node.type) {
    case GEO_NODE_GIZMO_LINEAR:
      return std::make_unique<LinearGizmo>();
    case GEO_NODE_GIZMO_DIAL:
      return std::make_unique<DialGizmo>();
    case GEO_NODE_GIZMO_TRANSFORM:
      return std::make_unique<TransformGizmos>();
  }
  return {};
}

static bool WIDGETGROUP_geometry_nodes_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  ScrArea *area = CTX_wm_area(C);
  View3D *v3d = static_cast<View3D *>(area->spacedata.first);
  if (v3d->gizmo_flag & V3D_GIZMO_HIDE_MODIFIER) {
    return false;
  }
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

static void WIDGETGROUP_geometry_nodes_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  auto &gzgroup_data = *static_cast<GeometryNodesGizmoGroup *>(gzgroup->customdata);

  View3D *v3d = CTX_wm_view3d(C);
  if (!v3d) {
    return;
  }

  Object *ob_orig = CTX_data_active_object(C);
  NodesModifierData &nmd = *reinterpret_cast<NodesModifierData *>(
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

  bke::GeometrySet geometry = bke::object_get_evaluated_geometry_set(*ob_eval);
  if (v3d->flag2 & V3D_SHOW_VIEWER) {
    const ViewerPath &viewer_path = v3d->viewer_path;
    if (const geo_eval_log::ViewerNodeLog *viewer_log =
            nmd.runtime->eval_log->find_viewer_node_log_for_path(viewer_path))
    {
      geometry = viewer_log->geometry;
    }
  }

  bNodeTree &ntree = *nmd.node_group;
  ntree.ensure_topology_cache();

  const float4x4 object_to_world{ob_orig->object_to_world()};

  Map<bke::GeoNodesGizmoID, std::unique_ptr<NodeGizmos>> new_gizmos_by_node;

  /* This needs to stay around for a bit longer because the compute contexts are required when
   * applying the gizmo changes. */
  auto compute_context_builder = std::make_shared<ComputeContextBuilder>();

  nodes::gizmos::foreach_active_gizmo(
      *ob_orig,
      nmd,
      *wm,
      *compute_context_builder,
      [&](const ComputeContext &compute_context, const bNode &gizmo_node) {
        const bke::GeoNodesGizmoID gizmo_id = {compute_context.hash(), gizmo_node.identifier};
        if (new_gizmos_by_node.contains(gizmo_id)) {
          /* Already handled. */
          return;
        }
        NodeGizmos *node_gizmos = nullptr;
        if (std::optional<std::unique_ptr<NodeGizmos>> old_gizmos =
                gzgroup_data.gizmos_by_node.pop_try(gizmo_id))
        {
          node_gizmos = old_gizmos->get();
          new_gizmos_by_node.add(gizmo_id, std::move(*old_gizmos));
        }
        else {
          std::unique_ptr<NodeGizmos> new_node_gizmos = create_node_gizmos(gizmo_node);
          new_node_gizmos->create_gizmos(*gzgroup);
          for (wmGizmo *gizmo : new_node_gizmos->get_all_gizmos()) {
            gizmo->flag |= WM_GIZMO_NEEDS_UNDO;
          }
          node_gizmos = new_node_gizmos.get();
          new_gizmos_by_node.add(gizmo_id, std::move(new_node_gizmos));
        }

        node_gizmos->update_style(gizmo_node);
        GeoTreeLog &tree_log = nmd.runtime->eval_log->get_tree_log(compute_context.hash());
        tree_log.ensure_socket_values();

        UpdateReport report;
        /* TODO: geometry transform */
        GizmosUpdateParams update_params{object_to_world, gizmo_node, tree_log, report};
        node_gizmos->update(update_params);

        bool any_interacting = false;
        for (const wmGizmo *gizmo : node_gizmos->get_all_gizmos()) {
          any_interacting |= gizmo_is_interacting(*gizmo);
        }

        if (!any_interacting) {
          node_gizmos->apply_change =
              [C = C,
               compute_context_builder,
               compute_context = &compute_context,
               gizmo_node_tree = &gizmo_node.owner_tree(),
               gizmo_node = &gizmo_node,
               ob_orig = ob_orig,
               nmd = &nmd,
               eval_log = nmd.runtime->eval_log](
                  const StringRef socket_identifier,
                  const FunctionRef<void(bke::SocketValueVariant & value)> modify_value) {
                gizmo_node_tree->ensure_topology_cache();
                const bNodeSocket &socket = gizmo_node->input_by_identifier(socket_identifier);

                nodes::gizmos::apply_gizmo_change(
                    *ob_orig, *nmd, *eval_log, *compute_context, *gizmo_node, modify_value);

                Main *main = CTX_data_main(C);
                ED_node_tree_propagate_change(const_cast<bContext *>(C), main, nullptr);
                WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
              };
        }

        if (report.missing_socket_logs) {
          /* Rerun modifier to make sure that values are logged. */
          DEG_id_tag_update(&ob_orig->id, ID_RECALC_GEOMETRY);
          WM_main_add_notifier(NC_GEOM | ND_DATA, nullptr);
        }
        if (!any_interacting) {
          if (report.missing_socket_logs || report.invalid_transform) {
            return;
          }
        }
      });

  for (std::unique_ptr<NodeGizmos> &node_gizmos : gzgroup_data.gizmos_by_node.values()) {
    const Vector<wmGizmo *> gizmos = node_gizmos->get_all_gizmos();
    for (wmGizmo *gizmo : gizmos) {
      WM_gizmo_unlink(&gzgroup->gizmos, gzgroup->parent_gzmap, gizmo, const_cast<bContext *>(C));
    }
  }
  gzgroup_data.gizmos_by_node = std::move(new_gizmos_by_node);
}

static void WIDGETGROUP_geometry_nodes_draw_prepare(const bContext * /*C*/,
                                                    wmGizmoGroup * /*gzgroup*/)
{
}

}  // namespace blender::ed::view3d::geometry_nodes_gizmos

void VIEW3D_GGT_geometry_nodes(wmGizmoGroupType *gzgt)
{
  using namespace blender::ed::view3d::geometry_nodes_gizmos;

  gzgt->name = "Geometry Nodes Widgets";
  gzgt->idname = "VIEW3D_GGT_geometry_nodes";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D);

  gzgt->poll = WIDGETGROUP_geometry_nodes_poll;
  gzgt->setup = WIDGETGROUP_geometry_nodes_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_geometry_nodes_refresh;
  gzgt->draw_prepare = WIDGETGROUP_geometry_nodes_draw_prepare;
}
