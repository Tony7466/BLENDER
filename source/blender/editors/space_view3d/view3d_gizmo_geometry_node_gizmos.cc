/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BKE_context.h"
#include "BKE_geometry_set.hh"
#include "BKE_gizmos.hh"
#include "BKE_layer.h"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_span.hh"

#include "DNA_node_types.h"

#include "DEG_depsgraph_query.h"

#include "ED_gizmo_library.h"
#include "ED_screen.h"

#include "RNA_access.h"
#include "RNA_prototypes.h"

#include "UI_resources.h"

#include "WM_api.h"
#include "WM_types.h"

#include "view3d_intern.h" /* own include */

namespace blender {

struct GeometryNodeGizmoWrapper {
  const bke::GizmosGeometry &gizmos_geometry;
  Array<wmGizmo *> gizmos_objects;

  GeometryNodeGizmoWrapper(const bke::GizmosGeometry &gizmos_geometry_source)
      : gizmos_geometry(gizmos_geometry_source), gizmos_objects(gizmos_geometry.gizmos_num())
  {
  }
};

static const bke::GizmosGeometry *gizmos_from_context_try(const bContext *C)
{
  const View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return nullptr;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  const Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base == nullptr || !BASE_SELECTABLE(v3d, base)) {
    return nullptr;
  }

  Object *object = base->object;
  if (object == nullptr) {
    return nullptr;
  }

  const Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  BLI_assert(depsgraph != nullptr);
  const Object *evaluated_object = DEG_get_evaluated_object(depsgraph, object);
  BLI_assert(evaluated_object != nullptr);

  const GeometrySet *evaluated_geometry_set = evaluated_object->runtime.geometry_set_eval;
  if (evaluated_geometry_set == nullptr) {
    return nullptr;
  }

  return evaluated_geometry_set->get_gizmos_for_read();
}

static bool WIDGETGROUP_geometry_node_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  if (gizmos_from_context_try(C) != nullptr) {
    return true;
  }
  return false;
}

static void WIDGETGROUP_geometry_node_setup(const bContext *C, wmGizmoGroup *gzgroup)
{
  const bke::GizmosGeometry *gizmos = gizmos_from_context_try(C);
  BLI_assert(gizmos != nullptr);

  GeometryNodeGizmoWrapper &gzgroup_data = *(new GeometryNodeGizmoWrapper(*gizmos));
  gzgroup->customdata = &gzgroup_data;

  for (const int index : IndexRange(gizmos->gizmos_num())) {
    wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, NULL);
    RNA_enum_set(gz->ptr, "transform", ED_GIZMO_ARROW_XFORM_FLAG_INVERTED);
    ED_gizmo_arrow3d_set_range_fac(gz, 1.0f);
    UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gz->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gz->color_hi);
    gzgroup_data.gizmos_objects[index] = gz;
  }
}

static void WIDGETGROUP_geometry_node_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  ViewLayer *view_layer = CTX_data_view_layer(C);
  const Object *object = BKE_view_layer_active_object_get(view_layer);
  GeometryNodeGizmoWrapper &gzgroup_data = *static_cast<GeometryNodeGizmoWrapper *>(
      gzgroup->customdata);

  const Span<bNode *> nodes = gzgroup_data.gizmos_geometry.nodes();
  const Span<bNodeTree *> trees = gzgroup_data.gizmos_geometry.trees();
  const Span<std::string> pathes = gzgroup_data.gizmos_geometry.pathes();

  const Span<int> paths_mapping = gzgroup_data.gizmos_geometry.paths_mapping();
  for (const int index : paths_mapping.index_range()) {
    PointerRNA node_ptr;

    bNode *node = nodes[index];
    bNodeTree *tree = trees[index];
    const StringRefNull rna_path = pathes[index];
    RNA_pointer_create(&tree->id, &RNA_Node, node, &node_ptr);

    wmGizmo *gz = gzgroup_data.gizmos_objects[index];
    const float3 normal = {1.0f, 0.0f, 0.0f};
    WM_gizmo_set_matrix_rotation_from_z_axis(gz, normal);
    WM_gizmo_set_matrix_location(gz, object->object_to_world[3]);
    WM_gizmo_target_property_def_rna(gz, "offset", &node_ptr, rna_path.data(), -1);
  }
}

static void WIDGETGROUP_geometry_node_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(CTX_data_scene(C), view_layer);
  const Object *object = BKE_view_layer_active_object_get(view_layer);
  GeometryNodeGizmoWrapper &gzgroup_data = *static_cast<GeometryNodeGizmoWrapper *>(
      gzgroup->customdata);

  for (const int index : IndexRange(gzgroup_data.gizmos_geometry.gizmos_num())) {
    wmGizmo *gz = gzgroup_data.gizmos_objects[index];
    const float3 normal = {1.0f, 0.0f, 0.0f};
    WM_gizmo_set_matrix_rotation_from_z_axis(gz, normal);
    WM_gizmo_set_matrix_location(gz, object->object_to_world[3]);
  }
}
}  // namespace blender

void VIEW3D_GGT_geometry_node_gizmos(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Geometry Node Gizmos";
  gzgt->idname = "VIEW3D_GGT_geometry_node_gizmos";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  using namespace blender;

  gzgt->poll = WIDGETGROUP_geometry_node_poll;
  gzgt->setup = WIDGETGROUP_geometry_node_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = WIDGETGROUP_geometry_node_refresh;
  gzgt->draw_prepare = WIDGETGROUP_geometry_node_draw_prepare;
}

/** \} */
