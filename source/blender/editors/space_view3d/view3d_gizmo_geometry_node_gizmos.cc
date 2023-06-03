/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

namespace blender {

struct GeometryNodeGizmoWrapper {
  const bke::GizmosGeoemtry &gizmos_geometry;
  Array<wmGizmo *> gizmos_objects;

  GeometryNodeGizmoWrapper(const bke::GizmosGeoemtry &gizmos_geometry_source) :
      gizmos_geometry(gizmos_geometry_source),
      gizmos_objects(gizmos_geometry.size())
    {}
};

static const bke::GizmosGeoemtry &gizmos_from_context(const bContext *C)
{
  const View3D *v3d = CTX_wm_view3d(C);
  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  const Base *base = BKE_view_layer_active_base_get(view_layer);
  BLI_assert(base != nullptr);
  BLI_assert(BASE_SELECTABLE(v3d, base));
  const Object *ob = base->object;
  BLI_assert(ob != nullptr);
  BLI_assert(ob.runtime != nullptr);
  const GeometrySet *geometry_set_eval = ob.runtime.geometry_set_eval;
  BLI_assert(geometry_set_eval != nullptr);
  BLI_assert(geometry_set_eval.has_gizmos());
  return *geometry_set_eval->get_gizmos_for_read();
}

static bool WIDGETGROUP_geometry_node_poll(const bContext *C, wmGizmoGroupType */*gzgt*/)
{
  const View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return false;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  const Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base == nullptr || !BASE_SELECTABLE(v3d, base)) {
    return false;
  }
  const Object *ob = base->object;
  if (ob.runtime == nullptr){
    return false;
  }
  const GeometrySet *geometry_set_eval = ob.runtime.geometry_set_eval;
  if (geometry_set_eval == nullptr){
    return false;
  }
  return geometry_set_eval->has_gizmos();
}

static void WIDGETGROUP_geometry_node_setup(const bContext *C, wmGizmoGroup *gzgroup)
{
  const bke::GizmosGeoemtry &gizmos = gizmos_from_context(C);

  GeometryNodeGizmoWrapper &gzgroup_data = new GeometryNodeGizmoWrapper(gizmos);
  gzgroup->customdata = &gzgroup_data;

  for (const int index : IndexRange(gizmos.gizmos_num())) {
    wmGizmo *gz = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, NULL);
    RNA_enum_set(gz->ptr, "transform", ED_GIZMO_ARROW_XFORM_FLAG_INVERTED);
    ED_gizmo_arrow3d_set_range_fac(gz, 4.0f);
    UI_GetThemeColor3fv(TH_GIZMO_SECONDARY, gz->color);
    gzgroup_data.gizmos_objects[index] = gz;
  }
}

static void WIDGETGROUP_geometry_node_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  GeometryNodeGizmoWrapper &gzgroup_data = *static_cast<GeometryNodeGizmoWrapper *>(gzgroup->customdata);

  const Span<int> paths_mapping = gzgroup_data.gizmos_geometry.paths_mapping();

  for (const int index : paths_mapping.index_range()) {
    PointerRNA node_ptr;
    RNA_pointer_create(&la->id, &RNA_Node, la, &node_ptr);

    wmGizmo *gz = ls_gzgroup->spot_angle;
    float dir[3];
    negate_v3_v3(dir, ob->object_to_world[2]);
    WM_gizmo_set_matrix_rotation_from_z_axis(gz, dir);
    WM_gizmo_set_matrix_location(gz, ob->object_to_world[3]);

    const char *propname = "spot_size";
    WM_gizmo_target_property_def_rna(gz, "offset", &lamp_ptr, propname, -1);
  }
}

static void WIDGETGROUP_geometry_node_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  LightSpotWidgetGroup *ls_gzgroup = gzgroup->customdata;
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(CTX_data_scene(C), view_layer);
  Object *ob = BKE_view_layer_active_object_get(view_layer);

  /* Spot radius gizmo. */
  wmGizmo *gz = ls_gzgroup->spot_radius;

  /* Draw circle in the screen space. */
  RegionView3D *rv3d = CTX_wm_region(C)->regiondata;
  WM_gizmo_set_matrix_rotation_from_z_axis(gz, rv3d->viewinv[2]);

  WM_gizmo_set_matrix_location(gz, ob->object_to_world[3]);
}

}

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
