/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spview3d
 */

#include "BKE_attribute.hh"
#include "BKE_context.h"
#include "BKE_geometry_set.hh"
#include "BKE_gizmos.hh"
#include "BKE_layer.h"

#include "BLI_array.hh"
#include "BLI_index_range.hh"
#include "BLI_math_matrix.hh"
#include "BLI_span.hh"

#include "DNA_node_types.h"

#include "DEG_depsgraph_query.h"

#include "ED_gizmo_library.h"
#include "ED_screen.h"

#include "RNA_access.h"
#include "RNA_path.h"
#include "RNA_prototypes.h"

#include "UI_resources.h"

#include "WM_api.h"
#include "WM_types.h"

#include "view3d_intern.h" /* own include */

namespace blender::gizmos::geometry_node_gizmo {

static const bke::GizmosGeometry static_empty_gizmos = {};

struct MetaData {
  int size;

  bool operator==(const MetaData &other) const
  {
    return this->size == other.size;
  }

  bool operator!=(const MetaData &other) const
  {
    return !(*this == other);
  }
};

static MetaData meta_data_for_gizmos(const bke::GizmosGeometry &gizmos)
{
  return {gizmos.gizmos_num()};
}

struct GizmoContext {
  const Object &object;
  const bke::GizmosGeometry &gizmo;
};

static std::optional<GizmoContext> gizmos_context_from_context(const bContext *C)
{
  const View3D *v3d = CTX_wm_view3d(C);
  if (v3d->gizmo_flag & (V3D_GIZMO_HIDE | V3D_GIZMO_HIDE_CONTEXT)) {
    return std::nullopt;
  }

  const Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  BKE_view_layer_synced_ensure(scene, view_layer);
  const Base *base = BKE_view_layer_active_base_get(view_layer);
  if (base == nullptr || !BASE_SELECTABLE(v3d, base)) {
    return std::nullopt;
  }

  Object *object = base->object;
  if (object == nullptr) {
    return std::nullopt;
  }

  const Depsgraph *depsgraph = CTX_data_depsgraph_pointer(C);
  BLI_assert(depsgraph != nullptr);
  const Object *evaluated_object = DEG_get_evaluated_object(depsgraph, object);
  BLI_assert(evaluated_object != nullptr);

  const bke::GeometrySet *evaluated_geometry_set = evaluated_object->runtime.geometry_set_eval;
  if (evaluated_geometry_set == nullptr) {
    return std::nullopt;
  }
  const bke::GizmosGeometry *gizmo = evaluated_geometry_set->get_gizmos_for_read();
  if (gizmo == nullptr) {
    GizmoContext context{*object, static_empty_gizmos};
    return context;
  }

  GizmoContext context{*object, *gizmo};
  return context;
}

struct GizmoData {
  Vector<wmGizmo *> gizmos_objects;
  MetaData meta_data;

  GizmoData(const bke::GizmosGeometry &gizmos)
      : gizmos_objects(gizmos.gizmos_num()), meta_data(meta_data_for_gizmos(gizmos))
  {
  }

  ~GizmoData() = default;

  static void free(void *this_)
  {
    static_cast<GizmoData *>(this_)->~GizmoData();
  }
};

static bool geometry_node_poll(const bContext *C, wmGizmoGroupType * /*gzgt*/)
{
  if (gizmos_context_from_context(C).has_value()) {
    return true;
  }
  return false;
}

static void geometry_node_setup(const bContext *C, wmGizmoGroup *gzgroup)
{
  const std::optional<GizmoContext> context = gizmos_context_from_context(C);
  BLI_assert(context.has_value());
  const bke::GizmosGeometry &gizmos = context->gizmo;

  gzgroup->customdata = new GizmoData(gizmos);
  gzgroup->customdata_free = GizmoData::free;
  GizmoData &gzgroup_data = *static_cast<GizmoData *>(gzgroup->customdata);

  for (const int index : IndexRange(gizmos.gizmos_num())) {
    wmGizmo *gizmo = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, NULL);
    RNA_enum_set(gizmo->ptr, "transform", ED_GIZMO_ARROW_XFORM_FLAG_INVERTED);
    RNA_enum_set(gizmo->ptr, "draw_options", ED_GIZMO_ARROW_DRAW_FLAG_STEM);
    UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gizmo->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gizmo->color_hi);
    gzgroup_data.gizmos_objects[index] = gizmo;
  }
}

static void refresh_data_to_new_meta(GizmoData &data,
                                     MetaData &new_meta,
                                     wmGizmoGroup *gzgroup,
                                     const bContext *C)
{
  const int size = data.gizmos_objects.size();
  if (data.meta_data.size > new_meta.size) {
    wmGizmo *gizmo = data.gizmos_objects.pop_last();
    WM_gizmo_unlink(&gzgroup->gizmos, gzgroup->parent_gzmap, gizmo, const_cast<bContext *>(C));
  }
  if (data.meta_data.size < new_meta.size) {
    wmGizmo *gizmo = WM_gizmo_new("GIZMO_GT_arrow_3d", gzgroup, NULL);
    RNA_enum_set(gizmo->ptr, "transform", ED_GIZMO_ARROW_XFORM_FLAG_INVERTED);
    RNA_enum_set(gizmo->ptr, "draw_options", ED_GIZMO_ARROW_DRAW_FLAG_STEM);
    UI_GetThemeColor3fv(TH_GIZMO_PRIMARY, gizmo->color);
    UI_GetThemeColor3fv(TH_GIZMO_HI, gizmo->color_hi);
    data.gizmos_objects.append(gizmo);
  }
  data.meta_data = new_meta;
  // printf("%d -> %d;\n", sze, data.gizmos_objects.size());
}

static void geometry_node_refresh(const bContext *C, wmGizmoGroup *gzgroup)
{
  const std::optional<GizmoContext> context = gizmos_context_from_context(C);
  BLI_assert(context.has_value());
  const Object &object = context->object;
  const bke::GizmosGeometry &gizmos = context->gizmo;
  GizmoData &gzgroup_data = *static_cast<GizmoData *>(gzgroup->customdata);

  MetaData new_meta_data = meta_data_for_gizmos(gizmos);
  if (gzgroup_data.meta_data != new_meta_data) {
    refresh_data_to_new_meta(gzgroup_data, new_meta_data, gzgroup, C);
  }

  const Span<std::string> pathes = gizmos.pathes();
  const Span<int> paths_mapping = gizmos.paths_mapping();
  for (const int index : paths_mapping.index_range()) {
    wmGizmo *gizmo = gzgroup_data.gizmos_objects[index];

    const StringRefNull rna_path = pathes[index];

    PointerRNA blender_data_pointer;
    RNA_pointer_create(NULL, &RNA_BlendData, CTX_data_main(C), &blender_data_pointer);

    PointerRNA node_ptr;
    PropertyRNA *value_prop;
    const bool path_is_resolved = RNA_path_resolve_full(
        &blender_data_pointer, rna_path.data(), &node_ptr, &value_prop, nullptr);
    BLI_assert(path_is_resolved);
    UNUSED_VARS_NDEBUG(path_is_resolved);

    WM_gizmo_target_property_def_rna(gizmo, "offset", &node_ptr, "value", -1);
  }
}

static void geometry_node_draw_prepare(const bContext *C, wmGizmoGroup *gzgroup)
{
  const std::optional<GizmoContext> context = gizmos_context_from_context(C);
  BLI_assert(context.has_value());
  const Object &object = context->object;
  const bke::GizmosGeometry &gizmos = context->gizmo;
  GizmoData &gzgroup_data = *static_cast<GizmoData *>(gzgroup->customdata);

  const bke::AttributeAccessor attributes = gizmos.attributes();
  const bke::AttributeReader<float3> positions = attributes.lookup_or_default<float3>(
      "position", ATTR_DOMAIN_POINT, float3());
  const bke::AttributeReader<float3> scales = attributes.lookup_or_default<float3>(
      "scale", ATTR_DOMAIN_POINT, float3());
  const bke::AttributeReader<math::Quaternion> rotations =
      attributes.lookup_or_default<math::Quaternion>(
          "rotation", ATTR_DOMAIN_POINT, math::Quaternion());

  const float4x4 object_mathix(object.object_to_world);

  for (const int index : IndexRange(gizmos.gizmos_num())) {
    wmGizmo *gizmo = gzgroup_data.gizmos_objects[index];
    const float3 normal = {1.0f, 0.0f, 0.0f};
    const float3 &position = positions.varray[index];
    const float3 &scale = scales.varray[index];
    const math::Quaternion &rotation = rotations.varray[index];
    const float3 world_position = math::transform_point(object_mathix, position);
    [[maybe_unused]] const float3 world_scale = math::transform_point(object_mathix, position);
    const float3 world_rotation = math::transform_point(object_mathix, position);
    WM_gizmo_set_matrix_location(gizmo, world_position);

    const float4x4 mathix = math::from_loc_rot_scale<MatBase<float, 4, 4>>(
        position, rotation, scale);

    const float4x4 final_mathix = mathix * object_mathix;

    const float3 normal_a = math::transform_point(final_mathix, float3(0.0f, 1.0f, 0.0f));
    const float3 normal_b = math::transform_point(final_mathix, float3(0.0f, 0.0f, 1.0f));
    WM_gizmo_set_matrix_rotation_from_yz_axis(gizmo, normal_a, normal_b);
  }
}
}  // namespace blender::gizmos::geometry_node_gizmo

void VIEW3D_GGT_geometry_node_gizmos(wmGizmoGroupType *gzgt)
{
  gzgt->name = "Geometry Node Gizmos";
  gzgt->idname = "VIEW3D_GGT_geometry_node_gizmos";

  gzgt->flag |= (WM_GIZMOGROUPTYPE_PERSISTENT | WM_GIZMOGROUPTYPE_3D | WM_GIZMOGROUPTYPE_DEPTH_3D);

  namespace ns = blender::gizmos::geometry_node_gizmo;

  gzgt->poll = ns::geometry_node_poll;
  gzgt->setup = ns::geometry_node_setup;
  gzgt->setup_keymap = WM_gizmogroup_setup_keymap_generic_maybe_drag;
  gzgt->refresh = ns::geometry_node_refresh;
  gzgt->draw_prepare = ns::geometry_node_draw_prepare;
}

/** \} */
