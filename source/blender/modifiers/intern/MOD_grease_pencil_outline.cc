/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_index_range.hh"
#include "BLI_span.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_lib_query.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(omd, modifier));

  MEMCPY_STRUCT_AFTER(omd, DNA_struct_default_get(GreasePencilOutlineModifierData), modifier);
  modifier::greasepencil::init_influence_data(&omd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *omd = reinterpret_cast<const GreasePencilOutlineModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilOutlineModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&omd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  modifier::greasepencil::free_influence_data(&omd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&omd->influence, ob, walk, user_data);
  walk(user_data, ob, (ID **)&omd->outline_material, IDWALK_CB_USER);
  walk(user_data, ob, (ID **)&omd->object, IDWALK_CB_NOP);
}

static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  if (ctx->scene->camera) {
    DEG_add_object_relation(
        ctx->node, ctx->scene->camera, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
    DEG_add_object_relation(
        ctx->node, ctx->scene->camera, DEG_OB_COMP_PARAMETERS, "Grease Pencil Outline Modifier");
  }
  if (omd->object != nullptr) {
    DEG_add_object_relation(
        ctx->node, omd->object, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
  }
  DEG_add_object_relation(
      ctx->node, ctx->object, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
}

static void generate_arc_from_point_to_point(const float3 &from,
                                             const float3 &to,
                                             const float3 &center_pt,
                                             const int subdivisions,
                                             Vector<float3> &r_perimeter)
{
  const float2 vec_from = from.xy() - center_pt.xy();
  const float2 vec_to = to.xy() - center_pt.xy();
  if (math::is_zero(vec_from) || math::is_zero(vec_to)) {
    return;
  }

  const float dot = math::dot(vec_from, vec_to);
  const float det = vec_from.x * vec_to.y - vec_from.y * vec_to.x;
  const float angle = math::atan2(det, dot) + M_PI;

  /* Number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
   * so we multiply by (angle / pi) to get the right amount of
   * points to insert. */
  const int num_points = int(((1 << (subdivisions + 1)) - 1) * (angle / M_PI));
  if (num_points <= 0) {
    return;
  }

  const float2x2 rotation = math::from_rotation<float2x2>(angle / float(num_points));
  float2 vec = vec_from;
  for ([[maybe_unused]] const int i : IndexRange(num_points).drop_back(1)) {
    vec = rotation * vec;
    r_perimeter.append_as(vec);
  }
}

static void generate_semi_circle_from_point_to_point(const float3 &from,
                                                     const float3 &to,
                                                     int subdivisions,
                                                     Vector<float3> &r_perimeter)
{
  const int num_points = (1 << (subdivisions + 1)) + 1;

  UNUSED_VARS(from, to, subdivisions, r_perimeter);
  //   int num_points = (1 << (subdivisions + 1)) + 1;
  //   float center_pt[3];
  //   interp_v3_v3v3(center_pt, &from->x, &to->x, 0.5f);

  //   float vec_center[2];
  //   sub_v2_v2v2(vec_center, &from->x, center_pt);
  //   if (is_zero_v2(vec_center)) {
  //     return 0;
  //   }

  //   float vec_p[3];
  //   float angle_incr = M_PI / (float(num_points) - 1);

  //   tPerimeterPoint *last_point = from;
  //   for (int i = 1; i < num_points; i++) {
  //     float angle = i * angle_incr;

  //     /* Rotate vector around point to get perimeter points. */
  //     rotate_v2_v2fl(vec_p, vec_center, angle);
  //     add_v2_v2(vec_p, center_pt);
  //     vec_p[2] = center_pt[2];

  //     tPerimeterPoint *new_point = new_perimeter_point(vec_p);
  //     BLI_insertlinkafter(list, last_point, new_point);

  //     last_point = new_point;
  //   }

  //   return num_points - 1;
}

static void generate_perimeter_cap(const float point[4],
                                   const float other_point[4],
                                   float radius,
                                   int subdivisions,
                                   short cap_type,
                                   Vector<float3> &r_perimeter)
{
  UNUSED_VARS(point, other_point, radius, subdivisions, cap_type, r_perimeter);
  //   float cap_vec[2];
  //   sub_v2_v2v2(cap_vec, other_point, point);
  //   normalize_v2(cap_vec);

  //   float cap_nvec[2];
  //   if (is_zero_v2(cap_vec)) {
  //     cap_nvec[0] = 0;
  //     cap_nvec[1] = radius;
  //   }
  //   else {
  //     cap_nvec[0] = -cap_vec[1];
  //     cap_nvec[1] = cap_vec[0];
  //     mul_v2_fl(cap_nvec, radius);
  //   }
  //   float cap_nvec_inv[2];
  //   negate_v2_v2(cap_nvec_inv, cap_nvec);

  //   float vec_perimeter[3];
  //   copy_v3_v3(vec_perimeter, point);
  //   add_v2_v2(vec_perimeter, cap_nvec);

  //   float vec_perimeter_inv[3];
  //   copy_v3_v3(vec_perimeter_inv, point);
  //   add_v2_v2(vec_perimeter_inv, cap_nvec_inv);

  //   tPerimeterPoint *p_pt = new_perimeter_point(vec_perimeter);
  //   tPerimeterPoint *p_pt_inv = new_perimeter_point(vec_perimeter_inv);

  //   BLI_addtail(list, p_pt);
  //   BLI_addtail(list, p_pt_inv);

  //   int num_points = 0;
  //   if (cap_type == GP_STROKE_CAP_ROUND) {
  //     num_points += generate_semi_circle_from_point_to_point(list, p_pt, p_pt_inv,
  //     subdivisions);
  //   }

  //   return num_points + 2;
}

/**
 * Calculate the perimeter (outline) of a stroke.
 * \param subdivisions: Number of subdivisions for the start and end caps
 */
static void gpencil_stroke_perimeter_ex(const bGPdata *gpd,
                                        const bGPDlayer *gpl,
                                        const bGPDstroke *gps,
                                        int subdivisions,
                                        const float thickness_chg,
                                        Vector<float3> &r_perimeter)
{
  UNUSED_VARS(gpd, gpl, gps, subdivisions, thickness_chg, r_perimeter);
  //   /* sanity check */
  //   if (gps->totpoints < 1) {
  //     return;
  //   }

  //   float defaultpixsize = 1000.0f / gpd->pixfactor;
  //   float ovr_radius = thickness_chg / defaultpixsize / 2.0f;
  //   float stroke_radius = ((gps->thickness + gpl->line_change) / defaultpixsize) / 2.0f;
  //   stroke_radius = max_ff(stroke_radius - ovr_radius, 0.0f);

  //   ListBase *perimeter_right_side = MEM_cnew<ListBase>(__func__);
  //   ListBase *perimeter_left_side = MEM_cnew<ListBase>(__func__);
  //   int num_perimeter_points = 0;

  //   bGPDspoint *first = &gps->points[0];
  //   bGPDspoint *last = &gps->points[gps->totpoints - 1];

  //   float first_radius = stroke_radius * first->pressure;
  //   float last_radius = stroke_radius * last->pressure;

  //   bGPDspoint *first_next;
  //   bGPDspoint *last_prev;
  //   if (gps->totpoints > 1) {
  //     first_next = &gps->points[1];
  //     last_prev = &gps->points[gps->totpoints - 2];
  //   }
  //   else {
  //     first_next = first;
  //     last_prev = last;
  //   }

  //   float first_pt[3];
  //   float last_pt[3];
  //   float first_next_pt[3];
  //   float last_prev_pt[3];
  //   copy_v3_v3(first_pt, &first->x);
  //   copy_v3_v3(last_pt, &last->x);
  //   copy_v3_v3(first_next_pt, &first_next->x);
  //   copy_v3_v3(last_prev_pt, &last_prev->x);

  //   /* Edge-case if single point. */
  //   if (gps->totpoints == 1) {
  //     first_next_pt[0] += 1.0f;
  //     last_prev_pt[0] -= 1.0f;
  //   }

  //   /* Generate points for start cap. */
  //   num_perimeter_points += generate_perimeter_cap(
  //       first_pt, first_next_pt, first_radius, perimeter_right_side, subdivisions,
  //       gps->caps[0]);

  //   /* Generate perimeter points. */
  //   float curr_pt[3], next_pt[3], prev_pt[3];
  //   float vec_next[2], vec_prev[2];
  //   float nvec_next[2], nvec_prev[2];
  //   float nvec_next_pt[3], nvec_prev_pt[3];
  //   float vec_tangent[2];

  //   float vec_miter_left[2], vec_miter_right[2];
  //   float miter_left_pt[3], miter_right_pt[3];

  //   for (int i = 1; i < gps->totpoints - 1; i++) {
  //     bGPDspoint *curr = &gps->points[i];
  //     bGPDspoint *prev = &gps->points[i - 1];
  //     bGPDspoint *next = &gps->points[i + 1];
  //     float radius = stroke_radius * curr->pressure;

  //     copy_v3_v3(curr_pt, &curr->x);
  //     copy_v3_v3(next_pt, &next->x);
  //     copy_v3_v3(prev_pt, &prev->x);

  //     sub_v2_v2v2(vec_prev, curr_pt, prev_pt);
  //     sub_v2_v2v2(vec_next, next_pt, curr_pt);
  //     float prev_length = len_v2(vec_prev);
  //     float next_length = len_v2(vec_next);

  //     if (normalize_v2(vec_prev) == 0.0f) {
  //       vec_prev[0] = 1.0f;
  //       vec_prev[1] = 0.0f;
  //     }
  //     if (normalize_v2(vec_next) == 0.0f) {
  //       vec_next[0] = 1.0f;
  //       vec_next[1] = 0.0f;
  //     }

  //     nvec_prev[0] = -vec_prev[1];
  //     nvec_prev[1] = vec_prev[0];

  //     nvec_next[0] = -vec_next[1];
  //     nvec_next[1] = vec_next[0];

  //     add_v2_v2v2(vec_tangent, vec_prev, vec_next);
  //     if (normalize_v2(vec_tangent) == 0.0f) {
  //       copy_v2_v2(vec_tangent, nvec_prev);
  //     }

  //     vec_miter_left[0] = -vec_tangent[1];
  //     vec_miter_left[1] = vec_tangent[0];

  //     /* calculate miter length */
  //     float an1 = dot_v2v2(vec_miter_left, nvec_prev);
  //     if (an1 == 0.0f) {
  //       an1 = 1.0f;
  //     }
  //     float miter_length = radius / an1;
  //     if (miter_length <= 0.0f) {
  //       miter_length = 0.01f;
  //     }

  //     normalize_v2_length(vec_miter_left, miter_length);

  //     copy_v2_v2(vec_miter_right, vec_miter_left);
  //     negate_v2(vec_miter_right);

  //     float angle = dot_v2v2(vec_next, nvec_prev);
  //     /* Add two points if angle is close to being straight. */
  //     if (fabsf(angle) < 0.0001f) {
  //       normalize_v2_length(nvec_prev, radius);
  //       normalize_v2_length(nvec_next, radius);

  //       copy_v3_v3(nvec_prev_pt, curr_pt);
  //       add_v2_v2(nvec_prev_pt, nvec_prev);

  //       copy_v3_v3(nvec_next_pt, curr_pt);
  //       negate_v2(nvec_next);
  //       add_v2_v2(nvec_next_pt, nvec_next);

  //       tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt);
  //       tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt);

  //       BLI_addtail(perimeter_left_side, normal_prev);
  //       BLI_addtail(perimeter_right_side, normal_next);
  //       num_perimeter_points += 2;
  //     }
  //     else {
  //       /* bend to the left */
  //       if (angle < 0.0f) {
  //         normalize_v2_length(nvec_prev, radius);
  //         normalize_v2_length(nvec_next, radius);

  //         copy_v3_v3(nvec_prev_pt, curr_pt);
  //         add_v2_v2(nvec_prev_pt, nvec_prev);

  //         copy_v3_v3(nvec_next_pt, curr_pt);
  //         add_v2_v2(nvec_next_pt, nvec_next);

  //         tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt);
  //         tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt);

  //         BLI_addtail(perimeter_left_side, normal_prev);
  //         BLI_addtail(perimeter_left_side, normal_next);
  //         num_perimeter_points += 2;

  //         num_perimeter_points += generate_arc_from_point_to_point(
  //             perimeter_left_side, normal_prev, normal_next, curr_pt, subdivisions, true);

  //         if (miter_length < prev_length && miter_length < next_length) {
  //           copy_v3_v3(miter_right_pt, curr_pt);
  //           add_v2_v2(miter_right_pt, vec_miter_right);
  //         }
  //         else {
  //           copy_v3_v3(miter_right_pt, curr_pt);
  //           negate_v2(nvec_next);
  //           add_v2_v2(miter_right_pt, nvec_next);
  //         }

  //         tPerimeterPoint *miter_right = new_perimeter_point(miter_right_pt);
  //         BLI_addtail(perimeter_right_side, miter_right);
  //         num_perimeter_points++;
  //       }
  //       /* bend to the right */
  //       else {
  //         normalize_v2_length(nvec_prev, -radius);
  //         normalize_v2_length(nvec_next, -radius);

  //         copy_v3_v3(nvec_prev_pt, curr_pt);
  //         add_v2_v2(nvec_prev_pt, nvec_prev);

  //         copy_v3_v3(nvec_next_pt, curr_pt);
  //         add_v2_v2(nvec_next_pt, nvec_next);

  //         tPerimeterPoint *normal_prev = new_perimeter_point(nvec_prev_pt);
  //         tPerimeterPoint *normal_next = new_perimeter_point(nvec_next_pt);

  //         BLI_addtail(perimeter_right_side, normal_prev);
  //         BLI_addtail(perimeter_right_side, normal_next);
  //         num_perimeter_points += 2;

  //         num_perimeter_points += generate_arc_from_point_to_point(
  //             perimeter_right_side, normal_prev, normal_next, curr_pt, subdivisions, false);

  //         if (miter_length < prev_length && miter_length < next_length) {
  //           copy_v3_v3(miter_left_pt, curr_pt);
  //           add_v2_v2(miter_left_pt, vec_miter_left);
  //         }
  //         else {
  //           copy_v3_v3(miter_left_pt, curr_pt);
  //           negate_v2(nvec_prev);
  //           add_v2_v2(miter_left_pt, nvec_prev);
  //         }

  //         tPerimeterPoint *miter_left = new_perimeter_point(miter_left_pt);
  //         BLI_addtail(perimeter_left_side, miter_left);
  //         num_perimeter_points++;
  //       }
  //     }
  //   }

  //   /* generate points for end cap */
  //   num_perimeter_points += generate_perimeter_cap(
  //       last_pt, last_prev_pt, last_radius, perimeter_right_side, subdivisions, gps->caps[1]);

  //   /* merge both sides to one list */
  //   BLI_listbase_reverse(perimeter_right_side);
  //   BLI_movelisttolist(perimeter_left_side,
  //                      perimeter_right_side);  // perimeter_left_side contains entire list
  //   ListBase *perimeter_list = perimeter_left_side;

  //   /* close by creating a point close to the first (make a small gap) */
  //   float close_pt[3];
  //   tPerimeterPoint *close_first = (tPerimeterPoint *)perimeter_list->first;
  //   tPerimeterPoint *close_last = (tPerimeterPoint *)perimeter_list->last;
  //   interp_v3_v3v3(close_pt, &close_last->x, &close_first->x, 0.99f);

  //   if (compare_v3v3(close_pt, &close_first->x, FLT_EPSILON) == false) {
  //     tPerimeterPoint *close_p_pt = new_perimeter_point(close_pt);
  //     BLI_addtail(perimeter_list, close_p_pt);
  //     num_perimeter_points++;
  //   }

  //   /* free temp data */
  //   BLI_freelistN(perimeter_right_side);
  //   MEM_freeN(perimeter_right_side);

  //   *r_num_perimeter_points = num_perimeter_points;
  //   return perimeter_list;
}

static void modify_drawing(const GreasePencilOutlineModifierData &omd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  const bke::CurvesGeometry &src_curves = drawing.strokes();
  if (src_curves.curve_num == 0) {
    return;
  }
  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, src_curves, omd.influence, curve_mask_memory);

  //   drawing.strokes_for_write() = create_dashes(pattern_info, src_curves, curves_mask);
  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, omd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_drawing(*omd, *ctx, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  auto *omd = static_cast<GreasePencilOutlineModifierData *>(ptr->data);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "thickness", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "use_keep_shape", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "subdivision", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "sample_length", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "outline_material", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "object", UI_ITEM_NONE, nullptr, ICON_NONE);

  Scene *scene = CTX_data_scene(C);
  if (scene->camera == nullptr) {
    uiItemL(layout, RPT_("Outline requires an active camera"), ICON_ERROR);
  }

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilOutline, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *omd = reinterpret_cast<const GreasePencilOutlineModifierData *>(md);

  BLO_write_struct(writer, GreasePencilOutlineModifierData, omd);
  modifier::greasepencil::write_influence_data(writer, &omd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &omd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilOutline = {
    /*idname*/ "GreasePencilOutline",
    /*name*/ N_("Outline"),
    /*struct_name*/ "GreasePencilOutlineModifierData",
    /*struct_size*/ sizeof(GreasePencilOutlineModifierData),
    /*srna*/ &RNA_GreasePencilOutlineModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_OUTLINE,

    /*copy_data*/ blender::copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ blender::modify_geometry_set,

    /*init_data*/ blender::init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ blender::free_data,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ blender::update_depsgraph,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};
