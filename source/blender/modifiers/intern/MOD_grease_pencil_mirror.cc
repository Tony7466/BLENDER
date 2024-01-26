/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_array_utils.hh"
#include "BLI_hash.h"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.hh"
#include "BLI_rand.h"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_lib_query.hh"
#include "BKE_material.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(mmd, modifier));

  MEMCPY_STRUCT_AFTER(mmd, DNA_struct_default_get(GreasePencilMirrorModifierData), modifier);
  modifier::greasepencil::init_influence_data(&mmd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *mmd = reinterpret_cast<const GreasePencilMirrorModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilMirrorModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&mmd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);
  modifier::greasepencil::free_influence_data(&mmd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);
  walk(user_data, ob, (ID **)&mmd->object, IDWALK_CB_NOP);
  modifier::greasepencil::foreach_influence_ID_link(&mmd->influence, ob, walk, user_data);
}

static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);
  if (mmd->object != nullptr) {
    DEG_add_object_relation(
        ctx->node, mmd->object, DEG_OB_COMP_TRANSFORM, "Grease Pencil Mirror Modifier");
    DEG_add_depends_on_transform_relation(ctx->node, "Grease Pencil Mirror Modifier");
  }
}

// static int count_selected_points(const bke::CurvesGeometry &curves, const IndexMask
// &curves_mask)
// {
//   int dst_point_num = 0;
//   curves_mask.foreach_index([&](const int64_t curve_i) {
//     const IndexRange points = curves.points_by_curve()[curve_i];
//     dst_point_num += int(points.size());
//   });
//   return dst_point_num;
// }

static IndexMask get_points_from_curves_mask(const bke::CurvesGeometry &curves,
                                             const IndexMask &curves_mask,
                                             IndexMaskMemory &memory)
{
  int64_t point_indices_num = 0;
  curves_mask.foreach_index([&](const int64_t curve_i) {
    const IndexRange points = curves.points_by_curve()[curve_i];
    point_indices_num += points.size();
  });
  Array<int64_t> point_indices(point_indices_num);
  curves_mask.foreach_index([&](const int64_t curve_i) {
    const IndexRange points = curves.points_by_curve()[curve_i];
    array_utils::fill_index_range(point_indices.as_mutable_span().slice(points));
    point_indices_num += points.size();
  });
  return IndexMask::from_indices(point_indices.as_span(), memory);
}

static float4x4 get_mirror_matrix(const int /*mirror_flags*/)
{
  return float4x4::identity();
}

/* Index map into original curve arrays. */
static Array<int> build_curves_map(const int curve_num,
                                   const IndexMask &curves_mask,
                                   const int num_mirrored_axes)
{
  /* Selected source points. */
  const int selected_curve_num = curves_mask.size();

  /* Number of copies on top of the base curves. */
  const int mirror_copies = (1 << num_mirrored_axes) - 1;
  const int dst_curve_num = curve_num + selected_curve_num * mirror_copies;

  /* Source indices for curves. */
  Array<int> dst_curve_indices(dst_curve_num);

  /* Initial curves range, copy original curves without mask. */
  IndexRange dst_curves_range = IndexRange(curve_num);
  array_utils::fill_index_range(dst_curve_indices.as_mutable_span().slice(dst_curves_range));

  for ([[maybe_unused]] const int i : IndexRange(mirror_copies)) {
    /* Range of curves and points for the current mirrored part. */
    dst_curves_range = dst_curves_range.after(selected_curve_num);
    /* Copy curves mask indices to the mirrored section. */
    curves_mask.to_indices(dst_curve_indices.as_mutable_span().slice(dst_curves_range));
  }
  return dst_curve_indices;
}

static void modify_drawing(const GreasePencilMirrorModifierData &omd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  const int num_mirrored_axes = ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X) ? 1 : 0) +
                                ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y) ? 1 : 0) +
                                ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z) ? 1 : 0);
  if (num_mirrored_axes == 0) {
    return;
  }

  const bke::CurvesGeometry &src_curves = drawing.strokes();
  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory, points_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, src_curves, omd.influence, curve_mask_memory);
  const IndexMask points_mask = get_points_from_curves_mask(
      src_curves, curves_mask, points_mask_memory);
  /* Selected source points. */
  const int selected_curve_num = curves_mask.size();
  const int selected_point_num = points_mask.size();

  /* Number of copies on top of the base curves. */
  const int mirror_copies = (1 << num_mirrored_axes) - 1;
  const int dst_curve_num = src_curves.curve_num + selected_curve_num * mirror_copies;
  const int dst_point_num = src_curves.point_num + selected_point_num * mirror_copies;

  Array<int> curves_index_map = build_curves_map(
      src_curves.curve_num, curves_mask, num_mirrored_axes);

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);

  /* Source indices for curves. */
  // Array<int> dst_curve_indices(dst_curve_num);

  /* Initial curves range, copy original curves without mask. */
  // IndexRange dst_curves_range = src_curves.curves_range();
  // IndexRange dst_points_range = src_curves.points_range();

  threading::parallel_for(curves_index_map.index_range(), 512, [&](const IndexRange range) {
    for (const int64_t i : range) {
      const int src_curve_i = curves_index_map[i];
    }
  });

  array_utils::fill_index_range(dst_curve_indices.as_mutable_span().slice(dst_curves_range));
  dst_curves.offsets_for_write()
      .slice(dst_curves_range)
      .copy_from(src_curves.offsets().drop_back(1));
  // array_utils::copy(src_curves.positions().slice(dst_curves_range),
  //                   dst_curves.positions_for_write().slice(dst_curves_range));

#if 0

  /* Add mirrored curve ranges, using the selection mask. */
  Vector<float4x4> mirror_matrices;
  mirror_matrices.reserve(mirror_copies);
  for (const int mirror_x : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X) ? 2 : 1)) {
    for (const int mirror_y : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y) ? 2 : 1)) {
      for (const int mirror_z : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z) ? 2 : 1)) {
        if (mirror_x == 0 && mirror_y == 0 && mirror_z == 0) {
          /* Unmirrored original curve and vertices. */
          continue;
        }
        /* Range of curves and points for the current mirrored part. */
        dst_curves_range = dst_curves_range.after(selected_curve_num);
        dst_points_range = dst_points_range.after(selected_point_num);

        /* Copy curves mask indices to the mirrored section. */
        curves_mask.to_indices(dst_curve_indices.as_mutable_span().slice(dst_curves_range));
        curves_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
          dst_curves.offsets_for_write().slice(
              dst_curves_range)[dst_i] = src_curves.offsets()[src_i];
        });
        // points_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i)
        // {
        //   dst_curves.positions_for_write().slice(
        //       dst_points_range)[dst_i] = src_curves.positions()[src_i];
        // });

        // TODO
        mirror_matrices.append(float4x4::identity());
      }
    }
  }

  // IndexMaskMemory curve_mask_new_memory;
  // const IndexMask curve_mask_new = IndexMask::from_indices(dst_curve_indices.as_span(),
  //                                                          curve_mask_new_memory);
  // bke::AnonymousAttributePropagationInfo propagation_info;
  // propagation_info.propagate_all = false;
  // bke::gather_attributes(src_curves.attributes(),
  //                        bke::AttrDomain::Curve,
  //                        propagation_info,
  //                        {},
  //                        dst_curve_indices,
  //                        dst_curves.attributes_for_write());
  bke::gather_attributes_group_to_group(src_curves.attributes(),
                                        bke::AttrDomain::Point,
                                        propagation_info,
                                        {"position"},
                                        src_curves.points_by_curve(),
                                        dst_curves.points_by_curve(),
                                        curve_mask_new,
                                        dst_curves.attributes_for_write());
#endif

  dst_curves.tag_positions_changed();
  dst_curves.tag_topology_changed();
}

#if 0
static void modify_drawing(const GreasePencilMirrorModifierData &omd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  const int num_mirrored_axes = ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X) ? 1 : 0) +
                                ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y) ? 1 : 0) +
                                ((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z) ? 1 : 0);
  if (num_mirrored_axes == 0) {
    return;
  }

  const bke::CurvesGeometry &src_curves = drawing.strokes();
  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory, points_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, src_curves, omd.influence, curve_mask_memory);
  const IndexMask points_mask = get_points_from_curves_mask(
      src_curves, curves_mask, points_mask_memory);
  /* Selected source points. */
  const int selected_curve_num = curves_mask.size();
  const int selected_point_num = points_mask.size();

  /* Number of copies on top of the base curves. */
  const int mirror_copies = (1 << num_mirrored_axes) - 1;
  const int dst_curve_num = src_curves.curve_num + selected_curve_num * mirror_copies;
  const int dst_point_num = src_curves.point_num + selected_point_num * mirror_copies;

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  /* Source indices for curves. */
  Array<int> dst_curve_indices(dst_curve_num);

  /* Initial curves range, copy original curves without mask. */
  IndexRange dst_curves_range = src_curves.curves_range();
  IndexRange dst_points_range = src_curves.points_range();
  array_utils::fill_index_range(dst_curve_indices.as_mutable_span().slice(dst_curves_range));
  dst_curves.offsets_for_write()
      .slice(dst_curves_range)
      .copy_from(src_curves.offsets().drop_back(1));
  // array_utils::copy(src_curves.positions().slice(dst_curves_range),
  //                   dst_curves.positions_for_write().slice(dst_curves_range));

  /* Add mirrored curve ranges, using the selection mask. */
  Vector<float4x4> mirror_matrices;
  mirror_matrices.reserve(mirror_copies);
  for (const int mirror_x : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X) ? 2 : 1)) {
    for (const int mirror_y : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y) ? 2 : 1)) {
      for (const int mirror_z : IndexRange((omd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z) ? 2 : 1)) {
        if (mirror_x == 0 && mirror_y == 0 && mirror_z == 0) {
          /* Unmirrored original curve and vertices. */
          continue;
        }
        /* Range of curves and points for the current mirrored part. */
        dst_curves_range = dst_curves_range.after(selected_curve_num);
        dst_points_range = dst_points_range.after(selected_point_num);

        /* Copy curves mask indices to the mirrored section. */
        curves_mask.to_indices(dst_curve_indices.as_mutable_span().slice(dst_curves_range));
        curves_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i) {
          dst_curves.offsets_for_write().slice(
              dst_curves_range)[dst_i] = src_curves.offsets()[src_i];
        });
        // points_mask.foreach_index(GrainSize(512), [&](const int64_t src_i, const int64_t dst_i)
        // {
        //   dst_curves.positions_for_write().slice(
        //       dst_points_range)[dst_i] = src_curves.positions()[src_i];
        // });

        // TODO
        mirror_matrices.append(float4x4::identity());
      }
    }
  }

  // IndexMaskMemory curve_mask_new_memory;
  // const IndexMask curve_mask_new = IndexMask::from_indices(dst_curve_indices.as_span(),
  //                                                          curve_mask_new_memory);
  // bke::AnonymousAttributePropagationInfo propagation_info;
  // propagation_info.propagate_all = false;
  // bke::gather_attributes(src_curves.attributes(),
  //                        bke::AttrDomain::Curve,
  //                        propagation_info,
  //                        {},
  //                        dst_curve_indices,
  //                        dst_curves.attributes_for_write());
  // bke::gather_attributes_group_to_group(src_curves.attributes(),
  //                                       bke::AttrDomain::Point,
  //                                       propagation_info,
  //                                       {"position"},
  //                                       src_curves.points_by_curve(),
  //                                       dst_curves.points_by_curve(),
  //                                       curve_mask_new,
  //                                       dst_curves.attributes_for_write());

  dst_curves.tag_positions_changed();
  dst_curves.tag_topology_changed();
}
#endif

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, mmd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_drawing(*mmd, *ctx, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  const eUI_Item_Flag toggles_flag = UI_ITEM_R_TOGGLE | UI_ITEM_R_FORCE_BLANK_DECORATE;

  uiLayoutSetPropSep(layout, true);

  uiLayout *row = uiLayoutRowWithHeading(layout, true, IFACE_("Axis"));
  uiItemR(row, ptr, "use_axis_x", toggles_flag, nullptr, ICON_NONE);
  uiItemR(row, ptr, "use_axis_y", toggles_flag, nullptr, ICON_NONE);
  uiItemR(row, ptr, "use_axis_z", toggles_flag, nullptr, ICON_NONE);

  uiItemR(layout, ptr, "object", UI_ITEM_NONE, nullptr, ICON_NONE);

  LayoutPanelState *influence_panel_state = BKE_panel_layout_panel_state_ensure(
      panel, "influence", true);
  PointerRNA influence_state_ptr = RNA_pointer_create(
      nullptr, &RNA_LayoutPanelState, influence_panel_state);
  if (uiLayout *influence_panel = uiLayoutPanel(
          C, layout, "Influence", &influence_state_ptr, "is_open"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilMirror, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *mmd = reinterpret_cast<const GreasePencilMirrorModifierData *>(md);

  BLO_write_struct(writer, GreasePencilMirrorModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMirrorModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilMirror = {
    /*idname*/ "GreasePencilMirror",
    /*name*/ N_("Mirror"),
    /*struct_name*/ "GreasePencilMirrorModifierData",
    /*struct_size*/ sizeof(GreasePencilMirrorModifierData),
    /*srna*/ &RNA_GreasePencilMirrorModifier,
    /*type*/ ModifierTypeType::Constructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_MIRROR,

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
