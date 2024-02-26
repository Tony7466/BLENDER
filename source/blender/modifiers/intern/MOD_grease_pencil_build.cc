/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_array.hh"
#include "BLI_hash.h"
#include "BLI_rand.h"
#include "BLI_task.h"

#include "BLT_translation.hh"

#include "BLO_read_write.hh"

#include "DNA_defaults.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_node_types.h" /* For `GeometryNodeCurveSampleMode` */
#include "DNA_object_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_lib_query.hh"
#include "BKE_modifier.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "GEO_extend_curves.hh"
#include "GEO_trim_curves.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *gpmd = reinterpret_cast<GreasePencilBuildModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilBuildModifierData), modifier);
  modifier::greasepencil::init_influence_data(&gpmd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, int flags)
{
  const auto *omd = reinterpret_cast<const GreasePencilBuildModifierData *>(md);
  auto *tomd = reinterpret_cast<GreasePencilBuildModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tomd->influence);

  BKE_modifier_copydata_generic(md, target, flags);
  modifier::greasepencil::copy_influence_data(&omd->influence, &tomd->influence, flags);
}

static void free_data(ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilBuildModifierData *>(md);
  modifier::greasepencil::free_influence_data(&omd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *omd = reinterpret_cast<GreasePencilBuildModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&omd->influence, ob, walk, user_data);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *mmd = reinterpret_cast<const GreasePencilBuildModifierData *>(md);

  BLO_write_struct(writer, GreasePencilBuildModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilBuildModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

static Array<int> points_per_curve_concurrent(const int stroke_count,
                                              const IndexMask &selection,
                                              const OffsetIndices<int> &points_by_curve,
                                              const int transition,
                                              const float factor,
                                              int &out_curves_num,
                                              int &out_points_num)
{
  out_curves_num = out_points_num = 0;
  const float factor_to_keep = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_GROW ?
                                   factor :
                                   (1.0f - factor);
  Array<int> result(stroke_count);
  for (const int i : IndexRange(stroke_count)) {
    const int points = points_by_curve[i].size() * (selection[i] ? factor_to_keep : 1.0f);
    result[i] = points;
    out_points_num += points;
    if (points) {
      out_curves_num++;
    }
  }
  return result;
}

static bke::CurvesGeometry build_concurrent(const bke::CurvesGeometry &curves,
                                            const IndexMask &selection,
                                            const int transition,
                                            const float factor)
{
  int dst_curves_num, dst_points_num;
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  Array<int> points_per_curve = points_per_curve_concurrent(curves.curves_num(),
                                                            selection,
                                                            points_by_curve,
                                                            transition,
                                                            factor,
                                                            dst_curves_num,
                                                            dst_points_num);
  if (dst_curves_num == 0) {
    return {};
  }

  const bool is_vanishing = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_VANISH;

  bke::CurvesGeometry dst_curves(dst_points_num, dst_curves_num);
  Array<int> dst_offsets(dst_curves_num + 1);
  Array<int> dst_to_src_point(dst_points_num);
  int next_curve = 1, next_point = 0;
  for (const int i : IndexRange(curves.curves_num())) {
    if (points_per_curve[i]) {
      dst_offsets[next_curve] = points_per_curve[i];

      const int extra_offset = is_vanishing ? points_by_curve[i].size() - points_per_curve[i] : 0;
      for (const int stroke_point : IndexRange(points_per_curve[i])) {
        dst_to_src_point[next_point] = extra_offset + stroke_point;
      }

      next_curve++;
    }
  }

  OffsetIndices dst_indices = offset_indices::accumulate_counts_to_offsets(dst_offsets);
  dst_curves.offsets_for_write() = dst_offsets;

  const bke::AttributeAccessor attributes = curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  gather_attributes(attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  return dst_curves;
}

static bool points_info_sequential(const int stroke_count,
                                   const IndexMask &selection,
                                   const OffsetIndices<int> &points_by_curve,
                                   const int transition,
                                   const float factor,
                                   int &out_curves_num,
                                   int &out_points_num)
{
  out_curves_num = out_points_num = 0;
  const float factor_to_keep = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_GROW ?
                                   factor :
                                   (1.0f - factor);

  int effective_points_num = 0;
  selection.foreach_index(
      [&](const int index) { effective_points_num += points_by_curve[index].size(); });

  const int untouched_points_num = points_by_curve.total_size() - effective_points_num;
  effective_points_num *= factor_to_keep;
  effective_points_num += untouched_points_num;

  out_points_num = effective_points_num;

  int counted_points_num = 0;
  for (const int i : IndexRange(stroke_count)) {
    if (selection[i] && counted_points_num >= effective_points_num) {
      continue;
    }
    else {
      out_curves_num++;
    }
  }
  return out_curves_num != 0;
}

static bke::CurvesGeometry build_sequential(const bke::CurvesGeometry &curves,
                                            const IndexMask &selection,
                                            const int transition,
                                            const float factor)
{
  int dst_curves_num, dst_points_num;
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  Array<int> points_per_curve = points_info_sequential(curves.curves_num(),
                                                       selection,
                                                       points_by_curve,
                                                       transition,
                                                       factor,
                                                       dst_curves_num,
                                                       dst_points_num);
  if (dst_curves_num == 0) {
    return {};
  }

  const bool is_vanishing = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_VANISH;

  bke::CurvesGeometry dst_curves(dst_points_num, dst_curves_num);
  Array<int> dst_offsets(dst_curves_num + 1);
  Array<int> dst_to_src_point(dst_points_num);

  int next_curve = 1, next_point = 0;
  selection.foreach_index([&](const int stroke) {
    ;
    for (const int point : points_by_curve[stroke]) {
      dst_to_src_point[next_point] = point;
      next_point++;
    }
    dst_offsets[next_curve] = next_point;
    next_curve++;
  });

  bool done_scanning = false;
  IndexMaskMemory memory;
  selection.complement(curves.curves_range(), memory).foreach_index([&](const int stroke) {
    if (done_scanning) {
      return;
    }
    for (const int point : points_by_curve[stroke]) {
      if (next_point >= dst_points_num) {
        done_scanning = true;
        break;
      }
      dst_to_src_point[next_point] = point;
    }
    dst_offsets[next_curve] = next_point;
    next_curve++;
  });

  BLI_assert(next_curve == dst_curves_num);
  BLI_assert(next_point == dst_points_num);

  OffsetIndices dst_indices = offset_indices::accumulate_counts_to_offsets(dst_offsets);
  dst_curves.offsets_for_write() = dst_offsets;

  const bke::AttributeAccessor attributes = curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  gather_attributes(attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  return dst_curves;
}

static void deform_drawing(const ModifierData &md,
                           const Object &ob,
                           bke::greasepencil::Drawing &drawing,
                           const int current_time)
{
  const auto &mmd = reinterpret_cast<const GreasePencilBuildModifierData &>(md);
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

  if (curves.points_num() == 0) {
    return;
  }

  IndexMaskMemory memory;
  const IndexMask selection = modifier::greasepencil::get_filtered_stroke_mask(
      &ob, curves, mmd.influence, memory);

  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                blender::bke::GeometrySet *geometry_set)
{
  auto *mmd = reinterpret_cast<GreasePencilBuildModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }

  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, mmd->influence, mask_memory);
  const Vector<bke::greasepencil::Drawing *> drawings =
      modifier::greasepencil::get_drawings_for_write(
          grease_pencil, layer_mask, grease_pencil.runtime->eval_frame);

  threading::parallel_for_each(drawings, [&](bke::greasepencil::Drawing *drawing) {
    deform_drawing(*md, *ctx->object, *drawing, grease_pencil.runtime->eval_frame);
  });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  const int mode = RNA_enum_get(ptr, "mode");
  int time_mode = RNA_enum_get(ptr, "time_mode");

  uiLayoutSetPropSep(layout, true);

  /* First: Build mode and build settings. */
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, nullptr, ICON_NONE);
  if (mode == MOD_GREASE_PENCIL_BUILD_MODE_SEQUENTIAL) {
    uiItemR(layout, ptr, "transition", UI_ITEM_NONE, nullptr, ICON_NONE);
  }
  if (mode == MOD_GREASE_PENCIL_BUILD_MODE_CONCURRENT) {
    /* Concurrent mode doesn't support MOD_GREASE_PENCIL_BUILD_TIMEMODE_DRAWSPEED, so unset it. */
    if (time_mode == MOD_GREASE_PENCIL_BUILD_TIMEMODE_DRAWSPEED) {
      RNA_enum_set(ptr, "time_mode", MOD_GREASE_PENCIL_BUILD_TIMEMODE_FRAMES);
      time_mode = MOD_GREASE_PENCIL_BUILD_TIMEMODE_FRAMES;
    }
    uiItemR(layout, ptr, "transition", UI_ITEM_NONE, nullptr, ICON_NONE);
  }
  uiItemS(layout);

  /* Second: Time mode and time settings. */

  uiItemR(layout, ptr, "time_mode", UI_ITEM_NONE, nullptr, ICON_NONE);
  if (mode == MOD_GREASE_PENCIL_BUILD_MODE_CONCURRENT) {
    uiItemR(layout, ptr, "concurrent_time_alignment", UI_ITEM_NONE, nullptr, ICON_NONE);
  }
  switch (time_mode) {
    case MOD_GREASE_PENCIL_BUILD_TIMEMODE_DRAWSPEED:
      uiItemR(layout, ptr, "speed_factor", UI_ITEM_NONE, nullptr, ICON_NONE);
      uiItemR(layout, ptr, "speed_maxgap", UI_ITEM_NONE, nullptr, ICON_NONE);
      break;
    case MOD_GREASE_PENCIL_BUILD_TIMEMODE_FRAMES:
      uiItemR(layout, ptr, "length", UI_ITEM_NONE, IFACE_("Frames"), ICON_NONE);
      if (mode != MOD_GREASE_PENCIL_BUILD_MODE_ADDITIVE) {
        uiItemR(layout, ptr, "start_delay", UI_ITEM_NONE, nullptr, ICON_NONE);
      }
      break;
    case MOD_GREASE_PENCIL_BUILD_TIMEMODE_PERCENTAGE:
      uiItemR(layout, ptr, "percentage_factor", UI_ITEM_NONE, nullptr, ICON_NONE);
      break;
    default:
      break;
  }
  uiItemS(layout);
  uiItemR(layout, ptr, "object", UI_ITEM_NONE, nullptr, ICON_NONE);

  /* Some housekeeping to prevent clashes between incompatible
   * options */

  /* Check for incompatible time modifier. */
  Object *ob = static_cast<Object *>(ob_ptr.data);
  auto *md = static_cast<GreasePencilBuildModifierData *>(ptr->data);

  // TODO: Time offset modifier not merged yet:

  // if (BKE_gpencil_modifiers_findby_type(ob, eGpencilModifierType_Time) != nullptr) {
  //   BKE_gpencil_modifier_set_error(md, "Build and Time Offset modifiers are incompatible");
  // }

  if (uiLayout *panel = uiLayoutPanelProp(
          C, layout, ptr, "open_frame_range_panel", "Effective Range"))
  {
    uiLayoutSetPropSep(panel, true);
    uiItemR(
        panel, ptr, "use_restrict_frame_range", UI_ITEM_NONE, IFACE_("Custom Range"), ICON_NONE);

    const bool active = RNA_boolean_get(ptr, "use_restrict_frame_range");
    uiLayout *col = uiLayoutColumn(panel, false);
    uiLayoutSetActive(col, active);
    uiItemR(col, ptr, "frame_start", UI_ITEM_NONE, IFACE_("Start"), ICON_NONE);
    uiItemR(col, ptr, "frame_end", UI_ITEM_NONE, IFACE_("End"), ICON_NONE);
  }

  if (uiLayout *panel = uiLayoutPanelProp(C, layout, ptr, "open_fading_panel", "Fading")) {
    uiLayoutSetPropSep(panel, true);
    uiItemR(panel, ptr, "use_fading", UI_ITEM_NONE, IFACE_("Fade"), ICON_NONE);

    const bool active = RNA_boolean_get(ptr, "use_fading");
    uiLayout *col = uiLayoutColumn(panel, false);
    uiLayoutSetActive(col, active);

    uiItemR(col, ptr, "fade_factor", UI_ITEM_NONE, IFACE_("Factor"), ICON_NONE);

    uiLayout *subcol = uiLayoutColumn(col, true);
    uiItemR(subcol, ptr, "fade_thickness_strength", UI_ITEM_NONE, IFACE_("Thickness"), ICON_NONE);
    uiItemR(subcol, ptr, "fade_opacity_strength", UI_ITEM_NONE, IFACE_("Opacity"), ICON_NONE);

    uiItemPointerR(col,
                   ptr,
                   "dst_vertex_group",
                   &ob_ptr,
                   "vertex_groups",
                   IFACE_("Weight Output"),
                   ICON_NONE);
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
  modifier_panel_register(region_type, eModifierType_GreasePencilBuild, panel_draw);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilBuild = {
    /*idname*/ "GreasePencilBuildModifier",
    /*name*/ N_("Build"),
    /*struct_name*/ "GreasePencilBuildModifierData",
    /*struct_size*/ sizeof(GreasePencilBuildModifierData),
    /*srna*/ &RNA_GreasePencilBuildModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/
    eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_EnableInEditmode |
        eModifierTypeFlag_SupportsEditmode,
    /*icon*/ ICON_MOD_LENGTH,

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
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};
