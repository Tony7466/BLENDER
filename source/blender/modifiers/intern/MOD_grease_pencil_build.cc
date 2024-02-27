/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_array.hh"
#include "BLI_hash.h"
#include "BLI_rand.h"
#include "BLI_sort.hh"
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

#include "GEO_reorder.hh"

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

static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  auto *mmd = reinterpret_cast<GreasePencilArrayModifierData *>(md);
  if (mmd->object != nullptr) {
    DEG_add_object_relation(ctx->node, mmd->object, DEG_OB_COMP_TRANSFORM, "Build Modifier");
  }
  DEG_add_object_relation(ctx->node, ctx->object, DEG_OB_COMP_TRANSFORM, "Build Modifier");
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

static Array<int> points_per_curve_concurrent(const bke::CurvesGeometry &curves,
                                              const IndexMask &selection,
                                              const int time_alignment,
                                              const int transition,
                                              const float factor,
                                              int &out_curves_num,
                                              int &out_points_num)
{
  const int stroke_count = curves.curves_num();
  const OffsetIndices<int> &points_by_curve = curves.points_by_curve();

  curves.ensure_evaluated_lengths();
  float max_length = 0;
  for (const int stroke : curves.curves_range()) {
    const float len = curves.evaluated_lengths_for_curve(stroke, false).last();
    max_length = math::max(max_length, len);
  }

  out_curves_num = out_points_num = 0;
  const float factor_to_keep = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_GROW ?
                                   math::clamp(factor, 0.0f, 1.0f) :
                                   math::clamp(1.0f - factor, 0.0f, 1.0f);

  auto get_factor = [&](const float factor, const int index) {
    const float max_factor = max_length / curves.evaluated_lengths_for_curve(index, false).last();
    if (time_alignment == MOD_GREASE_PENCIL_BUILD_TIMEALIGN_START) {
      return std::clamp(factor * max_factor, 0.0f, 1.0f);
    }
    /* Else: (#MOD_GREASE_PENCIL_BUILD_TIMEALIGN_END). */
    const float min_factor = max_factor - 1.0f;
    const float use_factor = factor * max_factor;
    return std::clamp(use_factor - min_factor, 0.0f, 1.0f);
  };

  Array<bool> select(stroke_count);
  selection.to_bools(select.as_mutable_span());
  Array<int> result(stroke_count);
  for (const int i : IndexRange(stroke_count)) {
    const float local_factor = (select[i] ? get_factor(factor_to_keep, i) : 1.0f);
    const int points = points_by_curve[i].size() * local_factor;
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
                                            const int time_alignment,
                                            const int transition,
                                            const float factor)
{
  int dst_curves_num, dst_points_num;
  Array<int> points_per_curve = points_per_curve_concurrent(
      curves, selection, time_alignment, transition, factor, dst_curves_num, dst_points_num);
  if (dst_curves_num == 0) {
    return {};
  }

  const OffsetIndices<int> points_by_curve = curves.points_by_curve();

  const bool is_vanishing = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_VANISH;

  bke::CurvesGeometry dst_curves(dst_points_num, dst_curves_num);
  Array<int> dst_offsets(dst_curves_num + 1);
  Array<int> dst_to_src_point(dst_points_num);

  dst_offsets[0] = 0;
  int next_curve = 0, next_point = 0;
  for (const int i : IndexRange(curves.curves_num())) {
    if (points_per_curve[i]) {
      dst_offsets[next_curve] = points_per_curve[i];

      const int extra_offset = is_vanishing ? points_by_curve[i].size() - points_per_curve[i] : 0;
      for (const int stroke_point : IndexRange(points_per_curve[i])) {
        if (stroke_point >= points_per_curve[i]) {
          break;
        }
        dst_to_src_point[next_point] = points_by_curve[i].first() + extra_offset + stroke_point;
        next_point++;
      }

      next_curve++;
    }
  }

  offset_indices::accumulate_counts_to_offsets(dst_offsets);
  array_utils::copy(dst_offsets.as_span(), dst_curves.offsets_for_write());

  const bke::AttributeAccessor attributes = curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  gather_attributes(attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  return dst_curves;
}

static void points_info_sequential(const bke::CurvesGeometry &curves,
                                   const IndexMask &selection,
                                   const int transition,
                                   const float factor,
                                   int &out_curves_num,
                                   int &out_points_num)
{
  const int stroke_count = curves.curves_num();
  const OffsetIndices<int> &points_by_curve = curves.points_by_curve();

  out_curves_num = out_points_num = 0;
  const float factor_to_keep = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_GROW ?
                                   factor :
                                   (1.0f - factor);

  const bool is_vanishing = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_VANISH;

  int effective_points_num = 0;
  selection.foreach_index(
      [&](const int index) { effective_points_num += points_by_curve[index].size(); });

  const int untouched_points_num = points_by_curve.total_size() - effective_points_num;
  effective_points_num *= factor_to_keep;
  effective_points_num += untouched_points_num;

  out_points_num = effective_points_num;

  Array<bool> select(stroke_count);
  selection.to_bools(select.as_mutable_span());

  int counted_points_num = 0;
  for (const int i : IndexRange(stroke_count)) {
    const int stroke = is_vanishing ? stroke_count - i - 1 : i;
    if (select[stroke] && counted_points_num >= effective_points_num) {
      continue;
    }
    else {
      counted_points_num += points_by_curve[stroke].size();
      out_curves_num++;
    }
  }
}

static bke::CurvesGeometry build_sequential(const bke::CurvesGeometry &curves,
                                            const IndexMask &selection,
                                            const int transition,
                                            const float factor)
{
  int dst_curves_num, dst_points_num;
  points_info_sequential(curves, selection, transition, factor, dst_curves_num, dst_points_num);
  if (dst_curves_num == 0) {
    return {};
  }

  const OffsetIndices<int> points_by_curve = curves.points_by_curve();

  const bool is_vanishing = transition == MOD_GREASE_PENCIL_BUILD_TRANSITION_VANISH;

  bke::CurvesGeometry dst_curves(dst_points_num, dst_curves_num);
  Array<int> dst_offsets(dst_curves_num + 1);
  Array<int> dst_to_src_point(dst_points_num);

  dst_offsets[0] = 0;

  int next_curve = 1, next_point = 0;
  IndexMaskMemory memory;
  selection.complement(curves.curves_range(), memory).foreach_index([&](const int stroke) {
    for (const int point : points_by_curve[stroke]) {
      dst_to_src_point[next_point] = point;
      next_point++;
    }
    dst_offsets[next_curve] = next_point;
    next_curve++;
  });

  const int stroke_count = curves.curves_num();
  bool done_scanning = false;
  selection.foreach_index([&](const int i) {
    const int stroke = is_vanishing ? stroke_count - i - 1 : i;
    if (done_scanning || next_point >= dst_points_num) {
      done_scanning = true;
      return;
    }
    for (const int point : points_by_curve[stroke]) {
      dst_to_src_point[next_point] = is_vanishing ? points_by_curve[stroke].last() -
                                                        (point - points_by_curve[stroke].first()) :
                                                    point;
      next_point++;
      if (next_point >= dst_points_num) {
        done_scanning = true;
        break;
      }
    }
    dst_offsets[next_curve] = next_point;
    next_curve++;
  });

  BLI_assert(next_curve == (dst_curves_num + 1));
  BLI_assert(next_point == dst_points_num);

  array_utils::copy(dst_offsets.as_span(), dst_curves.offsets_for_write());

  const bke::AttributeAccessor attributes = curves.attributes();
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();

  gather_attributes(attributes, bke::AttrDomain::Point, {}, {}, dst_to_src_point, dst_attributes);

  return dst_curves;
}

static bke::CurvesGeometry reorder_strokes(const bke::CurvesGeometry &curves,
                                           const Span<bool> select,
                                           const Object &object,
                                           MutableSpan<bool> r_selection)
{
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const Span<float3> positions = curves.positions();
  const float3 center = object.object_to_world().location();

  struct _Pair {
    float value;
    int index;
    bool selected;
  };

  Array<_Pair> distances(curves.curves_num());
  threading::parallel_for(curves.curves_range(), 65536, [&](const IndexRange range) {
    for (const int stroke : range) {
      const float3 p1 = positions[points_by_curve[stroke].first()];
      const float3 p2 = positions[points_by_curve[stroke].last()];
      distances[stroke].value = math::max(math::distance(p1, center), math::distance(p2, center));
      distances[stroke].index = stroke;
      distances[stroke].selected = select[stroke];
    }
  });

  parallel_sort(
      distances.begin(), distances.end(), [](_Pair &a, _Pair &b) { return a.value < b.value; });

  Array<int> new_order(curves.curves_num());
  threading::parallel_for(curves.curves_range(), 65536, [&](const IndexRange range) {
    for (const int i : range) {
      new_order[i] = distances[i].index;
      r_selection[i] = distances[i].selected;
    }
  });

  return geometry::reorder_curves_geometry(curves, new_order.as_span(), {});
}

static float get_build_factor(const int time_mode,
                              const int current_frame,
                              const int start_frame,
                              const int length,
                              const float percentage)
{
  if (time_mode == MOD_GREASE_PENCIL_BUILD_TIMEMODE_FRAMES) {
    return math::clamp(float(current_frame - start_frame) / length, 0.0f, 1.0f);
  }
  else if (time_mode == MOD_GREASE_PENCIL_BUILD_TIMEMODE_PERCENTAGE) {
    return percentage;
  }
  else {
    // TODO: Find a way to implement MOD_GREASE_PENCIL_BUILD_TIMEMODE_DRAWSPEED...
    return 0;
  }
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
  IndexMask selection = modifier::greasepencil::get_filtered_stroke_mask(
      &ob, curves, mmd.influence, memory);

  if (mmd.object) {
    const int curves_num = curves.curves_num();
    Array<bool> select(curves_num), reordered_select(curves_num);
    selection.to_bools(select);
    curves = reorder_strokes(
        curves, select.as_span(), *mmd.object, reordered_select.as_mutable_span());
    selection = IndexMask::from_bools(reordered_select, memory);
  }

  const float factor = get_build_factor(
      mmd.time_mode, current_time, mmd.start_delay, mmd.length, mmd.percentage_fac);

  switch (mmd.mode) {
    default:
    case MOD_GREASE_PENCIL_BUILD_MODE_SEQUENTIAL:
      curves = build_sequential(curves, selection, mmd.transition, factor);
      break;
    case MOD_GREASE_PENCIL_BUILD_MODE_CONCURRENT:
      curves = build_concurrent(curves, selection, mmd.time_alignment, mmd.transition, factor);
      break;
    case MOD_GREASE_PENCIL_BUILD_MODE_ADDITIVE:
      // Todo: I'm not sure what this mode means, looks like the same to me.
      // The original code path seems to indicate it will only build "extra stroke"
      // compared to the previous grease pencil frame, but it's by counting strokes
      // only, so it doesn't guarantee matching strokes...?
      curves = build_sequential(curves, selection, mmd.transition, factor);
      break;
  }

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

  const int eval_frame = grease_pencil.runtime->eval_frame;
  if (mmd->flag & MOD_GREASE_PENCIL_BUILD_RESTRICT_TIME) {
    if (eval_frame < mmd->start_frame || eval_frame > mmd->end_frame) {
      return;
    }
  }

  threading::parallel_for_each(drawings, [&](bke::greasepencil::Drawing *drawing) {
    deform_drawing(*md, *ctx->object, *drawing, eval_frame);
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
                   "target_vertex_group",
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
