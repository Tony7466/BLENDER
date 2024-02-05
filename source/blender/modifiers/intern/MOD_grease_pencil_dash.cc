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

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_lib_query.hh"
#include "BKE_material.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include <iostream>

namespace blender {

static void init_data(ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(dmd, modifier));

  MEMCPY_STRUCT_AFTER(dmd, DNA_struct_default_get(GreasePencilDashModifierData), modifier);
  modifier::greasepencil::init_influence_data(&dmd->influence, false);

  GreasePencilDashModifierSegment *ds = DNA_struct_default_alloc(GreasePencilDashModifierSegment);
  STRNCPY_UTF8(ds->name, DATA_("Segment"));
  dmd->segments_array = ds;
  dmd->segments_num = 1;
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *dmd = reinterpret_cast<const GreasePencilDashModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilDashModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&dmd->influence, &tmmd->influence, flag);

  tmmd->segments_array = static_cast<GreasePencilDashModifierSegment *>(
      MEM_dupallocN(dmd->segments_array));
}

static void free_data(ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  modifier::greasepencil::free_influence_data(&dmd->influence);

  MEM_SAFE_FREE(dmd->segments_array);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&dmd->influence, ob, walk, user_data);
}

static bool is_disabled(const Scene * /*scene*/, ModifierData *md, bool /*use_render_params*/)
{
  const auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  /* Enable if at least one segment has non-zero length. */
  for (const GreasePencilDashModifierSegment &dash_segment : dmd->segments()) {
    if (dash_segment.dash + dash_segment.gap - 1 > 0) {
      return false;
    }
  }
  return true;
}

static int floored_modulo(const int a, const int b)
{
  return a - math::floor(float(a) / float(b)) * b;
}

/* Combined segment info used by all strokes. */
struct PatternInfo {
  /* Segment indices per point over the length of the pattern.
   * -1: Gap, discard point.
   * >= 0: Segment index, same indices form a curve.
   */
  Array<int> dash_ids;
  int dash_offset = 0;

  int point_num = 0;
  int curve_num = 0;
};

static PatternInfo get_pattern_info(const GreasePencilDashModifierData &dmd)
{
  int pattern_length = 0;
  for (const GreasePencilDashModifierSegment &dash_segment : dmd.segments()) {
    pattern_length += dash_segment.dash + dash_segment.gap;
  }

  PatternInfo info;
  info.dash_ids.reinitialize(pattern_length);
  info.dash_offset = floored_modulo(dmd.dash_offset, pattern_length);
  info.curve_num = dmd.segments().size();

  /* Bake segments into a single pattern array for simplicity. */
  IndexRange dash_range(0);
  IndexRange gap_range(0);
  for (const int i : dmd.segments().index_range()) {
    const GreasePencilDashModifierSegment &dash_segment = dmd.segments()[i];
    dash_range = gap_range.after(dash_segment.dash);
    gap_range = dash_range.after(dash_segment.gap);
    info.dash_ids.as_mutable_span().slice(dash_range).fill(i);
    info.dash_ids.as_mutable_span().slice(gap_range).fill(-1);

    info.point_num += dash_range.size();
  }
  return info;
}

/**
 * Iterate over all dash curves.
 * \param fn: Function taking an \a IndexMask of source points, describing new curves.
 */
template<typename Fn>
static void foreach_dash(const PatternInfo &pattern_info,
                         const IndexRange src_points,
                         const bool cyclic,
                         Fn fn)
{
  BLI_assert(!src_points.is_empty());

  const Span<int> dash_ids = pattern_info.dash_ids;
  const int pattern_length = dash_ids.size();
  /* Points range in "pattern space". */
  const IndexRange points = {pattern_info.dash_offset, src_points.size()};

  int prev_dash_id = -1;
  IndexRange src_curve_points(0);
  for (const int i : points.index_range()) {
    const int repeat = points[i] / pattern_length;
    const int pattern_i = points[i] - repeat * pattern_length;
    BLI_assert(pattern_i < pattern_length);
    /* Unique ID, to ensure curve breaks in case there is just one dash. */
    const int dash_id = (dash_ids[pattern_i] >= 0 ?
                             dash_ids[pattern_i] + repeat * pattern_info.curve_num :
                             -1);
    if (dash_id >= 0 && dash_id == prev_dash_id) {
      /* Extend curve. */
      src_curve_points = IndexRange(src_curve_points.start(), src_curve_points.size() + 1);
    }
    else {
      if (!src_curve_points.is_empty()) {
        /* Report finished curve. */
        fn(IndexMask(src_curve_points.shift(src_points.start())));
      }

      /* New curve. */
      src_curve_points = (dash_id >= 0 ? IndexRange(i, 1) : IndexRange(0));
    }

    prev_dash_id = dash_id;
  }
}

static bke::CurvesGeometry create_dashes(const GreasePencilDashModifierData &dmd,
                                         const PatternInfo &pattern_info,
                                         const bke::CurvesGeometry &src_curves,
                                         const IndexMask &curves_mask)
{
  const bke::AttributeAccessor src_attributes = src_curves.attributes();
  const VArray<bool> src_cyclic = *src_attributes.lookup_or_default(
      "cyclic", bke::AttrDomain::Curve, false);

  /* Count new curves and points. */
  int dst_point_num = 0;
  int dst_curve_num = 0;
  for (const int src_curve_i : src_curves.curves_range()) {
    const IndexRange src_points = src_curves.points_by_curve()[src_curve_i];

    foreach_dash(
        pattern_info, src_points, src_cyclic[src_curve_i], [&](const IndexMask &src_points) {
          dst_point_num += src_points.size();
          dst_curve_num += 1;
        });
  }

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  /* Map each destination point and curve to its source. */
  Array<int> src_point_indices(dst_point_num);
  Array<int> src_curve_indices(dst_curve_num);

  std::cout << "Dash Curves (points=" << dst_point_num << ", curves=" << dst_curve_num << ")"
            << std::endl;
  {
    /* Start at curve offset and add points for each dash. */
    IndexRange dst_point_range(0);
    int dst_curve_i = 0;
    for (const int src_curve_i : src_curves.curves_range()) {
      const IndexRange src_points = src_curves.points_by_curve()[src_curve_i];
      std::cout << "Original curve " << src_curve_i << " (" << src_points.size() << " points)"
                << std::endl;
      foreach_dash(
          pattern_info, src_points, src_cyclic[src_curve_i], [&](const IndexMask &src_points) {
            {
              std::cout << "Dash source points: ";
              Array<int> src_indices(src_points.size());
              src_points.to_indices(src_indices.as_mutable_span());
              for (const int i : src_indices) {
                std::cout << i << ", ";
              }
              std::cout << std::endl;
            }

            dst_point_range = dst_point_range.after(src_points.size());
            dst_curves.offsets_for_write()[dst_curve_i] = dst_point_range.start();
            // std::cout << dst_curve_i << ": " << dst_point_range << std::endl;

            src_points.to_indices(src_point_indices.as_mutable_span().slice(dst_point_range));
            src_curve_indices[dst_curve_i] = src_curve_i;

            ++dst_curve_i;
          });
    }
    if (dst_curve_i > 0) {
      /* Last offset entry is total point count. */
      dst_curves.offsets_for_write()[dst_curve_i] = dst_point_range.one_after_last();
    }
    // std::cout << dst_curve_i << ": " << dst_point_range.one_after_last() << std::endl;
  }
  std::cout << "Dash Points: " << std::endl;
  for (const int i : src_point_indices.index_range()) {
    std::cout << i << " <- " << src_point_indices[i] << std::endl;
  }
  std::cout << "Dash Curves: " << std::endl;
  for (const int i : src_curve_indices.index_range()) {
    std::cout << i << " <- " << src_curve_indices[i] << std::endl;
  }
  std::flush(std::cout);

  bke::gather_attributes(
      src_attributes, bke::AttrDomain::Point, {}, {}, src_point_indices, dst_attributes);
  bke::gather_attributes(
      src_attributes, bke::AttrDomain::Curve, {}, {}, src_curve_indices, dst_attributes);

  dst_curves.update_curve_types();

  return std::move(dst_curves);
}

static void modify_drawing(const GreasePencilDashModifierData &dmd,
                           const ModifierEvalContext &ctx,
                           const PatternInfo &pattern_info,
                           bke::greasepencil::Drawing &drawing)
{
  UNUSED_VARS(dmd, ctx);
  const bke::CurvesGeometry &src_curves = drawing.strokes();
  if (src_curves.curve_num == 0) {
    return;
  }
  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, src_curves, dmd.influence, curve_mask_memory);

  drawing.strokes_for_write() = create_dashes(dmd, pattern_info, src_curves, curves_mask);
  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  const PatternInfo pattern_info = get_pattern_info(*dmd);

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, dmd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(
      drawings, [&](Drawing *drawing) { modify_drawing(*dmd, *ctx, pattern_info, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  auto *dmd = static_cast<GreasePencilDashModifierData *>(ptr->data);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "dash_offset", UI_ITEM_NONE, nullptr, ICON_NONE);

  uiLayout *row = uiLayoutRow(layout, false);
  uiLayoutSetPropSep(row, false);

  uiTemplateList(row,
                 (bContext *)C,
                 "MOD_UL_grease_pencil_dash_modifier_segments",
                 "",
                 ptr,
                 "segments",
                 ptr,
                 "segment_active_index",
                 nullptr,
                 3,
                 10,
                 0,
                 1,
                 UI_TEMPLATE_LIST_FLAG_NONE);

  uiLayout *col = uiLayoutColumn(row, false);
  uiLayout *sub = uiLayoutColumn(col, true);
  uiItemO(sub, "", ICON_ADD, "OBJECT_OT_grease_pencil_dash_modifier_segment_add");
  uiItemO(sub, "", ICON_REMOVE, "OBJECT_OT_grease_pencil_dash_modifier_segment_remove");
  uiItemS(col);
  sub = uiLayoutColumn(col, true);
  uiItemEnumO_string(
      sub, "", ICON_TRIA_UP, "OBJECT_OT_grease_pencil_dash_modifier_segment_move", "type", "UP");
  uiItemEnumO_string(sub,
                     "",
                     ICON_TRIA_DOWN,
                     "OBJECT_OT_grease_pencil_dash_modifier_segment_move",
                     "type",
                     "DOWN");

  if (dmd->segment_active_index >= 0 && dmd->segment_active_index < dmd->segments_num) {
    PointerRNA ds_ptr = RNA_pointer_create(ptr->owner_id,
                                           &RNA_GreasePencilDashModifierSegment,
                                           &dmd->segments()[dmd->segment_active_index]);

    sub = uiLayoutColumn(layout, true);
    uiItemR(sub, &ds_ptr, "dash", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(sub, &ds_ptr, "gap", UI_ITEM_NONE, nullptr, ICON_NONE);

    sub = uiLayoutColumn(layout, false);
    uiItemR(sub, &ds_ptr, "radius", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(sub, &ds_ptr, "opacity", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(sub, &ds_ptr, "material_index", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(sub, &ds_ptr, "use_cyclic", UI_ITEM_NONE, nullptr, ICON_NONE);
  }

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void segment_list_item_draw(uiList * /*ui_list*/,
                                   const bContext * /*C*/,
                                   uiLayout *layout,
                                   PointerRNA * /*idataptr*/,
                                   PointerRNA *itemptr,
                                   int /*icon*/,
                                   PointerRNA * /*active_dataptr*/,
                                   const char * /*active_propname*/,
                                   int /*index*/,
                                   int /*flt_flag*/)
{
  uiLayout *row = uiLayoutRow(layout, true);
  uiItemR(row, itemptr, "name", UI_ITEM_R_NO_BG, "", ICON_NONE);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilDash, panel_draw);

  uiListType *list_type = static_cast<uiListType *>(
      MEM_callocN(sizeof(uiListType), "Grease Pencil Dash modifier segments"));
  STRNCPY(list_type->idname, "MOD_UL_grease_pencil_dash_modifier_segments");
  list_type->draw_item = segment_list_item_draw;
  WM_uilisttype_add(list_type);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *dmd = reinterpret_cast<const GreasePencilDashModifierData *>(md);

  BLO_write_struct(writer, GreasePencilDashModifierData, dmd);
  modifier::greasepencil::write_influence_data(writer, &dmd->influence);

  BLO_write_struct_array(
      writer, GreasePencilDashModifierSegment, dmd->segments_num, dmd->segments_array);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &dmd->influence);

  BLO_read_data_address(reader, &dmd->segments_array);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilDash = {
    /*idname*/ "GreasePencilDash",
    /*name*/ N_("Dot Dash"),
    /*struct_name*/ "GreasePencilDashModifierData",
    /*struct_size*/ sizeof(GreasePencilDashModifierData),
    /*srna*/ &RNA_GreasePencilDashModifierData,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_DASH,

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
    /*is_disabled*/ blender::is_disabled,
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

blender::Span<GreasePencilDashModifierSegment> GreasePencilDashModifierData::segments() const
{
  return {this->segments_array, this->segments_num};
}

blender::MutableSpan<GreasePencilDashModifierSegment> GreasePencilDashModifierData::segments()
{
  return {this->segments_array, this->segments_num};
}
