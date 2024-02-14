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
  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(tmd, modifier));

  MEMCPY_STRUCT_AFTER(tmd, DNA_struct_default_get(GreasePencilTimeModifierData), modifier);
  modifier::greasepencil::init_influence_data(&tmd->influence, false);

  GreasePencilTimeModifierSegment *segment = DNA_struct_default_alloc(
      GreasePencilTimeModifierSegment);
  STRNCPY_UTF8(segment->name, DATA_("Segment"));
  tmd->segments_array = segment;
  tmd->segments_num = 1;
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *tmd = reinterpret_cast<const GreasePencilTimeModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilTimeModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&tmd->influence, &tmmd->influence, flag);

  tmmd->segments_array = static_cast<GreasePencilTimeModifierSegment *>(
      MEM_dupallocN(tmd->segments_array));
}

static void free_data(ModifierData *md)
{
  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);
  modifier::greasepencil::free_influence_data(&tmd->influence);

  MEM_SAFE_FREE(tmd->segments_array);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&tmd->influence, ob, walk, user_data);
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  //   const PatternInfo pattern_info = get_pattern_info(*tmd);

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, tmd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  //   threading::parallel_for_each(
  //       drawings, [&](Drawing *drawing) { modify_drawing(*tmd, *ctx, pattern_info, *drawing);
  //       });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  auto *tmd = static_cast<GreasePencilTimeModifierData *>(ptr->data);
  const auto mode = GreasePencilTimeModifierMode(RNA_enum_get(ptr, "mode"));
  const bool use_fixed_offset = (mode == MOD_GREASE_PENCIL_TIME_MODE_FIX);
  const bool use_custom_range = !ELEM(
      mode, MOD_GREASE_PENCIL_TIME_MODE_FIX, MOD_GREASE_PENCIL_TIME_MODE_CHAIN);
  uiLayout *row, *col;

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, nullptr, ICON_NONE);

  col = uiLayoutColumn(layout, false);

  const char *text = use_fixed_offset ? IFACE_("Frame") : IFACE_("Frame Offset");
  uiItemR(col, ptr, "offset", UI_ITEM_NONE, text, ICON_NONE);

  row = uiLayoutRow(col, false);
  uiLayoutSetActive(row, !use_fixed_offset);
  uiItemR(row, ptr, "frame_scale", UI_ITEM_NONE, IFACE_("Scale"), ICON_NONE);

  row = uiLayoutRow(layout, false);
  uiLayoutSetActive(row, !use_fixed_offset);
  uiItemR(row, ptr, "use_keep_loop", UI_ITEM_NONE, nullptr, ICON_NONE);

  if (mode == MOD_GREASE_PENCIL_TIME_MODE_CHAIN) {
    row = uiLayoutRow(layout, false);
    uiLayoutSetPropSep(row, false);

    uiTemplateList(row,
                   (bContext *)C,
                   "MOD_UL_grease_pencil_time_modifier_segments",
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

    col = uiLayoutColumn(row, false);

    uiLayout *sub = uiLayoutColumn(col, true);
    uiItemO(sub, "", ICON_ADD, "OBJECT_OT_grease_pencil_time_modifier_segment_add");
    uiItemO(sub, "", ICON_REMOVE, "OBJECT_OT_grease_pencil_time_modifier_segment_remove");
    uiItemS(col);
    sub = uiLayoutColumn(col, true);
    uiItemEnumO_string(
        sub, "", ICON_TRIA_UP, "OBJECT_OT_grease_pencil_time_modifier_segment_move", "type", "UP");
    uiItemEnumO_string(sub,
                       "",
                       ICON_TRIA_DOWN,
                       "OBJECT_OT_grease_pencil_time_modifier_segment_move",
                       "type",
                       "DOWN");

    if (tmd->segments().index_range().contains(tmd->segment_active_index)) {
      PointerRNA segment_ptr = RNA_pointer_create(ptr->owner_id,
                                                  &RNA_GreasePencilTimeModifierSegment,
                                                  &tmd->segments()[tmd->segment_active_index]);

      sub = uiLayoutColumn(layout, true);
      uiItemR(sub, &segment_ptr, "segment_mode", UI_ITEM_NONE, nullptr, ICON_NONE);
      sub = uiLayoutColumn(layout, true);
      uiItemR(sub, &segment_ptr, "segment_start", UI_ITEM_NONE, nullptr, ICON_NONE);
      uiItemR(sub, &segment_ptr, "segment_end", UI_ITEM_NONE, nullptr, ICON_NONE);
      uiItemR(sub, &segment_ptr, "segment_repeat", UI_ITEM_NONE, nullptr, ICON_NONE);
    }
  }

  PanelLayout custom_range_panel_layout = uiLayoutPanelProp(
      C, layout, ptr, "open_custom_range_panel");
  if (uiLayout *header = custom_range_panel_layout.header) {
    uiLayoutSetPropSep(header, false);
    uiLayoutSetActive(header, use_custom_range);
    uiItemR(header, ptr, "use_custom_frame_range", UI_ITEM_NONE, nullptr, ICON_NONE);
  }
  if (uiLayout *body = custom_range_panel_layout.body) {
    uiLayoutSetPropSep(body, true);
    uiLayoutSetActive(body, use_custom_range && RNA_boolean_get(ptr, "use_custom_frame_range"));

    col = uiLayoutColumn(body, true);
    uiItemR(col, ptr, "frame_start", UI_ITEM_NONE, IFACE_("Frame Start"), ICON_NONE);
    uiItemR(col, ptr, "frame_end", UI_ITEM_NONE, IFACE_("End"), ICON_NONE);
  }

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
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
  modifier_panel_register(region_type, eModifierType_GreasePencilTime, panel_draw);

  uiListType *list_type = static_cast<uiListType *>(
      MEM_callocN(sizeof(uiListType), "Grease Pencil Time modifier segments"));
  STRNCPY(list_type->idname, "MOD_UL_grease_pencil_time_modifier_segments");
  list_type->draw_item = segment_list_item_draw;
  WM_uilisttype_add(list_type);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *tmd = reinterpret_cast<const GreasePencilTimeModifierData *>(md);

  BLO_write_struct(writer, GreasePencilTimeModifierData, tmd);
  modifier::greasepencil::write_influence_data(writer, &tmd->influence);

  BLO_write_struct_array(
      writer, GreasePencilTimeModifierSegment, tmd->segments_num, tmd->segments_array);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &tmd->influence);

  BLO_read_data_address(reader, &tmd->segments_array);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilTime = {
    /*idname*/ "GreasePencilTime",
    /*name*/ N_("TimeOffset"),
    /*struct_name*/ "GreasePencilTimeModifierData",
    /*struct_size*/ sizeof(GreasePencilTimeModifierData),
    /*srna*/ &RNA_GreasePencilTimeModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_TIME,

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

blender::Span<GreasePencilTimeModifierSegment> GreasePencilTimeModifierData::segments() const
{
  return {this->segments_array, this->segments_num};
}

blender::MutableSpan<GreasePencilTimeModifierSegment> GreasePencilTimeModifierData::segments()
{
  return {this->segments_array, this->segments_num};
}
