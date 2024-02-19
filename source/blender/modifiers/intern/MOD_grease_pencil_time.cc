/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_span.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"
#include "BLI_vector_set.hh"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_ui_common.hh"

#include <iostream>

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

#if 0 
/* Create a timeline for an inverse linear time mapping. */
static void remap_frames_linear(const Map<int, GreasePencilFrame> &src_frames,
                                const Span<int> src_sorted_keys,
                                const FrameRange &dst_frame_range,
                                const int offset,
                                const float scale,
                                Map<int, GreasePencilFrame> &dst_frames)
{
  const bool reverse = scale < 0.0f;

  /* Inverse linear time mapping function. */
  auto time_remap = [=](const int key) { return int((key - offset) / scale); };

  auto insert_key = [&](const int key, const GreasePencilFrame &value) {
    if (key < dst_frame_range.sfra) {
      /* The frame is outside the range but might influence it.
       * In that case a null frame is inserted at the start. */
      // std::cout << "Frame " << (key * scale + offset) << " -> " << key << " outside range ("
      //           << dst_frame_range.sfra << ".." << dst_frame_range.efra << ")" << std::endl;
      dst_frames.add_overwrite(dst_frame_range.sfra, value);
    }
    else if (key <= dst_frame_range.efra) {
      /* Inside the frame range insert as a regular keyframe. */
      // std::cout << "Frame " << (key * scale + offset) << " -> " << key << " inside range ("
      //           << dst_frame_range.sfra << ".." << dst_frame_range.efra << ")" << std::endl;
      dst_frames.add_overwrite(key, value);
    }
  };

  std::cout << "INSERTING" << std::endl;
  for (const int i : src_sorted_keys.index_range()) {
    /* In case of a large scaling factor multiple source frames can map to the same destination
     * frame. Frames must be inserted in the correct order to ensure the last frame always defines
     * the following interval. */

    if (reverse) {
      const int src_key = src_sorted_keys[src_sorted_keys.size() - 1 - i];
      const GreasePencilFrame &src_value = src_frames.lookup(src_key);
      /* Reverse mode requires that frames get pushed to the end of their range
       * in order to cover the preceding interval. */
      const int src_end_key = (i > 0) ? src_sorted_keys[src_sorted_keys.size() - i] - 1 : 0;
      const int dst_key = (i > 0) ? time_remap(src_end_key) : dst_frame_range.sfra;

      std::cout << "Reverse: insert value [" << src_key << ".." << src_end_key << "] -> "
                << dst_key << std::endl;
      insert_key(dst_key, src_value);

      /* For the last frame a new end key is inserted at the original source key to hold the frame
       * only up to that key.
       *
       * src:
       *  ----F>>>>F>>>>>>F>>>>
       * dst:
       *  ---N<<<<F<<<<<<F<<<<F
       *
       * F: key frame
       * N: null frame
       */
      if (i == src_sorted_keys.size() - 1 && !src_value.is_null()) {
        const int dst_clip_key = time_remap(src_key) + 1;
        std::cout << "  insert clip frame at " << dst_clip_key << std::endl;
        insert_key(dst_clip_key, GreasePencilFrame::null());
      }
    }
    else {
      const int src_key = src_sorted_keys[i];
      const GreasePencilFrame &src_value = src_frames.lookup(src_key);
      const int dst_key = time_remap(src_key);
      insert_key(dst_key, src_value);
    }
  }

  { /* DEBUGGING*/
    struct DebugItem {
      int dst_frame;
      int drawing_index;
    };
    blender::Vector<DebugItem> frames_debug;
    for (const auto &item : dst_frames.items()) {
      frames_debug.append({item.key, item.value.drawing_index});
    }
    std::sort(frames_debug.begin(),
              frames_debug.end(),
              [&](const DebugItem &a, const DebugItem &b) { return a.dst_frame < b.dst_frame; });

    std::cout << "RESULT" << std::endl;
    for (const DebugItem &item : frames_debug) {
      std::cout << item.dst_frame << "(" << item.drawing_index << ")" << std::endl;
    }
  } /* DEBUGGING*/
}
#endif

struct FrameRange {
  /* Start frame. */
  int sfra;
  /* End frame (unlimited range when undefined). */
  int efra;
};

/**
 * \param src_interval Start and end of the interval for this keyframe.
 * \param src_keyframe_range Overall range of keyframes in the grease pencil data.
 * \param src_interval Frame range to fill with transformed keyframes.
 */
static void add_keyframe_instances(const GreasePencilFrame &value,
                            const FrameRange &src_interval,
                            const FrameRange &src_keyframe_range,
                            const FrameRange &dst_timeline_range,
                            GreasePencilTimeModifierMode mode,
                            const bool loop,
                            const int offset,
                            const float scale,
                            Map<int, GreasePencilFrame> &dst_frames)
{
  /* Keyframe interval must be contained within the overall range. */
  BLI_assert(src_interval.sfra >= src_keyframe_range.sfra &&
             src_interval.efra <= src_keyframe_range.efra);
  BLI_assert(scale >= 0);
  /* Inverse linear time mapping function. */
  auto time_transform = [=](const int key) { return int((key - offset) / scale); };
  auto has_overlap = [](const FrameRange &a, const FrameRange &b) -> bool {
    return a.efra >= b.sfra && a.sfra <= b.efra;
  };
  //auto try_insert_keyframe_instance = [&](const FrameRange &interval) -> bool {
  //  if (has_overlap(interval, dst_timeline_range)) {
  //    dst_frames.add(interval.sfra, value);
  //  }
  //};

  /* Compute linear transformation first to determine necessary instance range. */
  const FrameRange transformed_interval = {time_transform(src_interval.sfra),
                                           time_transform(src_interval.efra)};
  const FrameRange transformed_keyframe_range = {time_transform(src_keyframe_range.sfra),
                                                 time_transform(src_keyframe_range.efra)};

  switch (mode) {
    case MOD_GREASE_PENCIL_TIME_MODE_NORMAL:
      if (loop) {
        const float fnum = float(dst_timeline_range.sfra - transformed_interval.efra) /
                           float(transformed_keyframe_range.efra -
                                 transformed_keyframe_range.sfra);
      }
      else {
        //try_insert_keyframe_instance(transformed_interval);
      }
      break;
    case MOD_GREASE_PENCIL_TIME_MODE_REVERSE:
      break;
    case MOD_GREASE_PENCIL_TIME_MODE_FIX:
      break;
    case MOD_GREASE_PENCIL_TIME_MODE_PINGPONG:
      break;
    case MOD_GREASE_PENCIL_TIME_MODE_CHAIN:
      break;
  }
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  auto *tmd = reinterpret_cast<GreasePencilTimeModifierData *>(md);
  const Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  const auto mode = GreasePencilTimeModifierMode(tmd->mode);
  const bool use_custom_range = tmd->flag & MOD_GREASE_PENCIL_TIME_CUSTOM_RANGE;
  const bool use_loop = tmd->flag & MOD_GREASE_PENCIL_TIME_KEEP_LOOP;
  const FrameRange src_keyframe_range = {scene->r.sfra, scene->r.efra};
  const FrameRange dst_keyframe_range = use_custom_range ? FrameRange{tmd->sfra, tmd->efra} :
                                                           src_keyframe_range;
  /* XXX This is wrong, should subtract src_keyframe_range.sfra instead of 1.
   * But GPv2 does this, so keep it. */
  const int shift = dst_keyframe_range.sfra - 1;

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  // const int frame = grease_pencil.runtime->eval_frame;

  //   const PatternInfo pattern_info = get_pattern_info(*tmd);

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, tmd->influence, mask_memory);

  for (const int64_t i : layer_mask.index_range()) {
    Layer *layer = grease_pencil.layers_for_write()[layer_mask[i]];
    const Span<int> sorted_keys = layer->sorted_keys();
    const Map<int, GreasePencilFrame> &src_frames = layer->frames();

    Map<int, GreasePencilFrame> new_frames;
    for (const int i : sorted_keys.index_range()) {
      const int sfra = sorted_keys[i];
      const int efra = (i < sorted_keys.size() - 1) ? sorted_keys[i + 1] - 1 :
                                                      src_keyframe_range.efra;
      const FrameRange src_keyframe_interval = {sfra, efra};
      add_keyframe_instances(src_frames.lookup(sfra),
                             {sfra, efra},
                             src_keyframe_range,
                             dst_keyframe_range,
                             mode,
                             use_loop,
                             tmd->offset,
                             tmd->frame_scale,
                             new_frames);
    }
#if 0
    switch (GreasePencilTimeModifierMode(tmd->mode)) {
      case MOD_GREASE_PENCIL_TIME_MODE_NORMAL: {
        if (use_loop) {
          const FrameRange loop_range =
        }
        else {
          remap_frames_linear(layer->frames(),
                              layer->sorted_keys(),
                              dst_range,
                              shift + tmd->offset,
                              tmd->frame_scale,
                              new_frames);
        }
        break;
      }
      case MOD_GREASE_PENCIL_TIME_MODE_REVERSE: {
        remap_frames_linear(layer->frames(),
                            layer->sorted_keys(),
                            dst_range,
                            dst_range.efra + shift + tmd->offset,
                            -tmd->frame_scale,
                            new_frames);
        break;
      }
      case MOD_GREASE_PENCIL_TIME_MODE_FIX:
        break;
      case MOD_GREASE_PENCIL_TIME_MODE_PINGPONG: {
        // const int periods = 3;
        // IndexRange dst_range();
        // for (const int i : IndexRange(periods)) {
        //   remap_frames_linear(layer->frames(),
        //                       layer->sorted_keys(),
        //                       dst_range,
        //                       dst_range.efra + shift + tmd->offset,
        //                       -tmd->frame_scale,
        //                       new_frames);
        // }
        break;
      }
      case MOD_GREASE_PENCIL_TIME_MODE_CHAIN:
        break;
    }
#endif
    layer->frames_for_write() = std::move(new_frames);

    layer->tag_frames_map_keys_changed();
  }

  // const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
  //     grease_pencil, layer_mask, frame);
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
