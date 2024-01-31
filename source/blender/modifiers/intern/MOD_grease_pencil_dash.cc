/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_string.h"

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

namespace blender {

static void init_data(ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(dmd, modifier));

  MEMCPY_STRUCT_AFTER(dmd, DNA_struct_default_get(GreasePencilDashModifierData), modifier);
  modifier::greasepencil::init_influence_data(&dmd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *dmd = reinterpret_cast<const GreasePencilDashModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilDashModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&dmd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  modifier::greasepencil::free_influence_data(&dmd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&dmd->influence, ob, walk, user_data);
}

/**
 * Gap==0 means to start the next segment at the immediate next point, which will leave a visual
 * gap of "1 point". This makes the algorithm give the same visual appearance as displayed on the
 * UI and also simplifies the check for "no-length" situation where SEG==0 (which will not produce
 * any effective dash).
 */
static int real_gap(const GreasePencilDashModifierSegment &dash_segment)
{
  return dash_segment.gap - 1;
}

static bool is_disabled(const Scene * /*scene*/, ModifierData *md, bool /*use_render_params*/)
{
  const auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);
  /* Enable if at least one segment has non-zero length. */
  for (const GreasePencilDashModifierSegment &dash_segment : dmd->segments()) {
    if (dash_segment.dash + real_gap(dash_segment) > 0) {
      return false;
    }
  }
  return true;
}

// static bke::CurvesGeometry create_mirror_copies(const Object &ob,
//                                                 const GreasePencilDashModifierData &dmd,
//                                                 const bke::CurvesGeometry &base_curves,
//                                                 const bke::CurvesGeometry &mirror_curves)
// {
//   const bool use_mirror_x = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X);
//   const bool use_mirror_y = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y);
//   const bool use_mirror_z = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z);

//   Curves *base_curves_id = bke::curves_new_nomain(base_curves);
//   Curves *mirror_curves_id = bke::curves_new_nomain(mirror_curves);
//   bke::GeometrySet base_geo = bke::GeometrySet::from_curves(base_curves_id);
//   bke::GeometrySet mirror_geo = bke::GeometrySet::from_curves(mirror_curves_id);

//   std::unique_ptr<bke::Instances> instances = std::make_unique<bke::Instances>();
//   const int base_handle = instances->add_reference(bke::InstanceReference{base_geo});
//   const int mirror_handle = instances->add_reference(bke::InstanceReference{mirror_geo});
//   for (const int mirror_x : IndexRange(use_mirror_x ? 2 : 1)) {
//     for (const int mirror_y : IndexRange(use_mirror_y ? 2 : 1)) {
//       for (const int mirror_z : IndexRange(use_mirror_z ? 2 : 1)) {
//         if (mirror_x == 0 && mirror_y == 0 && mirror_z == 0) {
//           instances->add_instance(base_handle, float4x4::identity());
//         }
//         else {
//           const float4x4 matrix = get_mirror_matrix(
//               ob, dmd, bool(mirror_x), bool(mirror_y), bool(mirror_z));
//           instances->add_instance(mirror_handle, matrix);
//         }
//       }
//     }
//   }

//   geometry::RealizeInstancesOptions options;
//   options.keep_original_ids = true;
//   options.realize_instance_attributes = false;
//   options.propagation_info = {};
//   bke::GeometrySet result_geo = geometry::realize_instances(
//       bke::GeometrySet::from_instances(instances.release()), options);
//   return std::move(result_geo.get_curves_for_write()->geometry.wrap());
// }

static void modify_drawing(const GreasePencilDashModifierData &dmd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  UNUSED_VARS(dmd, ctx);
  //   const bool use_mirror_x = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_X);
  //   const bool use_mirror_y = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Y);
  //   const bool use_mirror_z = (dmd.flag & MOD_GREASE_PENCIL_MIRROR_AXIS_Z);
  //   if (!use_mirror_x && !use_mirror_y && !use_mirror_z) {
  //     return;
  //   }

  //   const bke::CurvesGeometry &src_curves = drawing.strokes();
  //   if (src_curves.curve_num == 0) {
  //     return;
  //   }
  //   /* Selected source curves. */
  //   IndexMaskMemory curve_mask_memory;
  //   const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
  //       ctx.object, src_curves, dmd.influence, curve_mask_memory);

  //   if (curves_mask.size() == src_curves.curve_num) {
  //     /* All geometry gets mirrored. */
  //     drawing.strokes_for_write() = create_mirror_copies(*ctx.object, dmd, src_curves,
  //     src_curves);
  //   }
  //   else {
  //     /* Create masked geometry, then mirror it. */
  //     bke::CurvesGeometry masked_curves = bke::curves_copy_curve_selection(
  //         src_curves, curves_mask, {});

  //     drawing.strokes_for_write() = create_mirror_copies(
  //         *ctx.object, dmd, src_curves, masked_curves);
  //   }

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

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, dmd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_drawing(*dmd, *ctx, *drawing); });
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
  uiItemO(sub, "", ICON_ADD, "GPENCIL_OT_segment_add");
  uiItemO(sub, "", ICON_REMOVE, "GPENCIL_OT_segment_remove");
  uiItemS(col);
  sub = uiLayoutColumn(col, true);
  uiItemEnumO_string(sub, "", ICON_TRIA_UP, "GPENCIL_OT_segment_move", "type", "UP");
  uiItemEnumO_string(sub, "", ICON_TRIA_DOWN, "GPENCIL_OT_segment_move", "type", "DOWN");

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

  if (uiLayout *influence_panel = uiLayoutPanel(
          C, layout, "Influence", ptr, "open_influence_panel"))
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
  modifier_panel_register(region_type, eModifierType_GreasePencilMirror, panel_draw);

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
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *dmd = reinterpret_cast<GreasePencilDashModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &dmd->influence);
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
