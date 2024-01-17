/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "BLO_read_write.hh"

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_screen_types.h"

#include "RNA_access.hh"

#include "BKE_colortools.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_lib_query.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "DEG_depsgraph_query.hh"

#include "ED_grease_pencil.hh"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_prototypes.h"

#include "DEG_depsgraph.hh"

#include "MEM_guardedalloc.h"

#include "ED_grease_pencil.hh"

using namespace blender;

static void init_data(ModifierData *md)
{
  GreasePencilSmoothModifierData *gpmd = reinterpret_cast<GreasePencilSmoothModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilSmoothModifierData), modifier);
  modifier::greasepencil::init_influence_data(&gpmd->influence, true);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const GreasePencilSmoothModifierData *gmd =
      reinterpret_cast<const GreasePencilSmoothModifierData *>(md);
  GreasePencilSmoothModifierData *tgmd = reinterpret_cast<GreasePencilSmoothModifierData *>(
      target);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&gmd->influence, &tgmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  GreasePencilSmoothModifierData *mmd = reinterpret_cast<GreasePencilSmoothModifierData *>(md);

  modifier::greasepencil::free_influence_data(&mmd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GreasePencilSmoothModifierData *mmd = reinterpret_cast<GreasePencilSmoothModifierData *>(md);

  modifier::greasepencil::foreach_influence_ID_link(&mmd->influence, ob, walk, user_data);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const GreasePencilSmoothModifierData *mmd = (const GreasePencilSmoothModifierData *)md;

  BLO_write_struct(writer, GreasePencilSmoothModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  GreasePencilSmoothModifierData *mmd = (GreasePencilSmoothModifierData *)md;
  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

static void deform_stroke(ModifierData *md, Depsgraph *depsgraph, Object *ob, GreasePencil *gp)
{
  GreasePencilSmoothModifierData *mmd = reinterpret_cast<GreasePencilSmoothModifierData *>(md);

  const Scene *scene = DEG_get_evaluated_scene(depsgraph);
  GreasePencil &grease_pencil = *gp;

  const int iterations = mmd->step;
  const float influence = mmd->factor;
  const bool keep_shape = (mmd->flag & MOD_GREASE_PENCIL_SMOOTH_KEEP_SHAPE);
  const bool smooth_ends = (mmd->flag & MOD_GREASE_PENCIL_SMOOTH_SMOOTH_ENDS);

  const bool smooth_position = (mmd->flag & MOD_GREASE_PENCIL_SMOOTH_MOD_LOCATION);
  const bool smooth_radius = (mmd->flag & MOD_GREASE_PENCIL_SMOOTH_MOD_THICKNESS);
  const bool smooth_opacity = (mmd->flag & MOD_GREASE_PENCIL_SMOOTH_MOD_STRENGTH);

  if (iterations <= 0 || influence <= 0) {
    return;
  }

  if (!(smooth_position || smooth_radius || smooth_opacity)) {
    return;
  }

  bool changed = false;
  const Array<ed::greasepencil::MutableDrawingInfo> drawings =
      blender::ed::greasepencil::retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
    bke::CurvesGeometry &curves = info.drawing.strokes_for_write();
    if (curves.points_num() == 0) {
      return;
    }

    IndexMaskMemory memory;
    const IndexMask strokes = modifier::greasepencil::get_filtered_stroke_mask(
        ob, curves, mmd->influence, memory);

    if (strokes.is_empty()) {
      return;
    }

    bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
    const OffsetIndices points_by_curve = curves.points_by_curve();
    const VArray<bool> cyclic = curves.cyclic();
    const VArray<bool> point_selection = VArray<bool>::ForSingle(true, curves.points_num());

    if (smooth_position) {
      bke::GSpanAttributeWriter positions = attributes.lookup_for_write_span("position");
      ed::greasepencil::smooth_curve_attribute(points_by_curve,
                                               point_selection,
                                               cyclic,
                                               strokes,
                                               iterations,
                                               influence,
                                               smooth_ends,
                                               keep_shape,
                                               positions.span);
      positions.finish();
      changed = true;
    }
    if (smooth_opacity && info.drawing.opacities().is_span()) {
      bke::GSpanAttributeWriter opacities = attributes.lookup_for_write_span("opacity");
      ed::greasepencil::smooth_curve_attribute(points_by_curve,
                                               point_selection,
                                               cyclic,
                                               strokes,
                                               iterations,
                                               influence,
                                               smooth_ends,
                                               false,
                                               opacities.span);
      opacities.finish();
      changed = true;
    }
    if (smooth_radius && info.drawing.radii().is_span()) {
      bke::GSpanAttributeWriter radii = attributes.lookup_for_write_span("radius");
      ed::greasepencil::smooth_curve_attribute(points_by_curve,
                                               point_selection,
                                               cyclic,
                                               strokes,
                                               iterations,
                                               influence,
                                               smooth_ends,
                                               false,
                                               radii.span);
      radii.finish();
      changed = true;
    }
  });

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  }
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  GreasePencil *gp = geometry_set->get_grease_pencil_for_write();
  if (!gp) {
    return;
  }

  deform_stroke(md, ctx->depsgraph, ctx->object, gp);
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *row, *col;
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  row = uiLayoutRow(layout, true);
  uiItemR(row, ptr, "use_edit_position", UI_ITEM_R_TOGGLE, IFACE_("Position"), ICON_NONE);
  uiItemR(row, ptr, "use_edit_strength", UI_ITEM_R_TOGGLE, IFACE_("Strength"), ICON_NONE);
  uiItemR(row, ptr, "use_edit_thickness", UI_ITEM_R_TOGGLE, IFACE_("Thickness"), ICON_NONE);
  
  /* TODO: UV not implemented yet in GPv3. */
  // uiItemR(row, ptr, "use_edit_uv", UI_ITEM_R_TOGGLE, IFACE_("UV"), ICON_NONE);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "factor", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "step", UI_ITEM_NONE, IFACE_("Repeat"), ICON_NONE);

  col = uiLayoutColumn(layout, false);
  uiLayoutSetActive(col, RNA_boolean_get(ptr, "use_edit_position"));
  uiItemR(col, ptr, "use_keep_shape", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "use_smooth_ends", UI_ITEM_NONE, nullptr, ICON_NONE);

  if (uiLayout *influence_panel = uiLayoutPanel(
          C, layout, "Influence", ptr, "open_influence_panel"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_vertex_group_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilSmooth, panel_draw);
}

ModifierTypeInfo modifierType_GreasePencilSmooth = {
    /*idname*/ "Grease Pencil Smooth Modifier",
    /*name*/ N_("Grease Pencil Smooth Modifier"),
    /*struct_name*/ "GreasePencilSmoothModifierData",
    /*struct_size*/ sizeof(GreasePencilSmoothModifierData),
    /*srna*/ &RNA_GreasePencilSmoothModifier,
    /*type*/ ModifierTypeType::OnlyDeform,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil,
    /*icon*/ ICON_SMOOTHCURVE,

    /*copy_data*/ copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ modify_geometry_set,

    /*init_data*/ init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ free_data,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ blend_write,
    /*blend_read*/ blend_read,
};
