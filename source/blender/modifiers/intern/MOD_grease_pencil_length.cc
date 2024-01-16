/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_hash.h"
#include "BLI_utildefines.h"
#include "BLI_task.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector_types.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_string_ref.hh"

#include "BLT_translation.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"
#include "DNA_material_types.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_defaults.h"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_geometry_set.hh"
#include "BKE_lib_query.h"
#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "ED_grease_pencil.hh"

#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_prototypes.h"
#include "RNA_access.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "GEO_stretch_curves.hh"

using namespace blender;

static void init_data(ModifierData *md)
{
  GreasePencilLengthModifierData *gpmd = (GreasePencilLengthModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilLengthModifierData), modifier);
}

static void copy_data(const ModifierData *md, ModifierData *target, int flags)
{
  BKE_modifier_copydata_generic(md, target, flags);
}

static float *noise_table(int len, int offset, int seed)
{
  float *table = static_cast<float *>(MEM_callocN(sizeof(float) * len, __func__));
  for (int i = 0; i < len; i++) {
    table[i] = BLI_hash_int_01(BLI_hash_int_2d(seed, i + offset + 1));
  }
  return table;
}

BLI_INLINE float table_sample(float *table, float x)
{
  return interpf(table[int(ceilf(x))], table[int(floor(x))], fractf(x));
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                blender::bke::GeometrySet *geometry_set)
{
    GreasePencilLengthModifierData* mmd=(GreasePencilLengthModifierData*) md;
    GreasePencil *gp=geometry_set->get_grease_pencil_for_write();
    if (!gp){ return; }

    Scene* scene = DEG_get_evaluated_scene(ctx->depsgraph);
    
    const Array<ed::greasepencil::MutableDrawingInfo> drawings = ed::greasepencil::retrieve_editable_drawings(*scene, *gp);
    threading::parallel_for_each(drawings, [&](const ed::greasepencil::MutableDrawingInfo &info) {
        bke::CurvesGeometry &curves = info.drawing.strokes_for_write();
        if (curves.points_num() == 0) {
            return;
        }

        const int cnum = curves.curves_num();
        const VArray<bool> all_true=VArray<bool>::ForSingle(true,cnum);
        IndexMaskMemory memory;
        IndexMask selection(cnum); selection.from_bools(all_true,memory);

        curves = blender::geometry::stretch_curves(curves,selection,
            std::move(VArray<float>::ForSingle(mmd->start_fac,cnum)),
            std::move(VArray<float>::ForSingle(mmd->end_fac,cnum)),
            std::move(VArray<float>::ForSingle(mmd->overshoot_fac,cnum)),
            std::move(VArray<bool>::ForSingle(mmd->flag&GP_LENGTH_USE_CURVATURE,cnum)),
            std::move(VArray<int>::ForSingle(mmd->start_fac/mmd->point_density,cnum)),
            std::move(VArray<float>::ForSingle(mmd->segment_influence,cnum)),
            std::move(VArray<float>::ForSingle(mmd->max_angle,cnum)),
            std::move(VArray<bool>::ForSingle(mmd->flag&GP_LENGTH_INVERT_CURVATURE,cnum)),{});

        info.drawing.tag_topology_changed();
    });


}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GreasePencilLengthModifierData *mmd = (GreasePencilLengthModifierData *)md;

  walk(user_data, ob, (ID **)&mmd->material, IDWALK_CB_USER);
}

static void random_header_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiItemR(layout, ptr, "use_random", UI_ITEM_NONE, IFACE_("Randomize"), ICON_NONE);
}

static void random_panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);

  uiLayoutSetActive(layout, RNA_boolean_get(ptr, "use_random"));

  uiItemR(layout, ptr, "step", UI_ITEM_NONE, nullptr, ICON_NONE);
}

static void offset_panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);
  uiLayoutSetPropSep(layout, true);
  uiItemR(
      layout, ptr, "random_start_factor", UI_ITEM_NONE, IFACE_("Random Offset Start"), ICON_NONE);
  uiItemR(layout, ptr, "random_end_factor", UI_ITEM_NONE, IFACE_("Random Offset End"), ICON_NONE);
  uiItemR(layout, ptr, "random_offset", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "seed", UI_ITEM_NONE, nullptr, ICON_NONE);
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);
  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, nullptr, ICON_NONE);

  uiLayout *col = uiLayoutColumn(layout, true);

  if (RNA_enum_get(ptr, "mode") == GP_LENGTH_RELATIVE) {
    uiItemR(col, ptr, "start_factor", UI_ITEM_NONE, IFACE_("Start"), ICON_NONE);
    uiItemR(col, ptr, "end_factor", UI_ITEM_NONE, IFACE_("End"), ICON_NONE);
  }
  else {
    uiItemR(col, ptr, "start_length", UI_ITEM_NONE, IFACE_("Start"), ICON_NONE);
    uiItemR(col, ptr, "end_length", UI_ITEM_NONE, IFACE_("End"), ICON_NONE);
  }

  uiItemR(layout, ptr, "overshoot_factor", UI_ITEM_R_SLIDER, IFACE_("Used Length"), ICON_NONE);

  modifier_panel_end(layout, ptr);
}

static void mask_panel_draw(const bContext * /*C*/, Panel *panel)
{
  //modifier_masking_panel_draw(panel, true, false);
}

static void curvature_header_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiItemR(layout, ptr, "use_curvature", UI_ITEM_NONE, IFACE_("Curvature"), ICON_NONE);
}

static void curvature_panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);

  uiLayout *col = uiLayoutColumn(layout, false);

  uiLayoutSetActive(col, RNA_boolean_get(ptr, "use_curvature"));

  uiItemR(col, ptr, "point_density", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "segment_influence", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "max_angle", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "invert_curvature", UI_ITEM_NONE, IFACE_("Invert"), ICON_NONE);
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_GreasePencilLength, panel_draw);
  modifier_subpanel_register(
      region_type, "curvature", "", curvature_header_draw, curvature_panel_draw, panel_type);
  PanelType *offset_panel = modifier_subpanel_register(
      region_type, "offset", "Random Offsets", nullptr, offset_panel_draw, panel_type);
  modifier_subpanel_register(
      region_type, "randomize", "", random_header_draw, random_panel_draw, offset_panel);
  modifier_subpanel_register(
      region_type, "mask", "Influence", nullptr, mask_panel_draw, panel_type);
}
ModifierTypeInfo modifierType_GreasePencilLength = {
    /*idname*/ "Grease Pencil Length Modifier",
    /*name*/ N_("Length Modifier"),
    /*struct_name*/ "GreasePencilLengthModifierData",
    /*struct_size*/ sizeof(GreasePencilLengthModifierData),
    /*srna*/ &RNA_GreasePencilLengthModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil,
    /*icon*/ ICON_MOD_LENGTH,

    /*copy_data*/ copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ modify_geometry_set,

    /*init_data*/ init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ nullptr,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ nullptr,
    /*blend_read*/ nullptr,
};
