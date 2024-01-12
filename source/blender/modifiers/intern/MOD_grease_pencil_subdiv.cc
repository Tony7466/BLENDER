/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_array.hh"
#include "BLI_index_mask.hh"
#include "BLI_math_matrix.h"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_material_types.h"
#include "DNA_modifier_types.h"
#include "DNA_screen_types.h"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_lib_query.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "GEO_subdivide_curves.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "ED_grease_pencil.hh"

#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_prototypes.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

using namespace blender;

static void init_data(ModifierData *md)
{
  GreasePencilSubdivModifierData *gpmd = (GreasePencilSubdivModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilSubdivModifierData), modifier);
}

static void copy_data(const ModifierData *md, ModifierData *target, int flag)
{
  BKE_modifier_copydata_generic(md, target, flag);
}

static void deform_stroke(ModifierData *md,
                          Depsgraph * /*depsgraph*/,
                          Object *ob,
                          GreasePencil *gpd,
                          bke::greasepencil::Drawing *drawing)
{
  GreasePencilSubdivModifierData *mmd = (GreasePencilSubdivModifierData *)md;

  // if (!is_stroke_affected_by_modifier(ob,
  //                                     mmd->layername,
  //                                     mmd->material,
  //                                     mmd->pass_index,
  //                                     mmd->layer_pass,
  //                                     2,
  //                                     gpl,
  //                                     gps,
  //                                     mmd->flag & GP_SUBDIV_INVERT_LAYER,
  //                                     mmd->flag & GP_SUBDIV_INVERT_PASS,
  //                                     mmd->flag & GP_SUBDIV_INVERT_LAYERPASS,
  //                                     mmd->flag & GP_SUBDIV_INVERT_MATERIAL))
  //{
  //   return;
  // }

  // TODO: Catmull method?

  /* For strokes with less than 3 points, only the Simple Subdivision makes sense. */
  // short type = gps->totpoints < 3 ? short(GP_SUBDIV_SIMPLE) : mmd->type;

  if(mmd->level<1){
    return;
  }

  index_mask::IndexMaskMemory memory;
  index_mask::IndexMask selection = ed::greasepencil::retrieve_editable_strokes(
      *ob, *drawing, memory);
  VArray<int> cuts = VArray<int>::ForSingle(mmd->level,drawing->strokes().points_num());

  drawing->strokes_for_write() = geometry::subdivide_curves(
      drawing->strokes(), selection, cuts, {});
  drawing->tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  GreasePencil *gp = geometry_set->get_grease_pencil_for_write();
  if (!gp) {
    return;
  }

  Array<ed::greasepencil::MutableDrawingInfo> drawings=ed::greasepencil::retrieve_editable_drawings(*DEG_get_evaluated_scene(ctx->depsgraph),*gp);

  threading::parallel_for_each(drawings,[&](const ed::greasepencil::MutableDrawingInfo &drawing){
    deform_stroke(md,ctx->depsgraph,ctx->object,gp,&drawing.drawing);
  });
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GreasePencilSubdivModifierData *mmd = (GreasePencilSubdivModifierData *)md;

  walk(user_data, ob, (ID **)&mmd->material, IDWALK_CB_USER);
}


static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);

  uiItemL(layout,"Subdiv type INOP",0);
  //uiItemR(layout, ptr, "subdivision_type", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "level", UI_ITEM_NONE, IFACE_("Subdivisions"), ICON_NONE);

  modifier_panel_end(layout, ptr);
}

static void mask_panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;
  uiItemL(layout,"Filter INOP",0);
  //gpencil_modifier_masking_panel_draw(panel, true, false);
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_GreasePencilSubdiv, panel_draw);
  modifier_subpanel_register(
      region_type, "mask", "Influence", nullptr, mask_panel_draw, panel_type);}

ModifierTypeInfo modifierType_GreasePencilSubdiv = {
    /*idname*/ "Grese Pencil Subdiv Modifier",
    /*name*/ N_("Subdiv Modifier"),
    /*struct_name*/ "GreasePencilSubdivModifierData",
    /*struct_size*/ sizeof(GreasePencilSubdivModifierData),
    /*srna*/ &RNA_GreasePencilSubdivModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil,
    /*icon*/ ICON_MOD_SUBSURF,

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
    /*foreach_ID_link*/ foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ nullptr,
    /*blend_read*/ nullptr,
};
