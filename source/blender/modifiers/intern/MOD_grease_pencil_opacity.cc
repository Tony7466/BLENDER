/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"

#include "BKE_modifier.hh"

#include "BLO_read_write.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.h"

#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(omd, modifier));

  MEMCPY_STRUCT_AFTER(omd, DNA_struct_default_get(GreasePencilOpacityModifierData), modifier);

  // TODO
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const GreasePencilOpacityModifierData *omd = (const GreasePencilOpacityModifierData *)md;
  GreasePencilOpacityModifierData *tomd = (GreasePencilOpacityModifierData *)target;

  BKE_modifier_copydata_generic(md, target, flag);

  // TODO
  UNUSED_VARS(omd, tomd);
}

static void required_data_mask(ModifierData *md, CustomData_MeshMasks *r_cddata_masks)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  // TODO
  UNUSED_VARS(omd, r_cddata_masks);
}

static void free_data(ModifierData *md)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  // TODO
  UNUSED_VARS(omd);
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  // TODO
  UNUSED_VARS(omd, ctx, geometry_set);
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  uiLayoutSetPropSep(layout, true);

  // TODO

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilOpacity, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const GreasePencilOpacityModifierData *omd = (const GreasePencilOpacityModifierData *)md;

  BLO_write_struct(writer, GreasePencilOpacityModifierData, omd);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;
  UNUSED_VARS(reader, omd);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilOpacity = {
    /*idname*/ "GreasePencilOpacity",
    /*name*/ N_("GreasePencilOpacity"),
    /*struct_name*/ "GreasePencilOpacityModifierData",
    /*struct_size*/ sizeof(GreasePencilOpacityModifierData),
    /*srna*/ &RNA_GreasePencilOpacityModifier,
    /*type*/ ModifierTypeType::NonGeometrical,
    /*flags*/
    static_cast<ModifierTypeFlag>(
        eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping),
    /*icon*/ ICON_MOD_OPACITY,

    /*copy_data*/ blender::copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ blender::modify_geometry_set,

    /*init_data*/ blender::init_data,
    /*required_data_mask*/ blender::required_data_mask,
    /*free_data*/ blender::free_data,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ nullptr,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};
