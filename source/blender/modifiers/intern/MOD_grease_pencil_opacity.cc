/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_modifier.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_enum_types.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include <iostream>

namespace blender {

using bke::greasepencil::Drawing;
using bke::greasepencil::FramesMapKey;
using bke::greasepencil::Layer;

static void init_data(ModifierData *md)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(omd, modifier));

  // XXX Why is this crashing, but the expanded code below works fine?!?
  // MEMCPY_STRUCT_AFTER(omd, DNA_struct_default_get(GreasePencilOpacityModifierData), modifier);
  {
    CHECK_TYPE_NONCONST(omd);
    memcpy((char *)(omd) + OFFSETOF_STRUCT_AFTER(omd, modifier),
           (const char *)(md) + OFFSETOF_STRUCT_AFTER(omd, modifier),
           sizeof(*(omd)) - OFFSETOF_STRUCT_AFTER(omd, modifier));
  }

  greasepencil::init_influence_data(&omd->influence);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const GreasePencilOpacityModifierData *omd = (const GreasePencilOpacityModifierData *)md;
  GreasePencilOpacityModifierData *tomd = (GreasePencilOpacityModifierData *)target;

  BKE_modifier_copydata_generic(md, target, flag);
  greasepencil::copy_influence_data(&omd->influence, &tomd->influence, flag);
}

static void required_data_mask(ModifierData *md, CustomData_MeshMasks *r_cddata_masks)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  // TODO
  UNUSED_VARS(omd, r_cddata_masks);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;
  greasepencil::foreach_influence_ID_link(&omd->influence, ob, walk, user_data);
}

static void free_data(ModifierData *md)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  greasepencil::free_influence_data(&omd->influence);
}

static void modify_curves(ModifierData *md,
                          const ModifierEvalContext *ctx,
                          bke::CurvesGeometry &curves)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<float> opacities = attributes.lookup_or_add_for_write_span<float>(
      "opacity", bke::AttrDomain::Point);

  OffsetIndices<int> points_by_curve = curves.points_by_curve();
  IndexMaskMemory mask_memory;
  IndexMask curves_mask = greasepencil::get_filtered_stroke_mask(
      ctx->object, curves, omd->influence.material_filter, mask_memory);
  for (const int64_t i : curves_mask.index_range()) {
    const int64_t curve_i = curves_mask[i];
    for (const int64_t point_i : points_by_curve[curve_i]) {
      opacities.span[point_i] *= 0.5f;
    }
  }

  opacities.finish();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  GreasePencilOpacityModifierData *omd = (GreasePencilOpacityModifierData *)md;
  const Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  const int frame = scene->r.cfra;

  GreasePencil *grease_pencil = geometry_set->get_grease_pencil_for_write();
  if (grease_pencil == nullptr) {
    return;
  }

  IndexMaskMemory mask_memory;
  IndexMask layer_mask = greasepencil::get_filtered_layer_mask(
      *grease_pencil, omd->influence.layer_filter, mask_memory);
  Vector<Drawing *> drawings = greasepencil::get_drawings_for_write(
      *grease_pencil, layer_mask, frame);
  for (Drawing *drawing : drawings) {
    modify_curves(md, ctx, drawing->strokes_for_write());
  }
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  if (uiLayout *influence_panel = uiLayoutPanel(
          C, layout, "Influence", ptr, "open_influence_panel"))
  {
    PointerRNA layer_filter_ptr = RNA_pointer_get(ptr, "layer_filter");
    PointerRNA material_filter_ptr = RNA_pointer_get(ptr, "material_filter");
    greasepencil::draw_layer_filter_settings(C, influence_panel, &layer_filter_ptr);
    greasepencil::draw_material_filter_settings(C, influence_panel, &material_filter_ptr);
  }

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
    /*type*/ ModifierTypeType::Nonconstructive,
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
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};
