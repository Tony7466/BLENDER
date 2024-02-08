/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

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

#include "GEO_realize_instances.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *emd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(emd, modifier));

  MEMCPY_STRUCT_AFTER(emd, DNA_struct_default_get(GreasePencilEnvelopeModifierData), modifier);
  modifier::greasepencil::init_influence_data(&emd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *emd = reinterpret_cast<const GreasePencilEnvelopeModifierData *>(md);
  auto *temd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(target);

  modifier::greasepencil::free_influence_data(&temd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&emd->influence, &temd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *emd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(md);
  modifier::greasepencil::free_influence_data(&emd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *emd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&emd->influence, ob, walk, user_data);
}

static void modify_drawing(const GreasePencilEnvelopeModifierData &emd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *emd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, emd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_drawing(*emd, *ctx, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  const GreasePencilEnvelopeModifierMode mode = GreasePencilEnvelopeModifierMode(
      RNA_enum_get(ptr, "mode"));

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, nullptr, ICON_NONE);

  uiItemR(layout, ptr, "spread", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "thickness", UI_ITEM_NONE, nullptr, ICON_NONE);

  switch (mode) {
    case MOD_GREASE_PENCIL_ENVELOPE_DEFORM:
      break;
    case MOD_GREASE_PENCIL_ENVELOPE_FILLS:
    case MOD_GREASE_PENCIL_ENVELOPE_SEGMENTS:
      uiItemR(layout, ptr, "strength", UI_ITEM_NONE, nullptr, ICON_NONE);
      uiItemR(layout, ptr, "mat_nr", UI_ITEM_NONE, nullptr, ICON_NONE);
      uiItemR(layout, ptr, "skip", UI_ITEM_NONE, nullptr, ICON_NONE);
      break;
  }

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_vertex_group_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilEnvelope, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *emd = reinterpret_cast<const GreasePencilEnvelopeModifierData *>(md);

  BLO_write_struct(writer, GreasePencilEnvelopeModifierData, emd);
  modifier::greasepencil::write_influence_data(writer, &emd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *emd = reinterpret_cast<GreasePencilEnvelopeModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &emd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilEnvelope = {
    /*idname*/ "GreasePencilEnvelope",
    /*name*/ N_("Envelope"),
    /*struct_name*/ "GreasePencilEnvelopeModifierData",
    /*struct_size*/ sizeof(GreasePencilEnvelopeModifierData),
    /*srna*/ &RNA_GreasePencilEnvelopeModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_ENVELOPE,

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
