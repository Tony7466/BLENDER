/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "DNA_modifier_types.h"

#include "DEG_depsgraph_build.hh"

#include "BLI_math_matrix_types.hh"

#include "BKE_action.h"
#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"

#include "BLO_read_write.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.h"

#include "RNA_prototypes.h"

#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *tmd = reinterpret_cast<GPTransformModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(tmd, modifier));
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  BKE_modifier_copydata_generic(md, target, flag);
}

static void modify_geometry_set(ModifierData * /*md*/,
                                const ModifierEvalContext * /*ctx*/,
                                bke::GeometrySet *geometry_set)
{
  using namespace bke::greasepencil;
  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();

  for (const int layer_i : grease_pencil.layers().index_range()) {
    const Layer &layer = *grease_pencil.layers()[layer_i];
    if (layer.parent == nullptr || !layer.is_visible()) {
      continue;
    }
    if (Drawing *drawing = get_eval_grease_pencil_layer_drawing_for_write(grease_pencil, layer_i))
    {
      Object &parent = *layer.parent;
      const float4x4 parent_matrix = [&]() {
        if (parent.type == OB_ARMATURE && layer.parsubstr[0] != '\0') {
          if (bPoseChannel *channel = BKE_pose_channel_find_name(parent.pose, layer.parsubstr)) {
            return float4x4_view(parent.object_to_world) * float4x4_view(channel->pose_mat);
          }
        }
        return float4x4(float4x4_view(parent.object_to_world));
      }();
      drawing->strokes_for_write().transform(parent_matrix);
    }
  }
}

static void update_depsgraph(ModifierData * /*md*/, const ModifierUpdateDepsgraphContext *ctx)
{
  using namespace bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(ctx->object->data);

  for (const Layer *layer : grease_pencil.layers()) {
    if (layer->parent == nullptr) {
      continue;
    }
    if (BKE_pose_channel_find_name(layer->parent->pose, layer->parsubstr) != nullptr) {
      DEG_add_bone_relation(ctx->node,
                            layer->parent,
                            layer->parsubstr,
                            DEG_OB_COMP_BONE,
                            "Grease Pencil Transform Modifier");
    }
    DEG_add_object_relation(
        ctx->node, layer->parent, DEG_OB_COMP_TRANSFORM, "Grease Pencil Transform Modifier");
  }
  DEG_add_depends_on_transform_relation(ctx->node, "Grease Pencil Transform Modifier");
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *tmd = reinterpret_cast<const GPTransformModifierData *>(md);
  BLO_write_struct(writer, GPTransformModifierData, tmd);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilTransform = {
    /*idname*/ "GreasePencilTransform",
    /*name*/ N_("Transform"),
    /*struct_name*/ "GPTransformModifierData",
    /*struct_size*/ sizeof(GPTransformModifierData),
    /*srna*/ &RNA_GreasePencilTransformModifier,
    /*type*/ ModifierTypeType::OnlyDeform,
    /*flags*/
    static_cast<ModifierTypeFlag>(
        eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping),
    /*icon*/ ICON_MOD_OFFSET,

    /*copy_data*/ blender::copy_data,

    /*deform_verts*/ nullptr,
    /*deform_matrices*/ nullptr,
    /*deform_verts_EM*/ nullptr,
    /*deform_matrices_EM*/ nullptr,
    /*modify_mesh*/ nullptr,
    /*modify_geometry_set*/ blender::modify_geometry_set,

    /*init_data*/ blender::init_data,
    /*required_data_mask*/ nullptr,
    /*free_data*/ nullptr,
    /*is_disabled*/ nullptr,
    /*update_depsgraph*/ blender::update_depsgraph,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ nullptr,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ nullptr,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ nullptr,
};
