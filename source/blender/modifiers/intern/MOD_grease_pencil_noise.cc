/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_utildefines.h"
#include "BLI_task.h"
#include "BLI_math_matrix.h"
#include "BLI_math_vector.hh"
#include "BLI_math_matrix.hh"
#include "BLI_string_ref.hh"
#include "BLI_hash.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_gpencil_legacy_types.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_object_types.h"
#include "DNA_screen_types.h"

#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_customdata.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_geometry_set.hh"
#include "BKE_gpencil_geom_legacy.h"
#include "BKE_gpencil_legacy.h"
#include "BKE_gpencil_modifier_legacy.h"
#include "BKE_lib_query.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "ED_grease_pencil.hh"

#include "MOD_gpencil_legacy_modifiertypes.h"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "MEM_guardedalloc.h"

#include "RNA_prototypes.h"
#include "RNA_access.hh"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

using namespace blender;

static void init_data(ModifierData *md)
{
  GreasePencilNoiseModifierData *gpmd = (GreasePencilNoiseModifierData *)md;

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilNoiseModifierData), modifier);

  gpmd->curve_intensity = BKE_curvemapping_add(1, 0.0f, 0.0f, 1.0f, 1.0f);
  CurveMapping *curve = gpmd->curve_intensity;
  BKE_curvemap_reset(curve->cm, &curve->clipr, CURVE_PRESET_BELL, CURVEMAP_SLOPE_POSITIVE);
  BKE_curvemapping_init(curve);
}

static void free_data(ModifierData *md)
{
  GreasePencilNoiseModifierData *gpmd = (GreasePencilNoiseModifierData *)md;

  if (gpmd->curve_intensity) {
    BKE_curvemapping_free(gpmd->curve_intensity);
  }
}

static void copy_data(const ModifierData *md, ModifierData *target, int flag)
{
  GreasePencilNoiseModifierData *gmd = (GreasePencilNoiseModifierData *)md;
  GreasePencilNoiseModifierData *tgmd = (GreasePencilNoiseModifierData *)target;

  if (tgmd->curve_intensity != nullptr) {
    BKE_curvemapping_free(tgmd->curve_intensity);
    tgmd->curve_intensity = nullptr;
  }

  BKE_modifier_copydata_generic(md, target, flag);

  tgmd->curve_intensity = BKE_curvemapping_copy(gmd->curve_intensity);
}


static bool depends_on_time(Scene * /*scene*/, ModifierData *md)
{
  GreasePencilNoiseModifierData *mmd = (GreasePencilNoiseModifierData *)md;
  return (mmd->flag & GP_NOISE_USE_RANDOM) != 0;
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

/**
 * Apply noise effect based on stroke direction.
 */
static void deform_stroke(ModifierData *md,
                          Depsgraph *depsgraph,
                          Object *ob,
                          GreasePencil * gpd,
                          bke::greasepencil::Drawing *drawing)
{
  GreasePencilNoiseModifierData *mmd = (GreasePencilNoiseModifierData *)md;
  MDeformVert *dvert = nullptr;
  /* Noise value in range [-1..1] */
  float3 vec1, vec2;
  //const int def_nr = BKE_object_defgroup_name_index(ob, mmd->vgname);
  const bool invert_group = (mmd->flag & GP_NOISE_INVERT_VGROUP) != 0;
  const bool use_curve = (mmd->flag & GP_NOISE_CUSTOM_CURVE) != 0 && mmd->curve_intensity;
  const int cfra = int(DEG_get_ctime(depsgraph));
  const bool is_keyframe = (mmd->noise_mode == GP_NOISE_RANDOM_KEYFRAME);

  // TODO: filtering.

  //if (!is_stroke_affected_by_modifier(ob,
  //                                    mmd->layername,
  //                                    mmd->material,
  //                                    mmd->pass_index,
  //                                    mmd->layer_pass,
  //                                    1,
  //                                    gpl,
  //                                    gps,
  //                                    mmd->flag & GP_NOISE_INVERT_LAYER,
  //                                    mmd->flag & GP_NOISE_INVERT_PASS,
  //                                    mmd->flag & GP_NOISE_INVERT_LAYERPASS,
  //                                    mmd->flag & GP_NOISE_INVERT_MATERIAL))
  //{
  //  return;
  //}

  bke::CurvesGeometry &strokes = drawing->strokes_for_write();
  for(const int stroke_i : strokes.curves_range()){

    int seed = mmd->seed;
    int stroke_seed = stroke_i;
    seed += stroke_seed;

    /* Make sure different modifiers get different seeds. */
    seed += BLI_hash_string(ob->id.name + 2);
    seed += BLI_hash_string(md->name);

    if (mmd->flag & GP_NOISE_USE_RANDOM) {
      if (!is_keyframe) {
        seed += cfra / mmd->step;
      }
      else {
        /* If change every keyframe, use the last keyframe. */
        /* TODO: not correct. */
        seed += cfra;
      }
    }

    /* Sanitize as it can create out of bound reads. */
    float noise_scale = clamp_f(mmd->noise_scale, 0.0f, 1.0f);

    /* Calculate or get stroke normals. */
    Span<float3> normals = strokes.evaluated_normals();
    MutableSpan<float3> positions = strokes.positions_for_write();
    MutableSpan<float> radiis = drawing->radii_for_write();
    MutableSpan<float> opacities = drawing->opacities_for_write();
    OffsetIndices<int> curves = strokes.points_by_curve();
    const int points_num = strokes.points_num();


    int len = ceilf(curves[stroke_i].size() * noise_scale) + 2;
    float *noise_table_position = (mmd->factor > 0.0f) ?
                                      noise_table(len, int(floor(mmd->noise_offset)), seed + 2) :
                                      nullptr;
    float *noise_table_strength = (mmd->factor_strength > 0.0f) ?
                                      noise_table(len, int(floor(mmd->noise_offset)), seed + 3) :
                                      nullptr;
    float *noise_table_thickness = (mmd->factor_thickness > 0.0f) ?
                                      noise_table(len, int(floor(mmd->noise_offset)), seed) :
                                      nullptr;
    float *noise_table_uvs = (mmd->factor_uvs > 0.0f) ?
                                noise_table(len, int(floor(mmd->noise_offset)), seed + 4) :
                                nullptr;

    for (const int point : curves[stroke_i].index_range()){

        // TODO: vertex group filtering.
        float weight = 1.0f;
        if (use_curve) {
          float value = float(point) / (points_num - 1);
          weight *= BKE_curvemapping_evaluateF(mmd->curve_intensity, 0, value);
        }

        if (mmd->factor > 0.0f) {
          /* Offset point randomly around the bi-normal vector. */
          if (points_num == 1) {
            vec1 = float3(1.0f, 0.0f, 0.0f);
          }
          else if (point != curves[stroke_i].last()) {
            /* Initial vector (p1 -> p0). */
            vec1 = positions[point] - positions[point + 1];
            /* if vec2 is zero, set to something */
            if (math::length(vec1) < 1e-8f) {
              vec1 = float3(1.0f, 0.0f, 0.0f);
            }
          }
          else {
            /* Last point reuse the penultimate normal (still stored in vec1)
            * because the previous point is already modified. */
          }

          /* Vector orthogonal to normal. */
          vec2 = math::cross(vec1, normals[point]);
          math::normalize(vec2);

          float noise = table_sample(noise_table_position,
                                    point * noise_scale + fractf(mmd->noise_offset));
          positions[point] += vec2 * (noise * 2.0f - 1.0f) * weight * mmd->factor * 0.1f;
        }

        if (mmd->factor_thickness > 0.0f) {
          float noise = table_sample(noise_table_thickness,
                                    point * noise_scale + fractf(mmd->noise_offset));
          radiis[point] *= max_ff(1.0f + (noise * 2.0f - 1.0f) * weight * mmd->factor_thickness, 0.0f);
          CLAMP_MIN(radiis[point], GPENCIL_STRENGTH_MIN);
        }

        if (mmd->factor_strength > 0.0f) {
          float noise = table_sample(noise_table_strength,
                                    point * noise_scale + fractf(mmd->noise_offset));
          opacities[point] *= max_ff(1.0f - noise * weight * mmd->factor_strength, 0.0f);
          CLAMP(radiis[point], GPENCIL_STRENGTH_MIN, 1.0f);
        }

        // TODO: UV not implemented yet in GPv3.
        //if (mmd->factor_uvs > 0.0f) {
        //  float noise = table_sample(noise_table_uvs, point * noise_scale + fractf(mmd->noise_offset));
        //  pt->uv_rot += (noise * 2.0f - 1.0f) * weight * mmd->factor_uvs * M_PI_2;
        //  CLAMP(pt->uv_rot, -M_PI_2, M_PI_2);
        //}
    }

    MEM_SAFE_FREE(noise_table_position);
    MEM_SAFE_FREE(noise_table_strength);
    MEM_SAFE_FREE(noise_table_thickness);
    MEM_SAFE_FREE(noise_table_uvs);
  }
}


static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                blender::bke::GeometrySet *geometry_set)
{
    GreasePencil *gp=geometry_set->get_grease_pencil_for_write();
    if (!gp){ return; }
    
  Array<ed::greasepencil::MutableDrawingInfo> drawings = ed::greasepencil::retrieve_editable_drawings(*DEG_get_evaluated_scene(ctx->depsgraph),*gp);

  threading::parallel_for_each(drawings,[&](const ed::greasepencil::MutableDrawingInfo &drawing){
    deform_stroke(md,ctx->depsgraph,ctx->object,gp,&drawing.drawing);
  });
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  GreasePencilNoiseModifierData *mmd = (GreasePencilNoiseModifierData *)md;

  walk(user_data, ob, (ID **)&mmd->material, IDWALK_CB_USER);
}

static void panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *col;
  uiLayout *layout = panel->layout;

  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, nullptr);

  uiLayoutSetPropSep(layout, true);

  col = uiLayoutColumn(layout, false);
  uiItemR(col, ptr, "factor", UI_ITEM_NONE, IFACE_("Position"), ICON_NONE);
  uiItemR(col, ptr, "factor_strength", UI_ITEM_NONE, IFACE_("Strength"), ICON_NONE);
  uiItemR(col, ptr, "factor_thickness", UI_ITEM_NONE, IFACE_("Thickness"), ICON_NONE);
  uiItemR(col, ptr, "factor_uvs", UI_ITEM_NONE, IFACE_("UV"), ICON_NONE);
  uiItemR(col, ptr, "noise_scale", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "noise_offset", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "seed", UI_ITEM_NONE, nullptr, ICON_NONE);

  modifier_panel_end(layout, ptr);
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

  uiItemR(layout, ptr, "random_mode", UI_ITEM_NONE, nullptr, ICON_NONE);

  const int mode = RNA_enum_get(ptr, "random_mode");
  if (mode != GP_NOISE_RANDOM_KEYFRAME) {
    uiItemR(layout, ptr, "step", UI_ITEM_NONE, nullptr, ICON_NONE);
  }
}

static void mask_panel_draw(const bContext * /*C*/, Panel *panel)
{
  uiLayout *layout = panel->layout;
  uiItemL(layout,"Filtering INOP",0);
  
  //modifier_masking_panel_draw(panel, true, true);
}

static void panel_register(ARegionType *region_type)
{
  PanelType *panel_type = modifier_panel_register(
      region_type, eModifierType_GreasePencilNoise, panel_draw);
  modifier_subpanel_register(
      region_type, "randomize", "", random_header_draw, random_panel_draw, panel_type);
  PanelType *mask_panel_type = modifier_subpanel_register(
      region_type, "mask", "Influence", nullptr, mask_panel_draw, panel_type);
  modifier_subpanel_register(region_type,
                                     "curve",
                                     "",
                                     modifier_grease_pencil_curve_header_draw,
                                     modifier_grease_pencil_curve_panel_draw,
                                     mask_panel_type);
}

ModifierTypeInfo modifierType_GreasePencilNoise = {
    /*idname*/ "Grease Pencil Noise Modifier",
    /*name*/ N_("Noise Modifier"),
    /*struct_name*/ "GreasePencilNoiseModifierData",
    /*struct_size*/ sizeof(GreasePencilNoiseModifierData),
    /*srna*/ &RNA_GreasePencilNoiseModifier,
    /*type*/ ModifierTypeType::OnlyDeform,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil,
    /*icon*/ ICON_GREASEPENCIL,

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
    /*depends_on_time*/ depends_on_time,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ panel_register,
    /*blend_write*/ nullptr,
    /*blend_read*/ nullptr,
};
