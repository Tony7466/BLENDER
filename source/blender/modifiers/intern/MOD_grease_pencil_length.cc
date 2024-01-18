/* SPDX-FileCopyrightText: 2005 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_hash.h"
#include "BLI_math_vector_types.hh"
#include "BLI_string_ref.hh"
#include "BLI_task.h"
#include "BLI_utildefines.h"

#include "BLT_translation.h"

#include "DNA_defaults.h"
#include "DNA_gpencil_modifier_types.h"
#include "DNA_material_types.h"
#include "DNA_node_types.h" /* For `GeometryNodeCurveSampleMode` */
#include "DNA_object_types.h"
#include "DNA_scene_types.h"
#include "DNA_screen_types.h"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_customdata.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_lib_query.hh"
#include "BKE_main.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "ED_grease_pencil.hh"

#include "MOD_grease_pencil_util.hh"
#include "MOD_modifiertypes.hh"
#include "MOD_ui_common.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "DEG_depsgraph.hh"
#include "DEG_depsgraph_query.hh"

#include "GEO_stretch_curves.hh"
#include "GEO_trim_curves.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  GreasePencilLengthModifierData *gpmd = reinterpret_cast<GreasePencilLengthModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(gpmd, modifier));

  MEMCPY_STRUCT_AFTER(gpmd, DNA_struct_default_get(GreasePencilLengthModifierData), modifier);
  modifier::greasepencil::init_influence_data(&gpmd->influence, true);
}

static void copy_data(const ModifierData *md, ModifierData *target, int flags)
{
  const auto *omd = reinterpret_cast<const GreasePencilLengthModifierData *>(md);
  auto *tomd = reinterpret_cast<GreasePencilLengthModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tomd->influence);

  BKE_modifier_copydata_generic(md, target, flags);
  modifier::greasepencil::copy_influence_data(&omd->influence, &tomd->influence, flags);
}

static void free_data(ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilLengthModifierData *>(md);
  modifier::greasepencil::free_influence_data(&omd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *omd = reinterpret_cast<GreasePencilLengthModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&omd->influence, ob, walk, user_data);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const GreasePencilLengthModifierData *mmd =
      reinterpret_cast<const GreasePencilLengthModifierData *>(md);

  BLO_write_struct(writer, GreasePencilLengthModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  GreasePencilLengthModifierData *mmd = reinterpret_cast<GreasePencilLengthModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

static Array<float> noise_table(int len, int offset, int seed)
{
  Array<float> table(len);
  for (int i = 0; i < len; i++) {
    table[i] = BLI_hash_int_01(BLI_hash_int_2d(seed, i + offset + 1));
  }
  return table;
}

BLI_INLINE float table_sample(Array<float> &table, float x)
{
  return math::interpolate(table[int(ceilf(x))], table[int(floor(x))], fractf(x));
}

static void deform_drawing(ModifierData &md,
                           Depsgraph * /*depsgraph*/,
                           Object &ob,
                           bke::greasepencil::Drawing &drawing)
{
  GreasePencilLengthModifierData &mmd = reinterpret_cast<GreasePencilLengthModifierData &>(md);
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

  if (curves.points_num() == 0) {
    return;
  }

  IndexMaskMemory memory;
  const IndexMask selection = modifier::greasepencil::get_filtered_stroke_mask(
      &ob, curves, mmd.influence, memory);

  const int cnum = curves.curves_num();

  curves = geometry::stretch_curves(
      curves,
      selection,
      std::move(VArray<float>::ForSingle(mmd.start_fac, cnum)),
      std::move(VArray<float>::ForSingle(mmd.end_fac, cnum)),
      std::move(VArray<float>::ForSingle(mmd.overshoot_fac, cnum)),
      std::move(VArray<bool>::ForSingle(mmd.flag & GP_LENGTH_USE_CURVATURE, cnum)),
      std::move(VArray<int>::ForSingle(mmd.point_density, cnum)),
      std::move(VArray<float>::ForSingle(mmd.segment_influence, cnum)),
      std::move(VArray<float>::ForSingle(mmd.max_angle, cnum)),
      std::move(VArray<bool>::ForSingle(mmd.flag & GP_LENGTH_INVERT_CURVATURE, cnum)),
      mmd.mode & GP_LENGTH_ABSOLUTE ? GEO_NODE_CURVE_SAMPLE_LENGTH : GEO_NODE_CURVE_SAMPLE_FACTOR,
      {});

  /* Always do the stretching first since it might depend on points which could be deleted by the
   * shrink. */
  if (mmd.start_fac < 0.0f || mmd.end_fac < 0.0f) {
    /* `trim_curves()` accepts the `end` valueas if it's sampling from the beginning of the
     * curve, so we need to get the lengths of the curves and substract it from the back when the
     * modifier is in Absolute mode. For convenience, we always call `trim_curves()` in LENGTH
     * mode since the function itself will need length to be sampled anyway. */
    Array<float> starts(curves.curves_num());
    Array<float> ends(curves.curves_num());
    curves.ensure_evaluated_lengths();
    for (const int curve : curves.curves_range()) {
      float length = curves.evaluated_length_total_for_curve(curve, false);
      if (mmd.mode & GP_LENGTH_ABSOLUTE) {
        starts[curve] = -math::min(mmd.start_fac, 0);
        ends[curve] = length - (-math::min(mmd.end_fac, 0));
      }
      else {
        starts[curve] = -math::min(mmd.start_fac, 0) * length;
        ends[curve] = (1 + math::min(mmd.end_fac, 0)) * length;
      }
    }
    curves = geometry::trim_curves(curves,
                                   selection,
                                   VArray<float>::ForSpan(starts.as_span()),
                                   VArray<float>::ForSpan(ends.as_span()),
                                   GEO_NODE_CURVE_SAMPLE_LENGTH,
                                   {});
  }

  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                blender::bke::GeometrySet *geometry_set)
{
  GreasePencilLengthModifierData *mmd = reinterpret_cast<GreasePencilLengthModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }

  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  const int current_frame = scene->r.cfra;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, mmd->influence, mask_memory);
  const Vector<bke::greasepencil::Drawing *> drawings =
      modifier::greasepencil::get_drawings_for_write(grease_pencil, layer_mask, current_frame);

  threading::parallel_for_each(drawings, [&](bke::greasepencil::Drawing *drawing) {
    deform_drawing(*md, ctx->depsgraph, *ctx->object, *drawing);
  });
}

static void panel_draw(const bContext *C, Panel *panel)
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

  if (uiLayout *random_layout = uiLayoutPanel(C, layout, "Randomize", ptr, "open_random_panel")) {
    uiItemR(random_layout, ptr, "use_random", UI_ITEM_NONE, IFACE_("Randomize"), ICON_NONE);

    uiLayout *subcol = uiLayoutColumn(random_layout, false);
    uiLayoutSetPropSep(subcol, true);
    uiLayoutSetActive(subcol, RNA_boolean_get(ptr, "use_random"));

    uiItemR(layout, ptr, "step", UI_ITEM_NONE, nullptr, ICON_NONE);
  }

  if (uiLayout *offset_layout = uiLayoutPanel(C, layout, "Offsets", ptr, "open_offset_panel")) {
    uiLayoutSetPropSep(offset_layout, true);
    uiItemR(offset_layout,
            ptr,
            "random_start_factor",
            UI_ITEM_NONE,
            IFACE_("Random Offset Start"),
            ICON_NONE);
    uiItemR(offset_layout,
            ptr,
            "random_end_factor",
            UI_ITEM_NONE,
            IFACE_("Random Offset End"),
            ICON_NONE);
    uiItemR(offset_layout, ptr, "random_offset", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(offset_layout, ptr, "seed", UI_ITEM_NONE, nullptr, ICON_NONE);
  }

  if (uiLayout *curvature_layout = uiLayoutPanel(
          C, layout, "Curvature", ptr, "open_curvature_panel"))
  {
    uiItemR(curvature_layout, ptr, "use_curvature", UI_ITEM_NONE, IFACE_("Curvature"), ICON_NONE);

    uiLayout *subcol = uiLayoutColumn(curvature_layout, false);
    uiLayoutSetPropSep(subcol, true);
    uiLayoutSetActive(subcol, RNA_boolean_get(ptr, "use_curvature"));

    uiItemR(subcol, ptr, "point_density", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(subcol, ptr, "segment_influence", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(subcol, ptr, "max_angle", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(subcol, ptr, "invert_curvature", UI_ITEM_NONE, IFACE_("Invert"), ICON_NONE);
  }

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
  modifier_panel_register(region_type, eModifierType_GreasePencilLength, panel_draw);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilLength = {
    /*idname*/ "GreasePencilLengthModifier",
    /*name*/ N_("Length"),
    /*struct_name*/ "GreasePencilLengthModifierData",
    /*struct_size*/ sizeof(GreasePencilLengthModifierData),
    /*srna*/ &RNA_GreasePencilLengthModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/
    (eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_EnableInEditmode |
     eModifierTypeFlag_SupportsEditmode),
    /*icon*/ ICON_MOD_LENGTH,

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
