/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MEM_guardedalloc.h"

#include "BLI_math_matrix.hh"

#include "DNA_defaults.h"
#include "DNA_material_types.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_lib_query.hh"
#include "BKE_material.h"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_query.hh"

#include "GEO_realize_instances.hh"

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

namespace blender {

using bke::greasepencil::Drawing;

static void init_data(ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(mmd, modifier));

  MEMCPY_STRUCT_AFTER(mmd, DNA_struct_default_get(GreasePencilMultiModifierData), modifier);
  modifier::greasepencil::init_influence_data(&mmd->influence, true);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *mmd = reinterpret_cast<const GreasePencilMultiModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilMultiModifierData *>(target);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&mmd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);
  modifier::greasepencil::free_influence_data(&mmd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&mmd->influence, ob, walk, user_data);
}

static bool is_disabled(const Scene * /*scene*/, ModifierData *md, bool /*use_render_params*/)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);
  if (mmd->duplications < 1) {
    return true;
  }
  return false;
}

static bke::CurvesGeometry duplicate_strokes(const bke::CurvesGeometry &curves, const IndexMask curves_mask, const IndexMask unselected_mask, const int count, int &original_point_count){
  bke::CurvesGeometry masked_curves = bke::curves_copy_curve_selection(
      curves, curves_mask, {});
  bke::CurvesGeometry unselected_curves = bke::curves_copy_curve_selection(
      curves, unselected_mask, {});

  original_point_count = masked_curves.points_num();

  Curves *masked_curves_id = bke::curves_new_nomain(masked_curves);
  Curves *unselected_curves_id = bke::curves_new_nomain(unselected_curves);

  bke::GeometrySet masked_geo = bke::GeometrySet::from_curves(masked_curves_id);
  bke::GeometrySet unselected_geo = bke::GeometrySet::from_curves(unselected_curves_id);

  std::unique_ptr<bke::Instances> instances = std::make_unique<bke::Instances>();
  const int masked_handle = instances->add_reference(bke::InstanceReference{masked_geo});
  const int unselected_handle = instances->add_reference(bke::InstanceReference{unselected_geo});

  for(int i=0;i<count;i++){
    instances->add_instance(masked_handle, float4x4::identity());
  }
  instances->add_instance(unselected_handle, float4x4::identity());

  geometry::RealizeInstancesOptions options;
  options.keep_original_ids = true;
  options.realize_instance_attributes = true;
  options.propagation_info = {};
  bke::GeometrySet result_geo = geometry::realize_instances(
      bke::GeometrySet::from_instances(instances.release()), options);
  return std::move(result_geo.get_curves_for_write()->geometry.wrap());
}

static void generate_curves(GreasePencilMultiModifierData &mmd, const ModifierEvalContext &ctx, Drawing &drawing)
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();

  IndexMaskMemory mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, curves, mmd.influence, mask_memory);

  const IndexMask unselected_mask = curves_mask.complement(curves.curves_range(),mask_memory);
  
  if(curves_mask.is_empty()){ return; }
  
  int original_point_count;
  bke::CurvesGeometry duplicated_strokes = duplicate_strokes(curves,curves_mask,unselected_mask,mmd.duplications + 1,original_point_count);

  const float offset = math::length(math::to_scale(float4x4(ctx.object->object_to_world))) * mmd.offset;
  const float distance = mmd.distance;

  const Span<float3> positions = duplicated_strokes.positions();
  const Span<float3> normals = duplicated_strokes.evaluated_normals();
  const Span<float3> tangents = duplicated_strokes.evaluated_tangents();

  const int points_num_pending = (mmd.duplications+1)*original_point_count;

  Array<float3> pos_l(points_num_pending);
  Array<float3> pos_r(points_num_pending);
  
  threading::parallel_for(curves.points_range().take_front(points_num_pending),1024,[&](const IndexRange parallel_range){
    for(const int point : parallel_range){
      const float3 minter = math::cross(normals[point],tangents[point]) * distance;
      pos_l = positions[point] + minter;
      pos_r = positions[point] - minter;
    }
  });

  for(const int i : IndexRange(mmd.duplications + 1)){
    Span<float3> instance_positions = positions.slice(IndexRange(original_point_count*i,original_point_count));
    Span<float3> use_pos_l = pos_l.as_span().slice(IndexRange(original_point_count*i,original_point_count));
    Span<float3> use_pos_r = pos_r.as_span().slice(IndexRange(original_point_count*i,original_point_count));
    threading::parallel_for(instance_positions.index_range(),512,[&](const IndexRange parallel_range){
      for(const int point : parallel_range){
        instance_positions[point] = math::interpolate(use_pos_l,use_pos_r,float(i) / float(mmd.duplications));
      }
    });
  }

  curves = duplicated_strokes;
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, mmd->influence, mask_memory);
  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { generate_curves(*md, *ctx, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "duplicates", UI_ITEM_NONE, nullptr, ICON_NONE);

  uiLayout *col = uiLayoutColumn(layout, false);
  uiLayoutSetActive(col, RNA_int_get(ptr, "duplicates") > 0);
  uiItemR(col, ptr, "distance", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(col, ptr, "offset", UI_ITEM_R_SLIDER, nullptr, ICON_NONE);

  if (uiLayout *fade_panel = uiLayoutPanelProp(C, layout, ptr, "open_fading_panel", "Fade")) {
    uiLayoutSetPropSep(fade_panel, true);
    uiItemR(fade_panel, ptr, "use_fade", UI_ITEM_NONE, nullptr, ICON_NONE);

    uiLayout *sub = uiLayoutColumn(fade_panel, false);
    uiLayoutSetActive(col, RNA_boolean_get(ptr, "use_fade"));

    uiItemR(sub, ptr, "fading_center", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(sub, ptr, "fading_thickness", UI_ITEM_R_SLIDER, nullptr, ICON_NONE);
    uiItemR(sub, ptr, "fading_opacity", UI_ITEM_R_SLIDER, nullptr, ICON_NONE);
  }

  if (uiLayout *influence_panel = uiLayoutPanelProp(
          C, layout, ptr, "open_influence_panel", "Influence"))
  {
    modifier::greasepencil::draw_layer_filter_settings(C, influence_panel, ptr);
    modifier::greasepencil::draw_material_filter_settings(C, influence_panel, ptr);
  }

  modifier_panel_end(layout, ptr);
}

static void panel_register(ARegionType *region_type)
{
  modifier_panel_register(region_type, eModifierType_GreasePencilMultiply, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *mmd = reinterpret_cast<const GreasePencilMultiModifierData *>(md);

  BLO_write_struct(writer, GreasePencilMultiModifierData, mmd);
  modifier::greasepencil::write_influence_data(writer, &mmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *mmd = reinterpret_cast<GreasePencilMultiModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &mmd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilMultiply = {
    /*idname*/ "GreasePencilMultiply",
    /*name*/ N_("Multiple Strokes"),
    /*struct_name*/ "GreasePencilMultiModifierData",
    /*struct_size*/ sizeof(GreasePencilMultiModifierData),
    /*srna*/ &RNA_GreasePencilMultiplyModifier,
    /*type*/ ModifierTypeType::Constructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_CURVE,

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
