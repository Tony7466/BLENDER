/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_index_range.hh"
#include "BLI_map.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_span.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"
#include "BLI_vector_set.hh"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_modifier.hh"
#include "BKE_screen.hh"

#include "BLO_read_write.hh"

#include "DEG_depsgraph_query.hh"

#include "UI_interface.hh"
#include "UI_resources.hh"

#include "BLT_translation.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "RNA_access.hh"
#include "RNA_prototypes.h"

#include "MOD_grease_pencil_util.hh"
#include "MOD_ui_common.hh"

namespace blender {

static void init_data(ModifierData *md)
{
  auto *tmd = reinterpret_cast<GreasePencilTextureModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(tmd, modifier));

  MEMCPY_STRUCT_AFTER(tmd, DNA_struct_default_get(GreasePencilTextureModifierData), modifier);
  modifier::greasepencil::init_influence_data(&tmd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *tmd = reinterpret_cast<const GreasePencilTextureModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilTextureModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&tmd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *tmd = reinterpret_cast<GreasePencilTextureModifierData *>(md);
  modifier::greasepencil::free_influence_data(&tmd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *tmd = reinterpret_cast<GreasePencilTextureModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&tmd->influence, ob, walk, user_data);
}

/* Funky GPv2 method of calculating a planar transform of a stroke. */
static float4x4 get_legacy_plane_transform(const Span<float3> positions)
{
  if (positions.size() < 2) {
    return float4x4::identity();
  }

  const float3 &pt0 = positions[0];
  const float3 &pt1 = positions[1];
  const float3 &pt3 = positions[int(positions.size() * 0.75)];
  /* Point vector at 3/4 */
  const float3 loc3 = (positions.size() == 2 ? pt3 * 0.001f : pt3) - pt0;

  /* Local X axis. */
  const float3 locx = math::normalize(pt1 - pt0);
  /* Vector orthogonal to polygon plane. */
  const float3 normal = math::normalize(math::cross(locx, loc3));

  return math::from_orthonormal_axes<float4x4>(pt0, locx, normal);
}

static float4x4 get_stroke_transform(const GreasePencilTextureModifierData &tmd,
                                     const float3 &minv = {-1.0f, -1.0f, -1.0f},
                                     const float3 &maxv = {1.0f, 1.0f, 1.0f})
{
  const float4x4 minmax_transform = math::scale(math::from_location<float4x4>(-minv),
                                                math::safe_rcp(maxv - minv));
  const float4x4 stroke_rotation = math::from_rotation<float4x4>(
      math::AxisAngle(math::AxisSigned::Z_POS, tmd.fill_rotation));
  const float3 rotation_center = {0.5f, 0.5f, 0.0f};
  const float3 stroke_inv_scale = float3(math::safe_rcp(tmd.fill_scale));
  const float4x4 curve_transform = math::from_scale<float4x4>(stroke_inv_scale) *
                                   math::from_origin_transform(stroke_rotation, rotation_center) *
                                   math::from_location<float4x4>(tmd.fill_offset) *
                                   minmax_transform;
  return curve_transform;
}

static void write_stroke_us(bke::greasepencil::Drawing &drawing,
                            const IndexMask &curves_mask,
                            const float offset,
                            const float rotation,
                            const float scale)
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const VArray<bool> cyclic = curves.cyclic();

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<float> stroke_us = attributes.lookup_or_add_for_write_span<float>(
      "u_stroke", bke::AttrDomain::Point);
  bke::SpanAttributeWriter<float> rotations = attributes.lookup_or_add_for_write_span<float>(
      "rotation", bke::AttrDomain::Point);

  curves.ensure_evaluated_lengths();

  curves_mask.foreach_index(GrainSize(512), [&](int64_t curve_i) {
    const bool is_cyclic = cyclic[curve_i];
    const Span<float> lengths = curves.evaluated_lengths_for_curve(curve_i, is_cyclic);
    const IndexRange points = points_by_curve[curve_i];
    stroke_us.span[points.first()] = offset;
    rotations.span[points.first()] += rotation;
    for (const int point_i : points.drop_front(1)) {
      stroke_us.span[point_i] = lengths[point_i - 1] * scale + offset;
      rotations.span[point_i] += rotation;
    }
  });

  stroke_us.finish();
  rotations.finish();
}

static void write_fill_uvs(bke::greasepencil::Drawing &drawing,
                           const IndexMask &curves_mask,
                           const float4x4 &stroke_transform)
{
  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const Span<float3> positions = curves.positions();

  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::SpanAttributeWriter<float2> fill_uvs = attributes.lookup_or_add_for_write_span<float2>(
      "uv_fill", bke::AttrDomain::Point);

  curves_mask.foreach_index(GrainSize(512), [&](int64_t curve_i) {
    const IndexRange points = points_by_curve[curve_i];
    const float4x4 plane_transform = get_legacy_plane_transform(positions.slice(points));
    const float4x4 transform = math::invert(plane_transform * stroke_transform);
    for (const int point_i : points) {
      fill_uvs.span[point_i] = math::transform_point(transform, positions[point_i]).xy();
    }
  });

  fill_uvs.finish();
}

static void modify_curves(const GreasePencilTextureModifierData &tmd,
                          const ModifierEvalContext &ctx,
                          bke::greasepencil::Drawing &drawing)
{
  IndexMaskMemory mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, drawing.strokes(), tmd.influence, mask_memory);

  const float4x4 stroke_transform = get_stroke_transform(tmd);

  switch (GreasePencilTextureModifierMode(tmd.mode)) {
    case MOD_GREASE_PENCIL_TEXTURE_STROKE:
      write_stroke_us(drawing, curves_mask, tmd.uv_offset, tmd.alignment_rotation, tmd.uv_scale);
      break;
    case MOD_GREASE_PENCIL_TEXTURE_FILL:
      write_fill_uvs(drawing, curves_mask, stroke_transform);
      break;
    case MOD_GREASE_PENCIL_TEXTURE_STROKE_AND_FILL:
      write_stroke_us(drawing, curves_mask, tmd.uv_offset, tmd.alignment_rotation, tmd.uv_scale);
      write_fill_uvs(drawing, curves_mask, stroke_transform);
      break;
  }
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;

  const auto &tmd = *reinterpret_cast<const GreasePencilTextureModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, tmd.influence, mask_memory);
  const int frame = grease_pencil.runtime->eval_frame;
  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_curves(tmd, *ctx, *drawing); });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  const auto &tmd = *static_cast<GreasePencilTextureModifierData *>(ptr->data);
  const auto mode = GreasePencilTextureModifierMode(tmd.mode);
  const auto fit_method = GreasePencilTextureModifierFit(tmd.fit_method);
  uiLayout *col;

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "mode", UI_ITEM_NONE, nullptr, ICON_NONE);

  if (ELEM(mode, MOD_GREASE_PENCIL_TEXTURE_STROKE, MOD_GREASE_PENCIL_TEXTURE_STROKE_AND_FILL)) {
    col = uiLayoutColumn(layout, false);
    uiItemR(col, ptr, "fit_method", UI_ITEM_NONE, IFACE_("Stroke Fit Method"), ICON_NONE);
    uiItemR(col, ptr, "uv_offset", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(col, ptr, "alignment_rotation", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(col, ptr, "uv_scale", UI_ITEM_NONE, IFACE_("Scale"), ICON_NONE);
  }

  if (mode == MOD_GREASE_PENCIL_TEXTURE_STROKE_AND_FILL) {
    uiItemS(layout);
  }

  if (ELEM(mode, MOD_GREASE_PENCIL_TEXTURE_FILL, MOD_GREASE_PENCIL_TEXTURE_STROKE_AND_FILL)) {
    col = uiLayoutColumn(layout, false);
    uiItemR(col, ptr, "fill_rotation", UI_ITEM_NONE, nullptr, ICON_NONE);
    uiItemR(col, ptr, "fill_offset", UI_ITEM_NONE, IFACE_("Offset"), ICON_NONE);
    uiItemR(col, ptr, "fill_scale", UI_ITEM_NONE, IFACE_("Scale"), ICON_NONE);
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
  modifier_panel_register(region_type, eModifierType_GreasePencilTexture, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *tmd = reinterpret_cast<const GreasePencilTextureModifierData *>(md);

  BLO_write_struct(writer, GreasePencilTextureModifierData, tmd);
  modifier::greasepencil::write_influence_data(writer, &tmd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *tmd = reinterpret_cast<GreasePencilTextureModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &tmd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilTexture = {
    /*idname*/ "GreasePencilTexture",
    /*name*/ N_("TimeOffset"),
    /*struct_name*/ "GreasePencilTextureModifierData",
    /*struct_size*/ sizeof(GreasePencilTextureModifierData),
    /*srna*/ &RNA_GreasePencilTextureModifier,
    /*type*/ ModifierTypeType::NonGeometrical,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_UVPROJECT,

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
