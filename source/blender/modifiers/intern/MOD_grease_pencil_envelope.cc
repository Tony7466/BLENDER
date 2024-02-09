/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"

#include "BLI_math_geom.h"

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

#include <iostream>

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

static inline float3 calculate_plane(const float3 &center, const float3 &prev, const float3 &next)
{
  const float3 v1 = math::normalize(prev - center);
  const float3 v2 = math::normalize(next - center);
  return math::normalize(v1 - v2);
}

static inline std::optional<float3> find_plane_intersection(const float3 &plane_point,
                                                            const float3 &plane_normal,
                                                            const float3 &from,
                                                            const float3 &to)
{
  const float lambda = line_plane_factor_v3(plane_point, plane_normal, from, to);
  if (lambda <= 0.0f || lambda >= 1.0f) {
    return std::nullopt;
  }
  return math::interpolate(from, to, lambda);
}

static float calc_radius_limit(const Span<float3> positions,
                               const bool is_cyclic,
                               const int spread,
                               const int point,
                               const float3 &direction)
{
  UNUSED_VARS(positions, is_cyclic, spread, point, direction);
  // if (is_cyclic) {
  //   BLI_assert(spread <= positions.size() / 2);
  //   for (const int line_i : IndexRange(spread)) {

  //     /* Raw indices, can be out of range. */
  //     const int from_spread_i = point - spread - 1 + line_i;
  //     const int to_spread_i = point + line_i;
  //     /* Clamp or wrap to valid indices. */
  //     const int from_i = is_cyclic ? (from_spread_i + points.size()) % points.size() :
  //                                    std::max(from_i, points.first());
  //     const int to_i = is_cyclic ? (to_spread_i + points.size()) % points.size() :
  //                                  std::min(to_spread_i, points.last());
  //     const float3 &from_pos = positions[from_i];
  //     const float3 &to_pos = positions[to_i];
  //   }
  // }
  // else {

  // }
  return FLT_MAX;
}

/**
 * Find a suitable center and radius to enclose the envelope around a point.
 */
static bool find_envelope(const Span<float3> positions,
                          const bool is_cyclic,
                          const int spread,
                          const int point,
                          float3 &r_center,
                          float &r_radius)
{
  /* Compute a plane normal for intersections. */
  const IndexRange points = positions.index_range();
  const float3 &pos = positions[point];
  const float3 &prev_pos =
      positions[points.contains(point - 1) ? point - 1 : (is_cyclic ? points.last() : point)];
  const float3 &next_pos =
      positions[points.contains(point + 1) ? point + 1 : (is_cyclic ? points.first() : point)];
  const float3 plane_normal = calculate_plane(pos, prev_pos, next_pos);
  if (math::is_zero(plane_normal)) {
    return false;
  }
  std::cout << "Plane normal " << plane_normal << std::endl;

  /* Find two intersections with maximal radii. */
  float max_distance1 = 0.0f;
  float max_distance2 = 0.0f;
  float3 intersect1 = pos;
  float3 intersect2 = pos;
  for (const int line_i : IndexRange(spread + 2)) {
    /* Raw indices, can be out of range. */
    const int from_spread_i = point - spread - 1 + line_i;
    const int to_spread_i = point + line_i;
    /* Clamp or wrap to valid indices. */
    const int from_i = is_cyclic ? (from_spread_i + points.size()) % points.size() :
                                   std::max(from_spread_i, int(points.first()));
    const int to_i = is_cyclic ? (to_spread_i + points.size()) % points.size() :
                                 std::min(to_spread_i, int(points.last()));
    std::cout << "  Envelope pair (" << from_i << ", " << to_i << ")" << std::endl;
    const float3 &from_pos = positions[from_i];
    const float3 &to_pos = positions[to_i];
    const float3 line_delta = to_pos - from_pos;

    const std::optional<float3> line_intersect = find_plane_intersection(
        pos, plane_normal, from_pos, to_pos);
    if (!line_intersect) {
      continue;
    }
    const float3 line_direction = line_intersect.value() - pos;
    const float line_distance = math::length(line_direction);

    /* Diameter of a sphere centered in the plane, touching both #pos and the intersection line. */
    const float cos_angle = math::abs(math::dot(plane_normal, line_delta)) /
                            math::length(line_delta);
    const float diameter = line_distance * 2.0f * cos_angle / (1 + cos_angle);
    std::cout << "  Diameter: " << diameter << std::endl;

    if (line_i == 0) {
      max_distance1 = diameter;
      intersect1 = line_intersect.value();
      continue;
    }
    /* Use as vector 1 or 2 based on primary direction. */
    if (math::dot(intersect1 - pos, line_direction) >= 0.0f) {
      if (diameter > max_distance1) {
        intersect1 = line_intersect.value();
        max_distance1 = diameter;
      }
    }
    else {
      if (diameter > max_distance2) {
        intersect2 = line_intersect.value();
        max_distance2 = diameter;
      }
    }
    std::cout << "Closest 1: r=" << max_distance1 << " p=" << intersect1 << std::endl;
    std::cout << "Closest 2: r=" << max_distance2 << " p=" << intersect2 << std::endl;
  }

  const float3 new_center = 0.5f * (intersect1 + intersect2);
  r_radius = 0.5f * (max_distance1 + max_distance2);
  if (r_radius < FLT_EPSILON) {
    return false;
  }

  /* Apply radius limiting to not cross existing lines. */
  {
    const float3 dir = math::normalize(new_center - pos);
    // if (!math::is_zero(dir) && (is_cyclic || (line_i > 0 && i < gps->totpoints - 1))) {
    if (!math::is_zero(dir)) {
      r_radius = std::min(r_radius, calc_radius_limit(positions, is_cyclic, spread, point, dir));
    }
  }

  r_center = math::interpolate(pos, new_center, r_radius / math::distance(intersect1, intersect2));
  std::cout << "Final radius=" << r_radius << " center=" << r_center << std::endl;

  return true;
}

static void deform_drawing_as_envelope(const GreasePencilEnvelopeModifierData &emd,
                                       bke::greasepencil::Drawing &drawing,
                                       const IndexMask &curves_mask)
{
  /* TODO is this still needed? */
  const float pixfactor = 1.0f;

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  const bke::AttributeAccessor attributes = curves.attributes();
  const MutableSpan<float3> positions = curves.positions_for_write();
  const MutableSpan<float> radii = drawing.radii_for_write();
  const OffsetIndices<int> points_by_curve = curves.points_by_curve();
  const VArray<float> vgroup_weights = modifier::greasepencil::get_influence_vertex_weights(
      curves, emd.influence);
  const VArray<bool> cyclic_flags = *attributes.lookup_or_default(
      "cyclic", bke::AttrDomain::Curve, false);

  /* Cache to avoid affecting neighboring point results when updating positions. */
  const Array<float3> old_positions(positions.as_span());

  curves_mask.foreach_index(GrainSize(512), [&](const int64_t curve_i) {
    const IndexRange points = points_by_curve[curve_i];
    const bool cyclic = cyclic_flags[curve_i];
    const int point_num = points.size();
    const int spread = cyclic ? (math::abs(((emd.spread + point_num / 2) % point_num) -
                                           point_num / 2)) :
                                std::min(emd.spread, point_num - 1);

    for (const int64_t point_i : points) {
      const float weight = vgroup_weights[point_i];

      float3 envelope_center;
      float envelope_radius;
      if (!find_envelope(old_positions, cyclic, spread, point_i, envelope_center, envelope_radius))
      {
        continue;
      }

      const float target_radius = radii[point_i] * emd.thickness + envelope_radius * pixfactor;
      radii[point_i] = math::interpolate(radii[point_i], target_radius, weight);
      positions[point_i] = math::interpolate(old_positions[point_i], envelope_center, weight);
    }
  });
  std::flush(std::cout);

  drawing.tag_positions_changed();
  curves.tag_radii_changed();
}

static void modify_drawing(const GreasePencilEnvelopeModifierData &emd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  IndexMaskMemory mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, drawing.strokes(), emd.influence, mask_memory);

  const auto mode = GreasePencilEnvelopeModifierMode(emd.mode);
  switch (mode) {
    case MOD_GREASE_PENCIL_ENVELOPE_DEFORM:
      deform_drawing_as_envelope(emd, drawing, curves_mask);
      break;
    case MOD_GREASE_PENCIL_ENVELOPE_SEGMENTS:
      break;
    case MOD_GREASE_PENCIL_ENVELOPE_FILLS:
      break;
  }
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
