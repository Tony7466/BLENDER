/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_range.hh"
#include "BLI_math_matrix.hh"
#include "BLI_math_vector.hh"
#include "BLI_span.hh"
#include "BLI_string.h"
#include "BLI_string_utf8.h"

#include "DNA_defaults.h"
#include "DNA_modifier_types.h"
#include "DNA_scene_types.h"

#include "BKE_curves.hh"
#include "BKE_geometry_set.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_instances.hh"
#include "BKE_lib_query.hh"
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
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  BLI_assert(MEMCMP_STRUCT_AFTER_IS_ZERO(omd, modifier));

  MEMCPY_STRUCT_AFTER(omd, DNA_struct_default_get(GreasePencilOutlineModifierData), modifier);
  modifier::greasepencil::init_influence_data(&omd->influence, false);
}

static void copy_data(const ModifierData *md, ModifierData *target, const int flag)
{
  const auto *omd = reinterpret_cast<const GreasePencilOutlineModifierData *>(md);
  auto *tmmd = reinterpret_cast<GreasePencilOutlineModifierData *>(target);

  modifier::greasepencil::free_influence_data(&tmmd->influence);

  BKE_modifier_copydata_generic(md, target, flag);
  modifier::greasepencil::copy_influence_data(&omd->influence, &tmmd->influence, flag);
}

static void free_data(ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  modifier::greasepencil::free_influence_data(&omd->influence);
}

static void foreach_ID_link(ModifierData *md, Object *ob, IDWalkFunc walk, void *user_data)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  modifier::greasepencil::foreach_influence_ID_link(&omd->influence, ob, walk, user_data);
  walk(user_data, ob, (ID **)&omd->outline_material, IDWALK_CB_USER);
  walk(user_data, ob, (ID **)&omd->object, IDWALK_CB_NOP);
}

static void update_depsgraph(ModifierData *md, const ModifierUpdateDepsgraphContext *ctx)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);
  if (ctx->scene->camera) {
    DEG_add_object_relation(
        ctx->node, ctx->scene->camera, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
    DEG_add_object_relation(
        ctx->node, ctx->scene->camera, DEG_OB_COMP_PARAMETERS, "Grease Pencil Outline Modifier");
  }
  if (omd->object != nullptr) {
    DEG_add_object_relation(
        ctx->node, omd->object, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
  }
  DEG_add_object_relation(
      ctx->node, ctx->object, DEG_OB_COMP_TRANSFORM, "Grease Pencil Outline Modifier");
}

/* Generate points in an arc between two directions. */
static void generate_arc_from_point_to_point(const float3 &from,
                                             const float3 &to,
                                             const float3 &center_pt,
                                             const int subdivisions,
                                             const int src_point_index,
                                             Vector<float3> &r_perimeter,
                                             Vector<int> &r_src_indices)
{
  const float3 vec_from = from - center_pt;
  const float3 vec_to = to - center_pt;
  if (math::is_zero(vec_from) || math::is_zero(vec_to)) {
    return;
  }

  const float dot = math::dot(vec_from.xy(), vec_to.xy());
  const float det = vec_from.x * vec_to.y - vec_from.y * vec_to.x;
  const float angle = math::atan2(det, dot);

  /* Number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
   * so we multiply by (angle / pi) to get the right amount of
   * points to insert. */
  const int num_points = std::max(int(((1 << (subdivisions + 1)) + 1) * (math::abs(angle) / M_PI)),
                                  2);
  const float delta_angle = angle / float(num_points - 1);
  const float delta_cos = math::cos(delta_angle);
  const float delta_sin = math::sin(delta_angle);

  float3 vec = vec_from;
  for ([[maybe_unused]] const int i : IndexRange(num_points)) {
    r_perimeter.append(center_pt + vec);
    r_src_indices.append(src_point_index);

    const float x = delta_cos * vec.x - delta_sin * vec.y;
    const float y = delta_sin * vec.x + delta_cos * vec.y;
    vec = float3(x, y, 0.0f);
  }
}

/* Generate a semi-circle around a point, opposite the direction. */
static void generate_cap(const float3 &point,
                         const float3 &tangent,
                         const float radius,
                         const int subdivisions,
                         const eGPDstroke_Caps cap_type,
                         const int src_point_index,
                         Vector<float3> &r_perimeter,
                         Vector<int> &r_src_indices)
{
  const float3 normal = {tangent.y, -tangent.x, 0.0f};
  switch (cap_type) {
    case GP_STROKE_CAP_ROUND:
      generate_arc_from_point_to_point(point - normal * radius,
                                       point + normal * radius,
                                       point,
                                       subdivisions,
                                       src_point_index,
                                       r_perimeter,
                                       r_src_indices);
      break;
    case GP_STROKE_CAP_FLAT:
      r_perimeter.append(point + normal * radius);
      break;
    case GP_STROKE_CAP_MAX:
      BLI_assert_unreachable();
      break;
  }
}

/* Generate a corner between two segments, with a rounded outer perimeter.
 * Note: The perimeter is considered to be to the right hand side of the stroke. The left side
 * perimeter can be generated by reversing the order of points. */
static void generate_corner(const float3 &pt_a,
                            const float3 &pt_b,
                            const float3 &pt_c,
                            const float radius,
                            const int subdivisions,
                            const int src_point_index,
                            Vector<float3> &r_perimeter,
                            Vector<int> &r_src_indices)
{
  const float3 tangent = math::normalize(pt_c - pt_b);
  const float3 tangent_prev = math::normalize(pt_b - pt_a);
  const float cos_angle = math::dot(tangent, tangent_prev);
  const float sin_angle = tangent_prev.x * tangent.y - tangent_prev.y * tangent.x;
  /* Whether the corner is an inside or outside corner.
   * This determines whether an arc is added or a single miter point. */
  const bool is_outside_corner = (sin_angle >= 0.0f);
  if (is_outside_corner) {
    const float3 normal = {tangent.y, -tangent.x, 0.0f};
    const float3 normal_prev = {tangent_prev.y, -tangent_prev.x, 0.0f};
    generate_arc_from_point_to_point(pt_b + normal_prev * radius,
                                     pt_b + normal * radius,
                                     pt_b,
                                     subdivisions,
                                     src_point_index,
                                     r_perimeter,
                                     r_src_indices);
  }
  else {
    // const float3 miter = math::normalize(tangent_prev + tangent);
    const float3 normal = {tangent.y, -tangent.x, 0.0f};
    const float3 normal_prev = {tangent_prev.y, -tangent_prev.x, 0.0f};
    r_perimeter.append(pt_b + normal * radius);
    r_src_indices.append(src_point_index);
  }
}

static void generate_stroke_perimeter(const Span<float3> all_positions,
                                      const VArray<float> all_radii,
                                      const IndexRange points,
                                      const int subdivisions,
                                      const bool is_cyclic,
                                      const eGPDstroke_Caps start_cap_type,
                                      const eGPDstroke_Caps end_cap_type,
                                      const float normal_offset,
                                      Vector<float3> &r_perimeter,
                                      Vector<int> &r_point_counts,
                                      Vector<int> &r_point_indices)
{
  const Span<float3> positions = all_positions.slice(points);
  const int point_num = points.size();
  if (point_num < 2) {
    return;
  }

  auto add_corner = [&](const int a, const int b, const int c) {
    const int point = points[b];
    const float3 pt_a = positions[a];
    const float3 pt_b = positions[b];
    const float3 pt_c = positions[c];
    const float radius = all_radii[point];
    generate_corner(pt_a, pt_b, pt_c, radius, subdivisions, point, r_perimeter, r_point_indices);
  };

  if (is_cyclic) {
    /* Cyclic curves have an "inside" and an "outside" perimeter.
     * The distinction is arbitrary, called left/right here. */

    /* Left side perimeter. */
    const int left_perimeter_start = r_perimeter.size();
    add_corner(point_num - 1, 0, 1);
    for (const int i : points.index_range().drop_front(1).drop_back(1)) {
      add_corner(i - 1, i, i + 1);
    }
    add_corner(point_num - 2, point_num - 1, 0);
    const int left_perimeter_count = r_perimeter.size() - left_perimeter_start;
    if (left_perimeter_count > 0) {
      r_point_counts.append(left_perimeter_count);
    }

    /* Right side perimeter. */
    const int right_perimeter_start = r_perimeter.size();
    add_corner(0, point_num - 1, point_num - 2);
    for (const int i : points.index_range().drop_front(1).drop_back(1)) {
      add_corner(point_num - i, point_num - i - 1, point_num - i - 2);
    }
    add_corner(1, 0, point_num - 1);
    const int right_perimeter_count = r_perimeter.size() - right_perimeter_start;
    if (right_perimeter_count > 0) {
      r_point_counts.append(right_perimeter_count);
    }
  }
  else {
    /* Open curves generate a start and end cap and a connecting stroke on either side. */
    const int perimeter_start = r_perimeter.size();

    /* Start cap. */
    {
      const float3 &center = positions.first();
      const float3 dir = math::normalize(positions[1] - center);
      const float radius = all_radii[points.first()];
      generate_cap(center,
                   dir,
                   radius,
                   subdivisions,
                   start_cap_type,
                   points.first(),
                   r_perimeter,
                   r_point_indices);
    }

    /* Left perimeter half. */
    for (const int i : points.index_range().drop_front(1).drop_back(1)) {
      add_corner(i - 1, i, i + 1);
    }

    /* End cap. */
    {
      const float3 &center = positions.last();
      const float3 dir = math::normalize(positions[point_num - 2] - center);
      const float radius = all_radii[points.last()];
      generate_cap(center,
                   dir,
                   radius,
                   subdivisions,
                   end_cap_type,
                   points.last(),
                   r_perimeter,
                   r_point_indices);
    }

    /* Right perimeter half. */
    for (const int i : points.index_range().drop_front(1).drop_back(1)) {
      add_corner(point_num - i, point_num - i - 1, point_num - i - 2);
    }

    /* Open curves have a single perimeter curve. */
    const int perimeter_count = r_perimeter.size() - perimeter_start;
    if (perimeter_count > 0) {
      r_point_counts.append(perimeter_count);
    }
  }

  UNUSED_VARS(normal_offset);
}

struct PerimeterData {
  /* New points per curve count. */
  Vector<int> point_counts;
  /* New point coordinates. */
  Vector<float3> positions;
  /* Source curve index. */
  Vector<int> curve_indices;
  /* Source point index. */
  Vector<int> point_indices;
};

static bke::CurvesGeometry create_curves_outline(const bke::greasepencil::Drawing &drawing,
                                                 const float4x4 &viewmat,
                                                 const IndexMask &curves_mask,
                                                 const int subdivisions,
                                                 const float stroke_radius)
{
  const bke::CurvesGeometry &src_curves = drawing.strokes();
  Span<float3> src_positions = src_curves.positions();
  bke::AttributeAccessor src_attributes = src_curves.attributes();
  VArray<float> src_radii = drawing.radii();
  const VArray<bool> src_cyclic = *src_attributes.lookup_or_default(
      "cyclic", bke::AttrDomain::Curve, false);
  VArray<int8_t> src_start_caps = *src_attributes.lookup_or_default<int8_t>(
      "start_cap", bke::AttrDomain::Curve, GP_STROKE_CAP_ROUND);
  VArray<int8_t> src_end_caps = *src_attributes.lookup_or_default<int8_t>(
      "end_cap", bke::AttrDomain::Curve, GP_STROKE_CAP_ROUND);

  /* Transform positions into view space. */
  Array<float3> view_positions(src_positions.size());
  threading::parallel_for(view_positions.index_range(), 4096, [&](const IndexRange range) {
    for (const int i : range) {
      view_positions[i] = math::transform_point(viewmat, src_positions[i]);
    }
  });

  const float4x4 viewinv = math::invert(viewmat);
  threading::EnumerableThreadSpecific<PerimeterData> thread_data;
  curves_mask.foreach_index([&](const int64_t curve_i) {
    PerimeterData &data = thread_data.local();

    const int prev_point_num = data.positions.size();
    const int prev_curve_num = data.point_counts.size();
    const IndexRange points = src_curves.points_by_curve()[curve_i];
    const float normal_offset = 0.0f;
    generate_stroke_perimeter(view_positions,
                              src_radii,
                              points,
                              subdivisions,
                              src_cyclic[curve_i],
                              eGPDstroke_Caps(src_start_caps[curve_i]),
                              eGPDstroke_Caps(src_end_caps[curve_i]),
                              normal_offset,
                              data.positions,
                              data.point_counts,
                              data.point_indices);

    /* Transform perimeter positions back into object space. */
    for (float3 &pos : data.positions.as_mutable_span().drop_front(prev_point_num)) {
      pos = math::transform_point(viewinv, pos);
    }

    data.curve_indices.append_n_times(curve_i, data.point_counts.size() - prev_curve_num);
  });

  int dst_curve_num = 0;
  int dst_point_num = 0;
  for (const PerimeterData &data : thread_data) {
    BLI_assert(data.point_counts.size() == data.curve_indices.size());
    BLI_assert(data.positions.size() == data.point_indices.size());
    dst_curve_num += data.point_counts.size();
    dst_point_num += data.positions.size();
  }

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  bke::SpanAttributeWriter<bool> dst_cyclic = dst_attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<float> dst_radius = dst_attributes.lookup_or_add_for_write_span<float>(
      "radius", bke::AttrDomain::Point);
  const MutableSpan<int> dst_offsets = dst_curves.offsets_for_write();
  const MutableSpan<float3> dst_positions = dst_curves.positions_for_write();
  /* Source indices for attribute mapping. */
  Array<int> dst_curve_map(dst_curve_num);
  Array<int> dst_point_map(dst_point_num);

  IndexRange curves;
  IndexRange points;
  for (const PerimeterData &data : thread_data) {
    curves = curves.after(data.point_counts.size());
    points = points.after(data.positions.size());

    /* Append curve data. */
    dst_curve_map.as_mutable_span().slice(curves).copy_from(data.curve_indices);
    /* Curve offsets are accumulated below. */
    dst_offsets.slice(curves).copy_from(data.point_counts);
    dst_cyclic.span.slice(curves).fill(true);

    /* Append point data. */
    dst_positions.slice(points).copy_from(data.positions);
    dst_point_map.as_mutable_span().slice(points).copy_from(data.point_indices);
    dst_radius.span.slice(points).fill(stroke_radius);
  }
  offset_indices::accumulate_counts_to_offsets(dst_curves.offsets_for_write());

  bke::gather_attributes(src_attributes,
                         bke::AttrDomain::Point,
                         {},
                         {"position", "radius"},
                         dst_point_map,
                         dst_attributes);
  bke::gather_attributes(
      src_attributes, bke::AttrDomain::Curve, {}, {"cyclic"}, dst_curve_map, dst_attributes);

  dst_cyclic.finish();
  dst_radius.finish();
  dst_curves.update_curve_types();

  return dst_curves;
}

static void modify_drawing(const GreasePencilOutlineModifierData &omd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing,
                           const float4x4 &viewmat)
{
  if (drawing.strokes().curve_num == 0) {
    return;
  }

  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, drawing.strokes(), omd.influence, curve_mask_memory);

  /* Legacy thickness setting is diameter in pixels, divide by 2000 to get radius. */
  const float radius = omd.thickness * 0.0005f;
  drawing.strokes_for_write() = create_curves_outline(
      drawing, viewmat, curves_mask, omd.subdiv, radius);
  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;
  using bke::greasepencil::Layer;
  using modifier::greasepencil::LayerDrawingInfo;

  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  const Scene *scene = DEG_get_evaluated_scene(ctx->depsgraph);
  if (!scene->camera) {
    return;
  }
  const float4x4 viewinv = scene->camera->world_to_object();

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, omd->influence, mask_memory);

  const Vector<LayerDrawingInfo> drawings = modifier::greasepencil::get_drawing_infos_by_layer(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings, [&](const LayerDrawingInfo &info) {
    const Layer &layer = *grease_pencil.layers()[info.layer_index];
    const float4x4 viewmat = viewinv * layer.to_world_space(*ctx->object);
    modify_drawing(*omd, *ctx, *info.drawing, viewmat);
  });
}

static void panel_draw(const bContext *C, Panel *panel)
{
  uiLayout *layout = panel->layout;

  PointerRNA ob_ptr;
  PointerRNA *ptr = modifier_panel_get_property_pointers(panel, &ob_ptr);
  // auto *omd = static_cast<GreasePencilOutlineModifierData *>(ptr->data);

  uiLayoutSetPropSep(layout, true);

  uiItemR(layout, ptr, "thickness", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "use_keep_shape", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "subdivision", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "sample_length", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "outline_material", UI_ITEM_NONE, nullptr, ICON_NONE);
  uiItemR(layout, ptr, "object", UI_ITEM_NONE, nullptr, ICON_NONE);

  Scene *scene = CTX_data_scene(C);
  if (scene->camera == nullptr) {
    uiItemL(layout, RPT_("Outline requires an active camera"), ICON_ERROR);
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
  modifier_panel_register(region_type, eModifierType_GreasePencilOutline, panel_draw);
}

static void blend_write(BlendWriter *writer, const ID * /*id_owner*/, const ModifierData *md)
{
  const auto *omd = reinterpret_cast<const GreasePencilOutlineModifierData *>(md);

  BLO_write_struct(writer, GreasePencilOutlineModifierData, omd);
  modifier::greasepencil::write_influence_data(writer, &omd->influence);
}

static void blend_read(BlendDataReader *reader, ModifierData *md)
{
  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  modifier::greasepencil::read_influence_data(reader, &omd->influence);
}

}  // namespace blender

ModifierTypeInfo modifierType_GreasePencilOutline = {
    /*idname*/ "GreasePencilOutline",
    /*name*/ N_("Outline"),
    /*struct_name*/ "GreasePencilOutlineModifierData",
    /*struct_size*/ sizeof(GreasePencilOutlineModifierData),
    /*srna*/ &RNA_GreasePencilOutlineModifier,
    /*type*/ ModifierTypeType::Nonconstructive,
    /*flags*/ eModifierTypeFlag_AcceptsGreasePencil | eModifierTypeFlag_SupportsEditmode |
        eModifierTypeFlag_EnableInEditmode | eModifierTypeFlag_SupportsMapping,
    /*icon*/ ICON_MOD_OUTLINE,

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
    /*update_depsgraph*/ blender::update_depsgraph,
    /*depends_on_time*/ nullptr,
    /*depends_on_normals*/ nullptr,
    /*foreach_ID_link*/ blender::foreach_ID_link,
    /*foreach_tex_link*/ nullptr,
    /*free_runtime_data*/ nullptr,
    /*panel_register*/ blender::panel_register,
    /*blend_write*/ blender::blend_write,
    /*blend_read*/ blender::blend_read,
};
