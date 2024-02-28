/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "BLI_enumerable_thread_specific.hh"
#include "BLI_index_range.hh"
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

/* Generate points in an arc between two points. */
static void generate_arc_from_point_to_point(const float3 &from,
                                             const float3 &to,
                                             const float3 &center_pt,
                                             const int subdivisions,
                                             const int src_index,
                                             Vector<float3> &r_perimeter,
                                             Vector<int> &r_src_indices)
{
  const float2 vec_from = from.xy() - center_pt.xy();
  const float2 vec_to = to.xy() - center_pt.xy();
  if (math::is_zero(vec_from) || math::is_zero(vec_to)) {
    return;
  }

  const float dot = math::dot(vec_from, vec_to);
  const float det = vec_from.x * vec_to.y - vec_from.y * vec_to.x;
  const float angle = math::atan2(det, dot) + M_PI;

  /* Number of points is 2^(n+1) + 1 on half a circle (n=subdivisions)
   * so we multiply by (angle / pi) to get the right amount of
   * points to insert. */
  const int num_points = ((1 << (subdivisions + 1)) - 1) * (angle / M_PI);
  if (num_points <= 0) {
    return;
  }

  const float2x2 rotation = math::from_rotation<float2x2>(angle / float(num_points));
  float2 vec = vec_from;
  for ([[maybe_unused]] const int i : IndexRange(num_points).drop_back(1)) {
    vec = rotation * vec;
    r_perimeter.append_as(vec);
    r_src_indices.append(src_index);
  }
}

static void generate_start_cap(const float3 &point,
                               const float3 &tangent,
                               const float radius,
                               const int subdivisions,
                               const eGPDstroke_Caps cap_type,
                               const int src_index,
                               Vector<float3> &r_perimeter,
                               Vector<int> &r_src_indices)
{
  const float3 normal = {tangent.y, -tangent.x, 0.0f};
  switch (cap_type) {
    case GP_STROKE_CAP_ROUND:
      generate_arc_from_point_to_point(point + normal * radius,
                                       point - normal * radius,
                                       point,
                                       subdivisions,
                                       src_index,
                                       r_perimeter,
                                       r_src_indices);
      break;
    case GP_STROKE_CAP_FLAT:
      r_perimeter.append(point - normal * radius);
      break;
    case GP_STROKE_CAP_MAX:
      BLI_assert_unreachable();
      break;
  }
}

static void generate_end_cap(const float3 &point,
                             const float3 &tangent,
                             const float radius,
                             const int subdivisions,
                             const eGPDstroke_Caps cap_type,
                             const int src_index,
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
                                       src_index,
                                       r_perimeter,
                                       r_src_indices);
      break;
    case GP_STROKE_CAP_FLAT:
      r_perimeter.append(point - normal * radius);
      break;
    case GP_STROKE_CAP_MAX:
      BLI_assert_unreachable();
      break;
  }
}

static void generate_corner(const float3 &pt_a,
                            const float3 &pt_b,
                            const float3 &pt_c,
                            const float radius,
                            const int src_index,
                            Vector<float3> &r_perimeter,
                            Vector<int> &r_src_indices)
{
  const float2 tangent = pt_c.xy() - pt_b.xy();
  // const float2 tangent_prev = pt_b.xy() - pt_a.xy();
  const float2 normal = {tangent.y, -tangent.x};
  r_perimeter.append(float3(pt_b.xy() + normal * radius));
  r_src_indices.append(src_index);
  UNUSED_VARS(pt_a);
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
                                      Vector<int> &r_src_indices)
{
  const Span<float3> positions = all_positions.slice(points);

  if (positions.size() < 2) {
    return;
  }

  if (is_cyclic) {
    const float3 &pt_a = positions.last();
    const float3 &pt_b = positions.first();
    const float3 &pt_c = positions[1];
    const float radius = all_radii[points.first()];
    generate_corner(pt_a, pt_b, pt_c, radius, points.first(), r_perimeter, r_src_indices);
  }
  else {
    const float3 &center = positions.first();
    const float3 dir = math::normalize(positions[1] - center);
    const float radius = all_radii[points.first()];
    generate_start_cap(center,
                       dir,
                       radius,
                       subdivisions,
                       start_cap_type,
                       points.first(),
                       r_perimeter,
                       r_src_indices);
  }
  // for (const int i : positions.index_range().drop_front(1).drop_back(1)) {
  //   const float2 pt_a = positions[i - 1].xy();
  //   const float2 pt_b = positions[i].xy();
  //   const float2 pt_c = positions[i + 1].xy();
  //   const float2 tangent = pt_c - pt_b;
  //   const float2 tangent_prev = pt_b - pt_a;
  //   const float2 normal = {tangent.y, -tangent.x};
  //   r_perimeter.append(pt_b + normal * radius);
  // }

  // for (const int i : positions.index_range().drop_front(1).drop_back(1)) {
  //   const float2 pt_a = positions[i - 1].xy();
  //   const float2 pt_b = positions[i].xy();
  //   const float2 pt_c = positions[i + 1].xy();
  //   const float2 tangent = pt_c - pt_b;
  //   const float2 tangent_prev = pt_b - pt_a;
  //   const float2 normal = {tangent.y, -tangent.x};
  //   r_perimeter.append(pt_b + normal * radius);
  // }
  UNUSED_VARS(end_cap_type, normal_offset);
}

struct PerimeterData {
  /* New points per curve count. */
  Vector<int> point_counts;
  /* New point coordinates. */
  Vector<float3> positions;
  /* Original point index. */
  Vector<int> src_indices;
};

static bke::CurvesGeometry create_curves_outline(const bke::greasepencil::Drawing &drawing,
                                                 const IndexMask &curves_mask,
                                                 const int subdivisions)
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

  threading::EnumerableThreadSpecific<PerimeterData> thread_data;
  curves_mask.foreach_index([&](const int64_t curve_i) {
    PerimeterData &data = thread_data.local();

    const int prev_point_num = data.positions.size();
    const IndexRange points = src_curves.points_by_curve()[curve_i];
    const float normal_offset = 0.0f;
    generate_stroke_perimeter(src_positions,
                              src_radii,
                              points,
                              subdivisions,
                              src_cyclic[curve_i],
                              eGPDstroke_Caps(src_start_caps[curve_i]),
                              eGPDstroke_Caps(src_end_caps[curve_i]),
                              normal_offset,
                              data.positions,
                              data.src_indices);
    data.point_counts.append(data.positions.size() - prev_point_num);
  });

  int dst_curve_num = 0;
  int dst_point_num = 0;
  for (const PerimeterData &data : thread_data) {
    BLI_assert(data.positions.size() == data.src_indices.size());
    dst_curve_num += data.point_counts.size();
    dst_point_num += data.positions.size();
  }

  bke::CurvesGeometry dst_curves(dst_point_num, dst_curve_num);
  bke::MutableAttributeAccessor dst_attributes = dst_curves.attributes_for_write();
  bke::SpanAttributeWriter<bool> dst_cyclic = dst_attributes.lookup_or_add_for_write_span<bool>(
      "cyclic", bke::AttrDomain::Curve);
  bke::SpanAttributeWriter<float> dst_radius = dst_attributes.lookup_or_add_for_write_span<float>(
      "radius", bke::AttrDomain::Point);

  for (const PerimeterData &data : thread_data) {
    BLI_assert(data.positions.size() == data.src_indices.size());
    dst_curve_num += data.point_counts.size();
    dst_point_num += data.positions.size();
  }

  return dst_curves;
}

static void modify_drawing(const GreasePencilOutlineModifierData &omd,
                           const ModifierEvalContext &ctx,
                           bke::greasepencil::Drawing &drawing)
{
  if (drawing.strokes().curve_num == 0) {
    return;
  }

  const int subdivisions = std::max(omd.subdiv, 0);
  /* Selected source curves. */
  IndexMaskMemory curve_mask_memory;
  const IndexMask curves_mask = modifier::greasepencil::get_filtered_stroke_mask(
      ctx.object, drawing.strokes(), omd.influence, curve_mask_memory);

  drawing.strokes_for_write() = create_curves_outline(drawing, curves_mask, subdivisions);
  drawing.tag_topology_changed();
}

static void modify_geometry_set(ModifierData *md,
                                const ModifierEvalContext *ctx,
                                bke::GeometrySet *geometry_set)
{
  using bke::greasepencil::Drawing;

  auto *omd = reinterpret_cast<GreasePencilOutlineModifierData *>(md);

  if (!geometry_set->has_grease_pencil()) {
    return;
  }
  GreasePencil &grease_pencil = *geometry_set->get_grease_pencil_for_write();
  const int frame = grease_pencil.runtime->eval_frame;

  IndexMaskMemory mask_memory;
  const IndexMask layer_mask = modifier::greasepencil::get_filtered_layer_mask(
      grease_pencil, omd->influence, mask_memory);

  const Vector<Drawing *> drawings = modifier::greasepencil::get_drawings_for_write(
      grease_pencil, layer_mask, frame);
  threading::parallel_for_each(drawings,
                               [&](Drawing *drawing) { modify_drawing(*omd, *ctx, *drawing); });
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
