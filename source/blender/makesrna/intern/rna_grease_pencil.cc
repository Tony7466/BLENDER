/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup RNA
 */

#include "BLI_string.h"

#include "DNA_curves_types.h"
#include "DNA_grease_pencil_types.h"
#include "DNA_scene_types.h"

#include "RNA_access.hh"
#include "RNA_define.hh"
#include "RNA_enum_types.hh"

#include "rna_internal.hh"

#include "WM_api.hh"

#ifdef RNA_RUNTIME

#  include <fmt/format.h>

#  include "BKE_attribute.hh"
#  include "BKE_curves.hh"
#  include "BKE_customdata.hh"
#  include "BKE_grease_pencil.hh"

#  include "BLI_cpp_type.hh"
#  include "BLI_span.hh"

#  include "DEG_depsgraph.hh"
#  include "DEG_depsgraph_build.hh"

static GreasePencil *rna_grease_pencil(const PointerRNA *ptr)
{
  return reinterpret_cast<GreasePencil *>(ptr->owner_id);
}

static void rna_grease_pencil_update(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  DEG_id_tag_update(&rna_grease_pencil(ptr)->id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, rna_grease_pencil(ptr));
}

static void rna_grease_pencil_tag_redraw(ID *id, GreasePencilDrawing * /*drawing*/)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(id);

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);
}

static void rna_grease_pencil_autolock(Main * /*bmain*/, Scene * /*scene*/, PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  if (grease_pencil->flag & GREASE_PENCIL_AUTOLOCK_LAYERS) {
    grease_pencil->autolock_inactive_layers();
  }
  else {
    for (Layer *layer : grease_pencil->layers_for_write()) {
      layer->set_locked(false);
    }
  }

  rna_grease_pencil_update(nullptr, nullptr, ptr);
}

static void rna_grease_pencil_dependency_update(Main *bmain, Scene * /*scene*/, PointerRNA *ptr)
{
  DEG_id_tag_update(&rna_grease_pencil(ptr)->id, ID_RECALC_GEOMETRY);
  DEG_relations_tag_update(bmain);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, rna_grease_pencil(ptr));
}

static void rna_grease_pencil_layer_mask_name_get(PointerRNA *ptr, char *dst)
{
  using namespace blender;
  GreasePencilLayerMask *mask = static_cast<GreasePencilLayerMask *>(ptr->data);
  if (mask->layer_name != nullptr) {
    strcpy(dst, mask->layer_name);
  }
  else {
    dst[0] = '\0';
  }
}

static int rna_grease_pencil_layer_mask_name_length(PointerRNA *ptr)
{
  using namespace blender;
  GreasePencilLayerMask *mask = static_cast<GreasePencilLayerMask *>(ptr->data);
  if (mask->layer_name != nullptr) {
    return strlen(mask->layer_name);
  }
  return 0;
}

static void rna_grease_pencil_layer_mask_name_set(PointerRNA *ptr, const char *value)
{
  using namespace blender;
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  GreasePencilLayerMask *mask = static_cast<GreasePencilLayerMask *>(ptr->data);

  const std::string oldname(mask->layer_name);
  if (bke::greasepencil::TreeNode *node = grease_pencil->find_node_by_name(oldname)) {
    grease_pencil->rename_node(*node, value);
  }
}

static int rna_grease_pencil_active_mask_index_get(PointerRNA *ptr)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  return layer->active_mask_index;
}

static void rna_grease_pencil_active_mask_index_set(PointerRNA *ptr, int value)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  layer->active_mask_index = value;
}

static void rna_grease_pencil_active_mask_index_range(
    PointerRNA *ptr, int *min, int *max, int * /*softmin*/, int * /*softmax*/)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  *min = 0;
  *max = max_ii(0, BLI_listbase_count(&layer->masks) - 1);
}

static void rna_iterator_grease_pencil_layers_begin(CollectionPropertyIterator *iter,
                                                    PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  blender::Span<Layer *> layers = grease_pencil->layers_for_write();

  rna_iterator_array_begin(
      iter, (void *)layers.data(), sizeof(Layer *), layers.size(), 0, nullptr);
}

static int rna_iterator_grease_pencil_layers_length(PointerRNA *ptr)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  return grease_pencil->layers().size();
}

static void tree_node_name_get(blender::bke::greasepencil::TreeNode &node, char *dst)
{
  if (!node.name().is_empty()) {
    strcpy(dst, node.name().c_str());
  }
  else {
    dst[0] = '\0';
  }
}

static int tree_node_name_length(blender::bke::greasepencil::TreeNode &node)
{
  if (!node.name().is_empty()) {
    return node.name().size();
  }
  return 0;
}

static std::optional<std::string> tree_node_name_path(blender::bke::greasepencil::TreeNode &node,
                                                      const char *prefix)
{
  using namespace blender::bke::greasepencil;
  BLI_assert(!node.name().is_empty());
  const size_t name_length = node.name().size();
  std::string name_esc(name_length * 2, '\0');
  BLI_str_escape(name_esc.data(), node.name().c_str(), name_length * 2);
  return fmt::format("{}[\"{}\"]", prefix, name_esc.c_str());
}

static std::optional<std::string> rna_GreasePencilLayer_path(const PointerRNA *ptr)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  return tree_node_name_path(layer->wrap().as_node(), "layers");
}

static void rna_GreasePencilLayer_name_get(PointerRNA *ptr, char *value)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  tree_node_name_get(layer->wrap().as_node(), value);
}

static int rna_GreasePencilLayer_name_length(PointerRNA *ptr)
{
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);
  return tree_node_name_length(layer->wrap().as_node());
}

static void rna_GreasePencilLayer_name_set(PointerRNA *ptr, const char *value)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  GreasePencilLayer *layer = static_cast<GreasePencilLayer *>(ptr->data);

  grease_pencil->rename_node(layer->wrap().as_node(), value);
}

static int rna_GreasePencilLayer_pass_index_get(PointerRNA *ptr)
{
  using namespace blender;
  const GreasePencil &grease_pencil = *rna_grease_pencil(ptr);
  const bke::greasepencil::Layer &layer =
      static_cast<const GreasePencilLayer *>(ptr->data)->wrap();
  const int layer_idx = *grease_pencil.get_layer_index(layer);

  const VArray layer_passes = *grease_pencil.attributes().lookup_or_default<int>(
      "pass_index", bke::AttrDomain::Layer, 0);
  return layer_passes[layer_idx];
}

static void rna_GreasePencilLayer_pass_index_set(PointerRNA *ptr, int value)
{
  using namespace blender;
  GreasePencil &grease_pencil = *rna_grease_pencil(ptr);
  const bke::greasepencil::Layer &layer =
      static_cast<const GreasePencilLayer *>(ptr->data)->wrap();
  const int layer_idx = *grease_pencil.get_layer_index(layer);

  bke::SpanAttributeWriter<int> layer_passes =
      grease_pencil.attributes_for_write().lookup_or_add_for_write_span<int>(
          "pass_index", bke::AttrDomain::Layer);
  layer_passes.span[layer_idx] = std::max(0, value);
  layer_passes.finish();
}

static GreasePencilFrame *rna_GreasePencilLayer_get_frame_at(GreasePencilLayer *layer_in,
                                                             int frame_number)
{
  using namespace blender::bke::greasepencil;
  Layer &layer = *static_cast<Layer *>(layer_in);
  return layer.frame_at(frame_number);
}

static void rna_GreasePencilLayer_clear(ID *id, GreasePencilLayer *layer_in)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(id);
  Layer &layer = *static_cast<Layer *>(layer_in);

  grease_pencil.remove_frames(layer, layer.sorted_keys());

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);
}

static PointerRNA rna_GreasePencil_active_layer_get(PointerRNA *ptr)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  if (grease_pencil->has_active_layer()) {
    return rna_pointer_inherit_refine(
        ptr, &RNA_GreasePencilLayer, static_cast<void *>(grease_pencil->get_active_layer()));
  }
  return PointerRNA_NULL;
}

static void rna_GreasePencil_active_layer_set(PointerRNA *ptr,
                                              PointerRNA value,
                                              ReportList * /*reports*/)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  grease_pencil->set_active_layer(static_cast<blender::bke::greasepencil::Layer *>(value.data));
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, nullptr);
}

static PointerRNA rna_GreasePencil_active_group_get(PointerRNA *ptr)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  if (grease_pencil->has_active_group()) {
    return rna_pointer_inherit_refine(
        ptr, &RNA_GreasePencilLayerGroup, static_cast<void *>(grease_pencil->get_active_group()));
  }
  return PointerRNA_NULL;
}

static void rna_GreasePencil_active_group_set(PointerRNA *ptr,
                                              PointerRNA value,
                                              ReportList * /*reports*/)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  GreasePencilLayerTreeNode *node = static_cast<GreasePencilLayerTreeNode *>(value.data);
  if (node->wrap().is_group()) {
    grease_pencil->active_node = node;
    WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);
  }
}

static std::optional<std::string> rna_GreasePencilLayerGroup_path(const PointerRNA *ptr)
{
  GreasePencilLayerTreeGroup *group = static_cast<GreasePencilLayerTreeGroup *>(ptr->data);
  return tree_node_name_path(group->wrap().as_node(), "layer_groups");
}

static void rna_GreasePencilLayerGroup_name_get(PointerRNA *ptr, char *value)
{
  GreasePencilLayerTreeGroup *group = static_cast<GreasePencilLayerTreeGroup *>(ptr->data);
  tree_node_name_get(group->wrap().as_node(), value);
}

static int rna_GreasePencilLayerGroup_name_length(PointerRNA *ptr)
{
  GreasePencilLayerTreeGroup *group = static_cast<GreasePencilLayerTreeGroup *>(ptr->data);
  return tree_node_name_length(group->wrap().as_node());
}

static void rna_GreasePencilLayerGroup_name_set(PointerRNA *ptr, const char *value)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  GreasePencilLayerTreeGroup *group = static_cast<GreasePencilLayerTreeGroup *>(ptr->data);

  grease_pencil->rename_node(group->wrap().as_node(), value);
}

static void rna_iterator_grease_pencil_layer_groups_begin(CollectionPropertyIterator *iter,
                                                          PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);

  blender::Span<LayerGroup *> groups = grease_pencil->layer_groups_for_write();

  rna_iterator_array_begin(
      iter, (void *)groups.data(), sizeof(LayerGroup *), groups.size(), 0, nullptr);
}

static int rna_iterator_grease_pencil_layer_groups_length(PointerRNA *ptr)
{
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  return grease_pencil->layer_groups().size();
}

static void rna_GreasePencilLayer_frames_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  Layer &layer = static_cast<GreasePencilLayer *>(ptr->data)->wrap();
  blender::Span<FramesMapKey> sorted_keys = layer.sorted_keys();

  rna_iterator_array_begin(
      iter, (void *)sorted_keys.data(), sizeof(FramesMapKey), sorted_keys.size(), false, nullptr);
}

static PointerRNA rna_GreasePencilLayer_frames_get(CollectionPropertyIterator *iter)
{
  using namespace blender::bke::greasepencil;
  const FramesMapKey frame_key = *static_cast<FramesMapKey *>(rna_iterator_array_get(iter));
  const Layer &layer = static_cast<GreasePencilLayer *>(iter->parent.data)->wrap();
  const GreasePencilFrame *frame = layer.frames().lookup_ptr(frame_key);
  return rna_pointer_inherit_refine(&iter->parent,
                                    &RNA_GreasePencilFrame,
                                    static_cast<void *>(const_cast<GreasePencilFrame *>(frame)));
}

static int rna_GreasePencilLayer_frames_length(PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  Layer &layer = static_cast<GreasePencilLayer *>(ptr->data)->wrap();
  return layer.frames().size();
}

static bool rna_GreasePencilLayer_frames_lookup_int(PointerRNA *ptr, int index, PointerRNA *r_ptr)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *rna_grease_pencil(ptr);
  Layer &layer = static_cast<GreasePencilLayer *>(ptr->data)->wrap();
  if (index < 0 || index >= layer.sorted_keys().size()) {
    return false;
  }
  const FramesMapKey frame_key = layer.sorted_keys()[index];
  const GreasePencilFrame *frame = layer.frames().lookup_ptr(frame_key);

  r_ptr->owner_id = &grease_pencil.id;
  r_ptr->type = &RNA_GreasePencilFrame;
  r_ptr->data = static_cast<void *>(const_cast<GreasePencilFrame *>(frame));
  return true;
}

static GreasePencilFrame *rna_Frames_frame_new(ID *id,
                                               GreasePencilLayer *layer_in,
                                               ReportList *reports,
                                               int frame_number)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(id);
  Layer &layer = static_cast<GreasePencilLayer *>(layer_in)->wrap();

  if (layer.frames().contains(frame_number)) {
    BKE_reportf(reports, RPT_ERROR, "Frame already exists on frame number %d", frame_number);
    return nullptr;
  }

  grease_pencil.insert_frame(layer, frame_number, 0, BEZT_KEYTYPE_KEYFRAME);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);

  return layer.frame_at(frame_number);
}

static void rna_Frames_frame_remove(ID *id,
                                    GreasePencilLayer *layer_in,
                                    ReportList *reports,
                                    int frame_number)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(id);
  Layer &layer = static_cast<GreasePencilLayer *>(layer_in)->wrap();

  if (!layer.frames().contains(frame_number)) {
    BKE_reportf(reports, RPT_ERROR, "Frame doesn't exists on frame number %d", frame_number);
    return;
  }

  if (grease_pencil.remove_frames(layer, {frame_number})) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);
  }
}

static GreasePencilFrame *rna_Frames_frame_copy(ID *id,
                                                GreasePencilLayer *layer_in,
                                                ReportList *reports,
                                                int from_frame_number,
                                                int to_frame_number,
                                                bool instance_drawing)
{
  using namespace blender::bke::greasepencil;
  GreasePencil &grease_pencil = *reinterpret_cast<GreasePencil *>(id);
  Layer &layer = static_cast<GreasePencilLayer *>(layer_in)->wrap();

  if (!layer.frames().contains(from_frame_number)) {
    BKE_reportf(reports, RPT_ERROR, "Frame doesn't exists on frame number %d", from_frame_number);
    return nullptr;
  }
  if (layer.frames().contains(to_frame_number)) {
    BKE_reportf(reports, RPT_ERROR, "Frame already exists on frame number %d", to_frame_number);
    return nullptr;
  }

  grease_pencil.insert_duplicate_frame(
      layer, from_frame_number, to_frame_number, instance_drawing);
  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, &grease_pencil);

  return layer.frame_at(to_frame_number);
}

static PointerRNA rna_Frame_drawing_get(PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  const GreasePencil &grease_pencil = *rna_grease_pencil(ptr);
  GreasePencilFrame &this_frame = *static_cast<GreasePencilFrame *>(ptr->data);
  if (this_frame.drawing_index == -1) {
    return PointerRNA_NULL;
  }

  /* RNA doesn't give access to the parented layer object, so we have to iterate over all layers
   * and search for the matching GreasePencilFrame pointer in the frames collection. */
  Layer *this_layer = nullptr;
  int frame_number = 0;
  for (const Layer *layer : grease_pencil.layers()) {
    layer->frames().foreach_item(
        [&](const FramesMapKey frame_key, const GreasePencilFrame &frame) {
          if (&frame == &this_frame) {
            this_layer = const_cast<Layer *>(layer);
            frame_number = int(frame_key);
            return;
          }
        });
  }
  if (this_layer == nullptr) {
    return PointerRNA_NULL;
  }

  const Drawing *drawing = grease_pencil.get_drawing_at(*this_layer, frame_number);

  return rna_pointer_inherit_refine(
      ptr, &RNA_GreasePencilDrawing, static_cast<void *>(const_cast<Drawing *>(drawing)));
}

static int rna_Frame_frame_number_get(PointerRNA *ptr)
{
  using namespace blender::bke::greasepencil;
  const GreasePencil &grease_pencil = *rna_grease_pencil(ptr);
  GreasePencilFrame &frame_to_find = *static_cast<GreasePencilFrame *>(ptr->data);
  int frame_number = 0;

  /* RNA doesn't give access to the parented layer object, so we have to iterate over all layers
   * and search for the matching GreasePencilFrame pointer in the frames collection. */
  for (const Layer *layer : grease_pencil.layers()) {
    layer->frames().foreach_item(
        [&](const FramesMapKey frame_key, const GreasePencilFrame &frame) {
          if (&frame == &frame_to_find) {
            frame_number = int(frame_key);
            return;
          }
        });
  }

  return frame_number;
}

static void rna_Frame_frame_number_index_range(
    PointerRNA * /*ptr*/, int *min, int *max, int * /*softmin*/, int * /*softmax*/)
{
  *min = MINAFRAME;
  *max = MAXFRAME;
}

static void rna_Drawing_add_strokes(GreasePencilDrawing *drawing_in,
                                    ReportList *reports,
                                    const int *sizes,
                                    const int sizes_num)
{
  using namespace blender;
  if (std::any_of(sizes, sizes + sizes_num, [](const int size) { return size < 1; })) {
    BKE_report(reports, RPT_ERROR, "Stroke sizes must be greater than zero");
    return;
  }

  bke::CurvesGeometry &curves = drawing_in->geometry.wrap();

  const int orig_points_num = curves.points_num();
  const int orig_curves_num = curves.curves_num();
  curves.resize(orig_points_num, orig_curves_num + sizes_num);

  /* Find the final number of points by accumulating the new. */
  MutableSpan<int> new_offsets = curves.offsets_for_write().drop_front(orig_curves_num);
  new_offsets.drop_back(1).copy_from({sizes, sizes_num});
  offset_indices::accumulate_counts_to_offsets(new_offsets, orig_points_num);

  curves.resize(curves.offsets().last(), curves.curves_num());

  /* Initialize new attribute values, since #CurvesGeometry::resize() doesn't do that. */
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();
  bke::fill_attribute_range_default(
      attributes, bke::AttrDomain::Point, {}, curves.points_range().drop_front(orig_points_num));
  bke::fill_attribute_range_default(
      attributes, bke::AttrDomain::Curve, {}, curves.curves_range().drop_front(orig_curves_num));

  curves.update_curve_types();

  bke::greasepencil::Drawing &drawing =
      reinterpret_cast<GreasePencilDrawing *>(drawing_in)->wrap();
  drawing.tag_topology_changed();

  WM_main_add_notifier(NC_GPENCIL | NA_EDITED, nullptr);
}

static blender::bke::greasepencil::Drawing *rna_grease_pencil_drawing(const PointerRNA *ptr)
{
  return reinterpret_cast<blender::bke::greasepencil::Drawing *>(ptr->data);
}

static int domain_num(const blender::bke::CurvesGeometry &curves,
                      const blender::bke::AttrDomain domain)
{
  return domain == blender::bke::AttrDomain::Point ? curves.points_num() : curves.curves_num();
}
static CustomData &domain_custom_data(blender::bke::CurvesGeometry &curves,
                                      const blender::bke::AttrDomain domain)
{
  return domain == blender::bke::AttrDomain::Point ? curves.point_data : curves.curve_data;
}

template<typename T>
static PointerRNA rna_attribute_array_get(PointerRNA *ptr,
                                          const blender::bke::AttrDomain domain,
                                          const blender::StringRef name,
                                          const T default_value = T())
{
  blender::bke::greasepencil::Drawing *drawing = rna_grease_pencil_drawing(ptr);
  blender::bke::CurvesGeometry &curves = drawing->strokes_for_write();
  const int num = domain_num(curves, domain);
  if (num <= 0) {
    return PointerRNA_NULL;
  }

  const eCustomDataType type = blender::bke::cpp_type_to_custom_data_type(
      blender::CPPType::get<T>());
  CustomData &custom_data = domain_custom_data(curves, domain);

  int layer_index = CustomData_get_named_layer_index(&custom_data, type, name);
  if (layer_index == -1) {
    T *data = (T *)CustomData_add_layer_named(&custom_data, type, CD_SET_DEFAULT, num, name);
    blender::MutableSpan<T> span = {data, num};
    if (span.first() != default_value) {
      span.fill(default_value);
    }
    layer_index = CustomData_get_named_layer_index(&custom_data, type, name);
  }

  CustomDataLayer &layer = custom_data.layers[layer_index];
  CustomData_ensure_data_is_mutable(&layer, num);
  layer.length = num;
  return rna_pointer_inherit_refine(ptr, &RNA_AttributeArray, static_cast<void *>(&layer));
}

/** Curve domain. */
static PointerRNA rna_attribute_array_cyclic_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<bool>(ptr, blender::bke::AttrDomain::Curve, "cyclic", false);
}

static PointerRNA rna_attribute_array_fill_colors_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<blender::ColorGeometry4f>(
      ptr,
      blender::bke::AttrDomain::Curve,
      "fill_color",
      blender::ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f));
}

static PointerRNA rna_attribute_array_start_caps_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<int8_t>(
      ptr, blender::bke::AttrDomain::Curve, "start_cap", GP_STROKE_CAP_TYPE_ROUND);
}

static PointerRNA rna_attribute_array_end_caps_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<int8_t>(
      ptr, blender::bke::AttrDomain::Curve, "end_cap", GP_STROKE_CAP_TYPE_ROUND);
}

static PointerRNA rna_attribute_array_softnesses_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Curve, "softness", 0.0f);
}

static PointerRNA rna_attribute_array_aspect_ratios_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(
      ptr, blender::bke::AttrDomain::Curve, "aspect_ratio", 1.0f);
}

static PointerRNA rna_attribute_array_material_indices_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<int>(ptr, blender::bke::AttrDomain::Curve, "material_index", 0);
}

static PointerRNA rna_attribute_array_init_times_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Curve, "init_time", 0.0f);
}

static PointerRNA rna_attribute_array_stroke_selections_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<bool>(ptr, blender::bke::AttrDomain::Curve, ".selection", true);
}

/** Point domain. */
static PointerRNA rna_attribute_array_positions_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<blender::float3>(
      ptr, blender::bke::AttrDomain::Point, "position", blender::float3(0.0f, 0.0f, 0.0f));
}

static PointerRNA rna_attribute_array_radii_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Point, "radius", 0.01f);
}

static PointerRNA rna_attribute_array_opacities_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Point, "opacity", 0.0f);
}

static PointerRNA rna_attribute_array_vertex_colors_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<blender::ColorGeometry4f>(
      ptr,
      blender::bke::AttrDomain::Point,
      "vertex_color",
      blender::ColorGeometry4f(0.0f, 0.0f, 0.0f, 0.0f));
}

static PointerRNA rna_attribute_array_rotations_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Point, "rotation", 0.0f);
}

static PointerRNA rna_attribute_array_delta_times_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<float>(ptr, blender::bke::AttrDomain::Point, "delta_time", 0.0f);
}

static PointerRNA rna_attribute_array_point_selections_get(PointerRNA *ptr)
{
  return rna_attribute_array_get<bool>(ptr, blender::bke::AttrDomain::Point, ".selection", true);
}

static void rna_Drawing_strokes_begin(CollectionPropertyIterator *iter, PointerRNA *ptr)
{
  blender::bke::greasepencil::Drawing *drawing = rna_grease_pencil_drawing(ptr);
  rna_iterator_array_begin(iter,
                           drawing->geometry.wrap().offsets_for_write().data(),
                           sizeof(int),
                           drawing->geometry.curve_num,
                           false,
                           nullptr);
}

static bool rna_Drawing_strokes_lookup_int(PointerRNA *ptr, int index, PointerRNA *r_ptr)
{
  using namespace blender::bke::greasepencil;
  GreasePencil *grease_pencil = rna_grease_pencil(ptr);
  Drawing *drawing = rna_grease_pencil_drawing(ptr);
  if (index < 0 || index >= drawing->geometry.curve_num) {
    return false;
  }
  r_ptr->owner_id = &grease_pencil->id;
  r_ptr->type = &RNA_Stroke;
  r_ptr->data = &drawing->geometry.wrap().offsets_for_write()[index];
  return true;
}

static int rna_Drawing_strokes_length(PointerRNA *ptr)
{
  blender::bke::greasepencil::Drawing *drawing = rna_grease_pencil_drawing(ptr);
  return drawing->geometry.curve_num;
}

static int rna_Drawing_points_length(PointerRNA *ptr)
{
  blender::bke::greasepencil::Drawing *drawing = rna_grease_pencil_drawing(ptr);
  return drawing->geometry.point_num;
}

static int rna_Stroke_start_point_get(PointerRNA *ptr)
{
  const int *offset_ptr = static_cast<int *>(ptr->data);
  return *offset_ptr;
}

static int rna_Stroke_stop_point_get(PointerRNA *ptr)
{
  const int *offset_ptr = static_cast<int *>(ptr->data);
  return *(offset_ptr + 1);
}

#else

static void rna_def_grease_pencil_drawing_stroke(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "Stroke", nullptr);
  RNA_def_struct_ui_text(srna,
                         "Stroke Point Indices",
                         "The start and stop point indices of a single stroke in a drawing, for "
                         "use in range(start, stop) and slices [start:stop]");

  prop = RNA_def_property(srna, "start", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_int_funcs(prop, "rna_Stroke_start_point_get", nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Start Point Index",
                           "The index of this stroke's start point, e.g. in `range(start, stop)` "
                           "or `array[start:stop]`");

  prop = RNA_def_property(srna, "stop", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_int_funcs(prop, "rna_Stroke_stop_point_get", nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stop Point Index",
                           "The index of this stroke's stop point, e.g. in `range(start, stop)` "
                           "or `array[start:stop]`");
}

static void rna_def_grease_pencil_drawing_api(StructRNA *srna)
{
  FunctionRNA *func;
  PropertyRNA *parm;

  func = RNA_def_function(srna, "tag_redraw", "rna_grease_pencil_tag_redraw");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID);
  RNA_def_function_ui_description(
      func, "Redraw the Grease Pencil object to reflect changes made in the drawing attributes");

  func = RNA_def_function(srna, "add_strokes", "rna_Drawing_add_strokes");
  RNA_def_function_flag(func, FUNC_USE_REPORTS);
  parm = RNA_def_int_array(func,
                           "sizes",
                           1,
                           nullptr,
                           0,
                           INT_MAX,
                           "Sizes",
                           "The number of points in each stroke",
                           1,
                           10000);
  RNA_def_property_array(parm, RNA_MAX_ARRAY_LENGTH);
  RNA_def_parameter_flags(parm, PROP_DYNAMIC, PARM_REQUIRED);
}

static void rna_def_grease_pencil_drawing(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static const EnumPropertyItem rna_enum_drawing_type_items[] = {
      {GP_DRAWING, "DRAWING", 0, "Drawing", ""},
      {GP_DRAWING_REFERENCE, "REFERENCE", 0, "Reference", ""},
      {0, nullptr, 0, nullptr, nullptr}};

  srna = RNA_def_struct(brna, "GreasePencilDrawing", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilDrawing");
  RNA_def_struct_ui_text(srna, "Grease Pencil Drawing", "A Grease Pencil drawing");

  /* Type. */
  prop = RNA_def_property(srna, "type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "base.type");
  RNA_def_property_enum_items(prop, rna_enum_drawing_type_items);
  /* TODO: Make this property editable when drawing references are supported. */
  RNA_def_parameter_clear_flags(prop, PROP_EDITABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Type", "Drawing type");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Number of strokes. */
  prop = RNA_def_property(srna, "num_strokes", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_funcs(prop, "rna_Drawing_strokes_length", nullptr, nullptr);
  RNA_def_parameter_clear_flags(prop, PROP_EDITABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Number of Strokes", "The number of strokes in the drawing");

  /* Number of points. */
  prop = RNA_def_property(srna, "num_points", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_int_funcs(prop, "rna_Drawing_points_length", nullptr, nullptr);
  RNA_def_parameter_clear_flags(prop, PROP_EDITABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Number of Points", "The number of stroke points in the drawing");

  /* Stroke points: indices of stroke's start and stop point. */
  prop = RNA_def_property(srna, "strokes", PROP_COLLECTION, PROP_NONE);
  RNA_def_parameter_clear_flags(prop, PROP_EDITABLE, ParameterFlag(0));
  RNA_def_property_collection_funcs(prop,
                                    "rna_Drawing_strokes_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_get",
                                    "rna_Drawing_strokes_length",
                                    "rna_Drawing_strokes_lookup_int",
                                    nullptr,
                                    nullptr);
  RNA_def_property_struct_type(prop, "Stroke");
  RNA_def_property_ui_text(
      prop, "Stroke Points", "The start and stop point indices of all strokes in the drawing");

  rna_def_grease_pencil_drawing_api(srna);

  /* Built-in attributes for curves. */

  /* Cyclic. */
  prop = RNA_def_property(srna, "cyclic", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_cyclic_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stroke Cyclic",
                           "The cyclic state of all strokes in the drawing. When a stroke is "
                           "cyclic, the start and end point are connected");

  /* Fill colors. */
  prop = RNA_def_property(srna, "fill_colors", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_fill_colors_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Fill Colors", "The fill colors of all strokes in the drawing");

  /* Start caps. */
  prop = RNA_def_property(srna, "start_caps", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_start_caps_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Start Caps", "The shape of the start captions of all strokes in the drawing");

  /* End caps. */
  prop = RNA_def_property(srna, "end_caps", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_end_caps_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke End Caps", "The shape of the end captions of all strokes in the drawing");

  /* Softnesses. */
  prop = RNA_def_property(srna, "softnesses", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_softnesses_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stroke Softnesses",
                           "The softnesses of all strokes in the drawing. Used to render a soft "
                           "gradient from the center line of the stroke to the edge");

  /* Aspect ratios. */
  prop = RNA_def_property(srna, "aspect_ratios", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_aspect_ratios_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Aspect Ratios", "The aspect ratios of textures on all strokes in the drawing");

  /* Material indices. */
  prop = RNA_def_property(srna, "material_indices", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_material_indices_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Material Indices", "The material indices of all strokes in the drawing");

  /* Init times. */
  prop = RNA_def_property(srna, "init_times", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_init_times_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stroke Init Times",
                           "The initial drawing timestamps of all strokes in the drawing. Used by "
                           "the build modifier to 'replay' how the stroke was drawn");

  /* Selection state. */
  prop = RNA_def_property(srna, "stroke_selections", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_stroke_selections_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Selections", "The selection state of all strokes in the drawing");

  /* Built-in attributes for curve points. */

  /* Positions. */
  prop = RNA_def_property(srna, "positions", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_positions_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Point Positions", "The positions of all points in the drawing");

  /* Radii. */
  prop = RNA_def_property(srna, "radii", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(prop, "rna_attribute_array_radii_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop, "Stroke Point Radii", "The radii of all points in the drawing");

  /* Opacities. */
  prop = RNA_def_property(srna, "opacities", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_opacities_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Point Opacities", "The opacities of all points in the drawing");

  /* Vertex colors. */
  prop = RNA_def_property(srna, "vertex_colors", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_vertex_colors_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Point Vertex Colors", "The vertex colors of all points in the drawing");

  /* Texture rotations. */
  prop = RNA_def_property(srna, "rotations", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_rotations_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stroke Point Texture Rotations",
                           "The texture rotations of all points in the drawing");

  /* Delta times. */
  prop = RNA_def_property(srna, "delta_times", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_delta_times_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop,
                           "Stroke Point Delta Times",
                           "The time difference between the moment a point was added and the "
                           "moment the previous point was added. Used by the build modifier to "
                           "'replay' how the stroke was drawn");

  /* Selection state. */
  prop = RNA_def_property(srna, "point_selections", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "AttributeArray");
  RNA_def_property_pointer_funcs(
      prop, "rna_attribute_array_point_selections_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(
      prop, "Stroke Point Selections", "The selection state of all points in the drawing");
}

static void rna_def_grease_pencil_frame(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "GreasePencilFrame", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilFrame");
  RNA_def_struct_ui_text(srna, "Grease Pencil Frame", "A Grease Pencil keyframe");

  /* Drawing. */
  prop = RNA_def_property(srna, "drawing", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilDrawing");
  /* TODO: Make drawing editable. */
  RNA_def_property_pointer_funcs(prop, "rna_Frame_drawing_get", nullptr, nullptr, nullptr);
  RNA_def_property_ui_text(prop, "Drawing", "A Grease Pencil drawing");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Frame number. */
  prop = RNA_def_property(srna, "frame_number", PROP_INT, PROP_NONE);
  /* TODO: Make property editable, ensure frame number isn't already in use. */
  RNA_def_property_clear_flag(prop, PROP_EDITABLE);
  RNA_def_property_int_funcs(
      prop, "rna_Frame_frame_number_get", nullptr, "rna_Frame_frame_number_index_range");
  RNA_def_property_range(prop, MINAFRAME, MAXFRAME);
  RNA_def_property_ui_text(prop, "Frame Number", "The frame on which the drawing appears");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Selection status. */
  prop = RNA_def_property(srna, "select", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GP_FRAME_SELECTED);
  RNA_def_property_ui_text(prop, "Select", "Frame is selected for editing in the Dope Sheet");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");
}

static void rna_def_grease_pencil_frames_api(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;

  FunctionRNA *func;
  PropertyRNA *parm;

  RNA_def_property_srna(cprop, "GreasePencilFrames");
  srna = RNA_def_struct(brna, "GreasePencilFrames", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilLayer");
  RNA_def_struct_ui_text(srna, "Grease Pencil Frames", "Collection of Grease Pencil frames");

  func = RNA_def_function(srna, "new", "rna_Frames_frame_new");
  RNA_def_function_ui_description(func, "Add a new Grease Pencil frame");
  RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
  parm = RNA_def_int(func,
                     "frame_number",
                     1,
                     MINAFRAME,
                     MAXFRAME,
                     "Frame Number",
                     "The frame on which the drawing appears",
                     MINAFRAME,
                     MAXFRAME);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_pointer(func, "frame", "GreasePencilFrame", "", "The newly created frame");
  RNA_def_function_return(func, parm);

  func = RNA_def_function(srna, "remove", "rna_Frames_frame_remove");
  RNA_def_function_ui_description(func, "Remove a Grease Pencil frame");
  RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
  parm = RNA_def_int(func,
                     "frame_number",
                     1,
                     MINAFRAME,
                     MAXFRAME,
                     "Frame Number",
                     "The frame number of the frame to remove",
                     MINAFRAME,
                     MAXFRAME);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);

  func = RNA_def_function(srna, "copy", "rna_Frames_frame_copy");
  RNA_def_function_ui_description(func, "Copy a Grease Pencil frame");
  RNA_def_function_flag(func, FUNC_USE_REPORTS | FUNC_USE_SELF_ID);
  parm = RNA_def_int(func,
                     "from_frame_number",
                     1,
                     MINAFRAME,
                     MAXFRAME,
                     "Source Frame Number",
                     "The frame number of the source frame",
                     MINAFRAME,
                     MAXFRAME);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_int(func,
                     "to_frame_number",
                     2,
                     MINAFRAME,
                     MAXFRAME,
                     "Frame Number of Copy",
                     "The frame number to copy the frame to",
                     MINAFRAME,
                     MAXFRAME);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_boolean(func,
                         "instance_drawing",
                         false,
                         "Instance Drawing",
                         "Let the copied frame use the same drawing as the source");
  parm = RNA_def_pointer(func, "copy", "GreasePencilFrame", "", "The newly copied frame");
  RNA_def_function_return(func, parm);
}

static void rna_def_grease_pencil_layers_mask_api(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  PropertyRNA *prop;

  RNA_def_property_srna(cprop, "GreasePencilLayerMasks");
  srna = RNA_def_struct(brna, "GreasePencilLayerMasks", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilLayer");
  RNA_def_struct_ui_text(
      srna, "Grease Pencil Mask Layers", "Collection of grease pencil masking layers");

  prop = RNA_def_property(srna, "active_mask_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_int_funcs(prop,
                             "rna_grease_pencil_active_mask_index_get",
                             "rna_grease_pencil_active_mask_index_set",
                             "rna_grease_pencil_active_mask_index_range");
  RNA_def_property_ui_text(prop, "Active Layer Mask Index", "Active index in layer mask array");
}

static void rna_def_grease_pencil_layer_mask(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "GreasePencilLayerMask", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilLayerMask");
  RNA_def_struct_ui_text(srna, "Grease Pencil Masking Layers", "List of Mask Layers");
  // RNA_def_struct_path_func(srna, "rna_GreasePencilLayerMask_path");

  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Layer", "Mask layer name");
  RNA_def_property_string_sdna(prop, nullptr, "layer_name");
  RNA_def_property_string_funcs(prop,
                                "rna_grease_pencil_layer_mask_name_get",
                                "rna_grease_pencil_layer_mask_name_length",
                                "rna_grease_pencil_layer_mask_name_set");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_clear_flag(prop, PROP_ANIMATABLE);
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA | NA_RENAME, nullptr);

  prop = RNA_def_property(srna, "hide", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GP_LAYER_MASK_HIDE);
  RNA_def_property_ui_icon(prop, ICON_HIDE_OFF, -1);
  RNA_def_property_ui_text(prop, "Hide", "Set mask Visibility");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "invert", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GP_LAYER_MASK_INVERT);
  RNA_def_property_ui_icon(prop, ICON_SELECT_INTERSECT, 1);
  RNA_def_property_ui_text(prop, "Invert", "Invert mask");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");
}

static void rna_def_grease_pencil_layer(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static const float scale_defaults[3] = {1.0f, 1.0f, 1.0f};

  static const EnumPropertyItem rna_enum_layer_blend_modes_items[] = {
      {GP_LAYER_BLEND_NONE, "REGULAR", 0, "Regular", ""},
      {GP_LAYER_BLEND_HARDLIGHT, "HARDLIGHT", 0, "Hard Light", ""},
      {GP_LAYER_BLEND_ADD, "ADD", 0, "Add", ""},
      {GP_LAYER_BLEND_SUBTRACT, "SUBTRACT", 0, "Subtract", ""},
      {GP_LAYER_BLEND_MULTIPLY, "MULTIPLY", 0, "Multiply", ""},
      {GP_LAYER_BLEND_DIVIDE, "DIVIDE", 0, "Divide", ""},
      {0, nullptr, 0, nullptr, nullptr}};

  srna = RNA_def_struct(brna, "GreasePencilLayer", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilLayer");
  RNA_def_struct_ui_text(srna, "Grease Pencil Layer", "Collection of related drawings");
  RNA_def_struct_path_func(srna, "rna_GreasePencilLayer_path");

  /* Frames. */
  prop = RNA_def_property(srna, "frames", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilFrame");
  RNA_def_property_ui_text(prop, "Frames", "Grease Pencil frames");
  RNA_def_property_collection_funcs(prop,
                                    "rna_GreasePencilLayer_frames_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_GreasePencilLayer_frames_get",
                                    "rna_GreasePencilLayer_frames_length",
                                    "rna_GreasePencilLayer_frames_lookup_int",
                                    nullptr,
                                    nullptr);
  rna_def_grease_pencil_frames_api(brna, prop);

  /* Name. */
  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Name", "Layer name");
  RNA_def_property_string_funcs(prop,
                                "rna_GreasePencilLayer_name_get",
                                "rna_GreasePencilLayer_name_length",
                                "rna_GreasePencilLayer_name_set");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA | NA_RENAME, "rna_grease_pencil_update");

  /* Mask Layers. */
  prop = RNA_def_property(srna, "mask_layers", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "masks", nullptr);
  RNA_def_property_struct_type(prop, "GreasePencilLayerMask");
  RNA_def_property_ui_text(prop, "Masks", "List of Masking Layers");
  rna_def_grease_pencil_layers_mask_api(brna, prop);

  /* Visibility. */
  prop = RNA_def_property(srna, "hide", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE);
  RNA_def_property_ui_icon(prop, ICON_HIDE_OFF, -1);
  RNA_def_property_ui_text(prop, "Hide", "Set layer visibility");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Lock. */
  prop = RNA_def_property(srna, "lock", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_LOCKED);
  RNA_def_property_ui_icon(prop, ICON_UNLOCKED, 1);
  RNA_def_property_ui_text(
      prop, "Locked", "Protect layer from further editing and/or frame changes");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Opacity. */
  prop = RNA_def_property(srna, "opacity", PROP_FLOAT, PROP_FACTOR);
  RNA_def_property_float_sdna(prop, "GreasePencilLayer", "opacity");
  RNA_def_property_ui_text(prop, "Opacity", "Layer Opacity");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Onion Skinning. */
  prop = RNA_def_property(srna, "use_onion_skinning", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_icon(prop, ICON_ONIONSKIN_OFF, 1);
  RNA_def_property_boolean_negative_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE_ONION_SKINNING);
  RNA_def_property_ui_text(
      prop, "Onion Skinning", "Display onion skins before and after the current frame");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Use Masks. */
  prop = RNA_def_property(srna, "use_masks", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_icon(prop, ICON_CLIPUV_HLT, -1);
  RNA_def_property_boolean_negative_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE_MASKS);
  RNA_def_property_ui_text(
      prop,
      "Use Masks",
      "The visibility of drawings on this layer is affected by the layers in its masks list");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Use Lights. */
  prop = RNA_def_property(srna, "use_lights", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_USE_LIGHTS);
  RNA_def_property_ui_text(
      prop, "Use Lights", "Enable the use of lights on stroke and fill materials");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Pass index for compositing and modifiers. */
  prop = RNA_def_property(srna, "pass_index", PROP_INT, PROP_UNSIGNED);
  RNA_def_property_ui_text(prop, "Pass Index", "Index number for the \"Layer Index\" pass");
  RNA_def_property_int_funcs(prop,
                             "rna_GreasePencilLayer_pass_index_get",
                             "rna_GreasePencilLayer_pass_index_set",
                             nullptr);
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Parent. */
  prop = RNA_def_property(srna, "parent", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "Object");
  RNA_def_property_flag(prop, PROP_EDITABLE | PROP_ID_SELF_CHECK);
  RNA_def_property_override_flag(prop, PROPOVERRIDE_OVERRIDABLE_LIBRARY);
  RNA_def_property_ui_text(prop, "Parent", "Parent object");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_dependency_update");

  /* Parent Bone. */
  prop = RNA_def_property(srna, "parent_bone", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "parsubstr");
  RNA_def_property_ui_text(
      prop, "Parent Bone", "Name of parent bone. Only used when the parent object is an armature");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_dependency_update");

  /* Translation. */
  prop = RNA_def_property(srna, "translation", PROP_FLOAT, PROP_TRANSLATION);
  RNA_def_property_array(prop, 3);
  RNA_def_property_float_sdna(prop, nullptr, "translation");
  RNA_def_property_ui_range(prop, -FLT_MAX, FLT_MAX, 1, RNA_TRANSLATION_PREC_DEFAULT);
  RNA_def_property_ui_text(prop, "Translation", "Translation of the layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Rotation. */
  prop = RNA_def_property(srna, "rotation", PROP_FLOAT, PROP_EULER);
  RNA_def_property_array(prop, 3);
  RNA_def_property_float_sdna(prop, nullptr, "rotation");
  RNA_def_property_ui_range(prop, -FLT_MAX, FLT_MAX, 1, RNA_TRANSLATION_PREC_DEFAULT);
  RNA_def_property_ui_text(prop, "Rotation", "Euler rotation of the layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Scale.*/
  prop = RNA_def_property(srna, "scale", PROP_FLOAT, PROP_XYZ);
  RNA_def_property_array(prop, 3);
  RNA_def_property_float_sdna(prop, nullptr, "scale");
  RNA_def_property_float_array_default(prop, scale_defaults);
  RNA_def_property_ui_range(prop, -FLT_MAX, FLT_MAX, 1, 3);
  RNA_def_property_ui_text(prop, "Scale", "Scale of the layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* View layer for render. */
  prop = RNA_def_property(srna, "viewlayer_render", PROP_STRING, PROP_NONE);
  RNA_def_property_string_sdna(prop, nullptr, "viewlayername");
  RNA_def_property_ui_text(
      prop,
      "ViewLayer",
      "Only include Layer in this View Layer render output (leave blank to include always)");

  /* Use View layer masks. */
  prop = RNA_def_property(srna, "use_viewlayer_masks", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_negative_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_DISABLE_MASKS_IN_VIEWLAYER);
  RNA_def_property_ui_text(
      prop, "Use Masks in Render", "Include the mask layers when rendering the view-layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Blend Mode. */
  prop = RNA_def_property(srna, "blend_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "blend_mode");
  RNA_def_property_enum_items(prop, rna_enum_layer_blend_modes_items);
  RNA_def_property_ui_text(prop, "Blend Mode", "Blend mode");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* API: Get frame at. */
  FunctionRNA *func;
  PropertyRNA *parm;
  func = RNA_def_function(srna, "get_frame_at", "rna_GreasePencilLayer_get_frame_at");
  RNA_def_function_ui_description(func, "Get the frame at given frame number");
  parm = RNA_def_int(
      func, "frame_number", 1, MINAFRAME, MAXFRAME, "Frame Number", "", MINAFRAME, MAXFRAME);
  RNA_def_parameter_flags(parm, PropertyFlag(0), PARM_REQUIRED);
  parm = RNA_def_pointer(func, "frame", "GreasePencilFrame", "Frame", "");
  RNA_def_function_return(func, parm);

  /* API: Clear. */
  func = RNA_def_function(srna, "clear", "rna_GreasePencilLayer_clear");
  RNA_def_function_ui_description(func, "Remove all the Grease Pencil frames from the layer");
  RNA_def_function_flag(func, FUNC_USE_SELF_ID);
}

static void rna_def_grease_pencil_layers_api(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  PropertyRNA *prop;

  RNA_def_property_srna(cprop, "GreasePencilv3Layers");
  srna = RNA_def_struct(brna, "GreasePencilv3Layers", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencil");
  RNA_def_struct_ui_text(srna, "Grease Pencil Layers", "Collection of Grease Pencil layers");

  prop = RNA_def_property(srna, "active_layer", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilLayer");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_GreasePencil_active_layer_get",
                                 "rna_GreasePencil_active_layer_set",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Active Layer", "Active Grease Pencil layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA | NA_SELECTED, nullptr);
}

static void rna_def_grease_pencil_layer_group(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  srna = RNA_def_struct(brna, "GreasePencilLayerGroup", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencilLayerTreeGroup");
  RNA_def_struct_ui_text(srna, "Grease Pencil Layer Group", "Group of Grease Pencil layers");
  RNA_def_struct_path_func(srna, "rna_GreasePencilLayerGroup_path");

  /* Name */
  prop = RNA_def_property(srna, "name", PROP_STRING, PROP_NONE);
  RNA_def_property_ui_text(prop, "Name", "Group name");
  RNA_def_property_string_funcs(prop,
                                "rna_GreasePencilLayerGroup_name_get",
                                "rna_GreasePencilLayerGroup_name_length",
                                "rna_GreasePencilLayerGroup_name_set");
  RNA_def_struct_name_property(srna, prop);
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA | NA_RENAME, "rna_grease_pencil_update");

  /* Visibility */
  prop = RNA_def_property(srna, "hide", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE);
  RNA_def_property_ui_icon(prop, ICON_HIDE_OFF, -1);
  RNA_def_property_ui_text(prop, "Hide", "Set layer group visibility");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Lock */
  prop = RNA_def_property(srna, "lock", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_LOCKED);
  RNA_def_property_ui_icon(prop, ICON_UNLOCKED, 1);
  RNA_def_property_ui_text(
      prop, "Locked", "Protect group from further editing and/or frame changes");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Use Masks. */
  prop = RNA_def_property(srna, "use_masks", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_negative_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE_MASKS);
  RNA_def_property_ui_text(prop,
                           "Use Masks",
                           "The visibility of drawings in the layers in this group is affected by "
                           "the layers in the masks lists");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "use_onion_skinning", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_ui_icon(prop, ICON_ONIONSKIN_OFF, 1);
  RNA_def_property_boolean_negative_sdna(
      prop, "GreasePencilLayerTreeNode", "flag", GP_LAYER_TREE_NODE_HIDE_ONION_SKINNING);
  RNA_def_property_ui_text(
      prop, "Onion Skinning", "Display onion skins before and after the current frame");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");
}

static void rna_def_grease_pencil_layer_group_api(BlenderRNA *brna, PropertyRNA *cprop)
{
  StructRNA *srna;
  PropertyRNA *prop;

  RNA_def_property_srna(cprop, "GreasePencilv3LayerGroup");
  srna = RNA_def_struct(brna, "GreasePencilv3LayerGroup", nullptr);
  RNA_def_struct_sdna(srna, "GreasePencil");
  RNA_def_struct_ui_text(srna, "Grease Pencil Group", "Collection of Grease Pencil layers");

  prop = RNA_def_property(srna, "active_group", PROP_POINTER, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilLayerGroup");
  RNA_def_property_pointer_funcs(prop,
                                 "rna_GreasePencil_active_group_get",
                                 "rna_GreasePencil_active_group_set",
                                 nullptr,
                                 nullptr);
  RNA_def_property_flag(prop, PROP_EDITABLE);
  RNA_def_property_ui_text(prop, "Active Layer Group", "Active Grease Pencil layer");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA | NA_SELECTED, nullptr);
}

static void rna_def_grease_pencil_onion_skinning(StructRNA *srna)
{
  PropertyRNA *prop;

  static EnumPropertyItem prop_enum_onion_modes_items[] = {
      {GP_ONION_SKINNING_MODE_ABSOLUTE,
       "ABSOLUTE",
       0,
       "Frames",
       "Frames in absolute range of the scene frame"},
      {GP_ONION_SKINNING_MODE_RELATIVE,
       "RELATIVE",
       0,
       "Keyframes",
       "Frames in relative range of the Grease Pencil keyframes"},
      {GP_ONION_SKINNING_MODE_SELECTED, "SELECTED", 0, "Selected", "Only selected keyframes"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  static EnumPropertyItem prop_enum_onion_keyframe_type_items[] = {
      {GREASE_PENCIL_ONION_SKINNING_FILTER_ALL, "ALL", 0, "All", "Include all Keyframe types"},
      {GP_ONION_SKINNING_FILTER_KEYTYPE_KEYFRAME,
       "KEYFRAME",
       ICON_KEYTYPE_KEYFRAME_VEC,
       "Keyframe",
       "Normal keyframe, e.g. for key poses"},
      {GP_ONION_SKINNING_FILTER_KEYTYPE_BREAKDOWN,
       "BREAKDOWN",
       ICON_KEYTYPE_BREAKDOWN_VEC,
       "Breakdown",
       "A breakdown pose, e.g. for transitions between key poses"},
      {GP_ONION_SKINNING_FILTER_KEYTYPE_MOVEHOLD,
       "MOVING_HOLD",
       ICON_KEYTYPE_MOVING_HOLD_VEC,
       "Moving Hold",
       "A keyframe that is part of a moving hold"},
      {GP_ONION_SKINNING_FILTER_KEYTYPE_EXTREME,
       "EXTREME",
       ICON_KEYTYPE_EXTREME_VEC,
       "Extreme",
       "An 'extreme' pose, or some other purpose as needed"},
      {GP_ONION_SKINNING_FILTER_KEYTYPE_JITTER,
       "JITTER",
       ICON_KEYTYPE_JITTER_VEC,
       "Jitter",
       "A filler or baked keyframe for keying on ones, or some other purpose as needed"},
      {BEZT_KEYTYPE_GENERATED,
       "GENERATED",
       ICON_KEYTYPE_GENERATED_VEC,
       "Generated",
       "A key generated automatically by a tool, not manually created"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  prop = RNA_def_property(srna, "ghost_before_range", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "onion_skinning_settings.num_frames_before");
  RNA_def_property_range(prop, 0, 120);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop,
                           "Frames Before",
                           "Maximum number of frames to show before current frame "
                           "(0 = don't show any frames before current)");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "ghost_after_range", PROP_INT, PROP_NONE);
  RNA_def_property_int_sdna(prop, nullptr, "onion_skinning_settings.num_frames_after");
  RNA_def_property_range(prop, 0, 120);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop,
                           "Frames After",
                           "Maximum number of frames to show after current frame "
                           "(0 = don't show any frames after current)");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "use_ghost_custom_colors", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, nullptr, "onion_skinning_settings.flag", GP_ONION_SKINNING_USE_CUSTOM_COLORS);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Use Custom Ghost Colors", "Use custom colors for ghost frames");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "before_color", PROP_FLOAT, PROP_COLOR);
  RNA_def_property_float_sdna(prop, nullptr, "onion_skinning_settings.color_before");
  RNA_def_property_array(prop, 3);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Before Color", "Base color for ghosts before the active frame");
  RNA_def_property_update(prop,
                          NC_SCREEN | NC_SCENE | ND_TOOLSETTINGS | ND_DATA | NC_GPENCIL,
                          "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "after_color", PROP_FLOAT, PROP_COLOR);
  RNA_def_property_float_sdna(prop, nullptr, "onion_skinning_settings.color_after");
  RNA_def_property_array(prop, 3);
  RNA_def_property_range(prop, 0.0f, 1.0f);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "After Color", "Base color for ghosts after the active frame");
  RNA_def_property_update(prop,
                          NC_SCREEN | NC_SCENE | ND_TOOLSETTINGS | ND_DATA | NC_GPENCIL,
                          "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "onion_mode", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "onion_skinning_settings.mode");
  RNA_def_property_enum_items(prop, prop_enum_onion_modes_items);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Mode", "Mode to display frames");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "onion_keyframe_type", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_sdna(prop, nullptr, "onion_skinning_settings.filter");
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_enum_items(prop, prop_enum_onion_keyframe_type_items);
  RNA_def_property_ui_text(prop, "Filter by Type", "Type of keyframe (for filtering)");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "use_onion_fade", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, nullptr, "onion_skinning_settings.flag", GP_ONION_SKINNING_USE_FADE);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(
      prop, "Fade", "Display onion keyframes with a fade in color transparency");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "use_onion_loop", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(
      prop, nullptr, "onion_skinning_settings.flag", GP_ONION_SKINNING_SHOW_LOOP);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(
      prop, "Show Start Frame", "Display onion keyframes for looping animations");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  prop = RNA_def_property(srna, "onion_factor", PROP_FLOAT, PROP_NONE);
  RNA_def_property_float_sdna(prop, nullptr, "onion_skinning_settings.opacity");
  RNA_def_property_range(prop, 0.0, 1.0f);
  RNA_def_parameter_clear_flags(prop, PROP_ANIMATABLE, ParameterFlag(0));
  RNA_def_property_ui_text(prop, "Onion Opacity", "Change fade opacity of displayed onion frames");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");
}

static void rna_def_grease_pencil_data(BlenderRNA *brna)
{
  StructRNA *srna;
  PropertyRNA *prop;

  static EnumPropertyItem prop_stroke_depth_order_items[] = {
      {0, "2D", 0, "2D Layers", "Display strokes using grease pencil layers to define order"},
      {GREASE_PENCIL_STROKE_ORDER_3D,
       "3D",
       0,
       "3D Location",
       "Display strokes using real 3D position in 3D space"},
      {0, nullptr, 0, nullptr, nullptr},
  };

  srna = RNA_def_struct(brna, "GreasePencilv3", "ID");
  RNA_def_struct_sdna(srna, "GreasePencil");
  RNA_def_struct_ui_text(srna, "Grease Pencil", "Grease Pencil data-block");
  RNA_def_struct_ui_icon(srna, ICON_OUTLINER_DATA_GREASEPENCIL);

  /* attributes */
  rna_def_attributes_common(srna);

  /* Animation Data */
  rna_def_animdata_common(srna);

  /* Materials */
  prop = RNA_def_property(srna, "materials", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_collection_sdna(prop, nullptr, "material_array", "material_array_num");
  RNA_def_property_struct_type(prop, "Material");
  RNA_def_property_ui_text(prop, "Materials", "");
  RNA_def_property_srna(prop, "IDMaterials"); /* see rna_ID.cc */
  RNA_def_property_collection_funcs(prop,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    "rna_IDMaterials_assign_int");

  /* Layers */
  prop = RNA_def_property(srna, "layers", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilLayer");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_grease_pencil_layers_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_grease_pencil_layers_length",
                                    nullptr, /* TODO */
                                    nullptr, /* TODO */
                                    nullptr);
  RNA_def_property_ui_text(prop, "Layers", "Grease Pencil layers");
  rna_def_grease_pencil_layers_api(brna, prop);

  /* Layer Groups */
  prop = RNA_def_property(srna, "layer_groups", PROP_COLLECTION, PROP_NONE);
  RNA_def_property_struct_type(prop, "GreasePencilLayerGroup");
  RNA_def_property_collection_funcs(prop,
                                    "rna_iterator_grease_pencil_layer_groups_begin",
                                    "rna_iterator_array_next",
                                    "rna_iterator_array_end",
                                    "rna_iterator_array_dereference_get",
                                    "rna_iterator_grease_pencil_layer_groups_length",
                                    nullptr, /* TODO */
                                    nullptr, /* TODO */
                                    nullptr);
  RNA_def_property_ui_text(prop, "Layer Groups", "Grease Pencil layer groups");
  rna_def_grease_pencil_layer_group_api(brna, prop);

  prop = RNA_def_property(srna, "use_autolock_layers", PROP_BOOLEAN, PROP_NONE);
  RNA_def_property_boolean_sdna(prop, nullptr, "flag", GREASE_PENCIL_AUTOLOCK_LAYERS);
  RNA_def_property_ui_text(
      prop,
      "Auto-Lock Layers",
      "Automatically lock all layers except the active one to avoid accidental changes");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_autolock");

  /* Uses a single flag, because the depth order can only be 2D or 3D. */
  prop = RNA_def_property(srna, "stroke_depth_order", PROP_ENUM, PROP_NONE);
  RNA_def_property_enum_bitflag_sdna(prop, nullptr, "flag");
  RNA_def_property_enum_items(prop, prop_stroke_depth_order_items);
  RNA_def_property_ui_text(
      prop,
      "Stroke Depth Order",
      "Defines how the strokes are ordered in 3D space (for objects not displayed 'In Front')");
  RNA_def_property_update(prop, NC_GPENCIL | ND_DATA, "rna_grease_pencil_update");

  /* Onion skinning. */
  rna_def_grease_pencil_onion_skinning(srna);
}

void RNA_def_grease_pencil(BlenderRNA *brna)
{
  rna_def_grease_pencil_data(brna);
  rna_def_grease_pencil_layer(brna);
  rna_def_grease_pencil_layer_mask(brna);
  rna_def_grease_pencil_layer_group(brna);
  rna_def_grease_pencil_frame(brna);
  rna_def_grease_pencil_drawing(brna);
  rna_def_grease_pencil_drawing_stroke(brna);
}

#endif
