/* SPDX-FileCopyrightText: 2011 by Bastien Montagne. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup modifiers
 */

#include "MOD_grease_pencil_util.hh"

#include "BLI_set.hh"

#include "DNA_grease_pencil_types.h"
#include "DNA_material_types.h"
#include "DNA_modifier_types.h"

#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "DEG_depsgraph_query.hh"

namespace blender::greasepencil {

using bke::greasepencil::Drawing;
using bke::greasepencil::Layer;

/**
 * Get a list of pass IDs used by grease pencil materials.
 * This way the material pass can be looked up by index instead of having to get the material for
 * each curve.
 */
static Vector<int> get_grease_pencil_material_passes(const Object *ob)
{
  short *totcol = BKE_object_material_len_p(const_cast<Object *>(ob));
  Vector<int> result(*totcol);
  Material *ma = nullptr;
  for (short i = 0; i < *totcol; i++) {
    ma = BKE_object_material_get(const_cast<Object *>(ob), i + 1);
    /* Pass index of the grease pencil material. */
    result[i] = ma->gp_style->index;
  }
  return result;
}

static IndexMask get_filtered_layer_mask(const GreasePencil &grease_pencil,
                                         const std::optional<StringRef> layer_name_filter,
                                         const std::optional<int> layer_pass_filter,
                                         const bool layer_filter_invert,
                                         const bool layer_pass_filter_invert,
                                         IndexMaskMemory &memory)
{
  bke::AttributeAccessor layer_attributes = grease_pencil.attributes();
  const Span<const Layer *> layers = grease_pencil.layers();
  const VArray<int> layer_passes =
      layer_attributes.lookup_or_default<int>("pass", bke::AttrDomain::Layer, 0).varray;

  IndexMask result = IndexMask::from_predicate(
      grease_pencil.layers().index_range(), GrainSize(4096), memory, [&](const int64_t layer_i) {
        if (layer_name_filter) {
          const Layer &layer = *layers[layer_i];
          const bool match = (layer.name() == layer_name_filter.value());
          if (match ^ layer_filter_invert) {
            return false;
          }
        }
        if (layer_pass_filter) {
          const int layer_pass = layer_passes.get(layer_i);
          const bool match = (layer_pass == layer_pass_filter.value());
          if (match ^ layer_pass_filter_invert) {
            return false;
          }
        }
        return true;
      });
  return result;
}

IndexMask get_filtered_layer_mask(const GreasePencil &grease_pencil,
                                  const GreasePencilModifierFilterData &filter_data,
                                  IndexMaskMemory &memory)
{
  /* TODO Add an option to toggle pass filter on and off, instead of using "pass > 0". */
  return get_filtered_layer_mask(
      grease_pencil,
      filter_data.layername[0] != '\0' ? std::make_optional<StringRef>(filter_data.layername) :
                                         std::nullopt,
      filter_data.layer_pass > 0 ? std::make_optional<int>(filter_data.layer_pass) : std::nullopt,
      filter_data.flag & GREASE_PENCIL_FILTER_INVERT_LAYER,
      filter_data.flag & GREASE_PENCIL_FILTER_INVERT_LAYER_PASS,
      memory);
}

static Vector<bool> is_material_affected_by_modifier(const Object *ob,
                                                     const bke::CurvesGeometry &curves,
                                                     const Material *material_filter,
                                                     const std::optional<int> material_pass_filter,
                                                     const bool material_filter_invert,
                                                     const bool material_pass_filter_invert)
{
  bke::AttributeAccessor attributes = curves.attributes();
  VArray<int> stroke_materials =
      attributes.lookup_or_default<int>("material_index", bke::AttrDomain::Curve, 0).varray;

  Vector<bool> result(curves.curves_num(), true);
  if (material_filter != nullptr) {
    const int material_filter_index = BKE_grease_pencil_object_material_index_get(
        const_cast<Object *>(ob), const_cast<Material *>(material_filter));
    for (const int stroke_i : result.index_range()) {
      const int material_index = stroke_materials.get(stroke_i);
      const bool match = (material_index == material_filter_index);
      if (match ^ material_filter_invert) {
        result[stroke_i] = false;
      }
    }
  }
  if (material_pass_filter) {
    const Vector<int> material_pass_by_index = get_grease_pencil_material_passes(ob);
    for (const int stroke_i : result.index_range()) {
      const int material_index = stroke_materials.get(stroke_i);
      const int material_pass = material_pass_by_index[material_index];
      const bool match = (material_pass == material_pass_filter.value());
      if (match ^ material_pass_filter_invert) {
        result[stroke_i] = false;
      }
    }
  }

  return result;
}

Vector<bke::greasepencil::Drawing *> get_drawings_for_write(GreasePencil &grease_pencil,
                                                            const IndexMask &layer_mask,
                                                            int frame)
{
  /* Set of unique drawing indices. */
  Set<int> drawing_indices;
  for (const int64_t i : layer_mask.index_range()) {
    Layer *layer = grease_pencil.layers_for_write()[layer_mask[i]];
    const int drawing_index = layer->drawing_index_at(frame);
    if (drawing_index >= 0) {
      drawing_indices.add(drawing_index);
    }
  }

  /* List of owned drawings, ignore drawing references to other data blocks. */
  Vector<bke::greasepencil::Drawing *> drawings;
  for (const int drawing_index : drawing_indices) {
    GreasePencilDrawingBase *drawing_base = grease_pencil.drawing(drawing_index);
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
      drawings.append(&drawing->wrap());
    }
  }
  return drawings;
}

}  // namespace blender::greasepencil
