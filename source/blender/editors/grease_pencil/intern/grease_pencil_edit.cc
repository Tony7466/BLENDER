/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_span.hh"

#include "BKE_context.h"
#include "BKE_grease_pencil.hh"

#include "DEG_depsgraph.h"

#include "ED_curves.h"
#include "ED_grease_pencil.h"
#include "ED_screen.h"

#include "WM_api.h"

namespace blender::ed::greasepencil {

bool active_grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  return true;
}

bool editable_grease_pencil_poll(bContext *C)
{
  Object *object = CTX_data_active_object(C);
  if (object == nullptr || object->type != OB_GREASE_PENCIL) {
    return false;
  }
  if (!ED_operator_object_active_editable_ex(C, object)) {
    return false;
  }
  if ((object->mode & OB_MODE_EDIT) == 0) {
    return false;
  }
  return true;
}

bool editable_grease_pencil_point_selection_poll(bContext *C)
{
  if (!editable_grease_pencil_poll(C)) {
    return false;
  }

  /* Allowed: point and segment selection mode, not allowed: stroke selection mode. */
  ToolSettings *ts = CTX_data_tool_settings(C);
  return (ts->gpencil_selectmode_edit != GP_SELECTMODE_STROKE);
}

bool grease_pencil_painting_poll(bContext *C)
{
  if (!active_grease_pencil_poll(C)) {
    return false;
  }
  Object *object = CTX_data_active_object(C);
  if ((object->mode & OB_MODE_PAINT_GREASE_PENCIL) == 0) {
    return false;
  }
  ToolSettings *ts = CTX_data_tool_settings(C);
  if (!ts || !ts->gp_paint) {
    return false;
  }
  return true;
}

static void keymap_grease_pencil_editing(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(keyconf, "Grease Pencil Edit Mode", 0, 0);
  keymap->poll = editable_grease_pencil_poll;
}

static void keymap_grease_pencil_painting(wmKeyConfig *keyconf)
{
  wmKeyMap *keymap = WM_keymap_ensure(keyconf, "Grease Pencil Paint Mode", 0, 0);
  keymap->poll = grease_pencil_painting_poll;
}

static int grease_pencil_stroke_smooth_exec(bContext *C, wmOperator * /*op*/)
{
  using namespace blender;
  Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  /* TODO : these variables should be operator's properties */
  const int iterations = 1;
  const bool keep_shape = true;

  grease_pencil.foreach_editable_drawing(
      scene->r.cfra, [&](int /*drawing_index*/, bke::greasepencil::Drawing &drawing) {
        /* Smooth all selected curves in the current drawing */

        /* Curves geometry and attributes*/
        bke::CurvesGeometry &curves = drawing.strokes_for_write();
        Array<float3> curves_positions_copy(curves.positions());
        bke::AttributeAccessor curves_attributes = curves.attributes();

        const offset_indices::OffsetIndices<int> points_by_curve = curves.points_by_curve();
        const VArray<bool> cyclic = curves.cyclic();

        /* Selection-based mask */
        IndexMaskMemory memory;
        bke::AttributeReader<bool> selection_attribute = curves_attributes.lookup_or_default<bool>(
            ".selection", ATTR_DOMAIN_POINT, true);
        const IndexMask mask = IndexMask::from_bools(selection_attribute.varray, memory);

        threading::parallel_for(curves.curves_range(), 256, [&](const IndexRange range) {
          for (const int curve_i : range) {
            /* Smooth a single curve*/

            const IndexRange points = points_by_curve[curve_i];
            const bool is_cyclic = cyclic[curve_i];

            bke::curves::poly::gaussian_blur(is_cyclic,
                                             points,
                                             iterations,
                                             keep_shape,
                                             curves.positions_for_write(),
                                             curves_positions_copy,
                                             mask);
          }
        });
      });

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
  WM_event_add_notifier(C, NC_GEOM | ND_DATA, &grease_pencil);

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_stroke_smooth(wmOperatorType *ot)
{
  /* identifiers */
  ot->name = "Smooth Stroke";
  ot->idname = "GREASE_PENCIL_OT_stroke_smooth";
  ot->description = "Smooth selected strokes";

  /* callbacks */
  ot->exec = grease_pencil_stroke_smooth_exec;
  ot->poll = editable_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;

  /* TODO : add operator properties */
}

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_edit(void)
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_stroke_smooth);
}

void ED_keymap_grease_pencil(wmKeyConfig *keyconf)
{
  using namespace blender::ed::greasepencil;
  keymap_grease_pencil_editing(keyconf);
  keymap_grease_pencil_painting(keyconf);
}
