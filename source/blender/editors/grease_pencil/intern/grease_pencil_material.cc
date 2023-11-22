/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "DNA_material_types.h"

#include "BKE_context.hh"
#include "BKE_curves_utils.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_material.h"

#include "DEG_depsgraph.hh"

#include "ED_grease_pencil.hh"

#include "WM_api.hh"

namespace blender::ed::greasepencil {

/* -------------------------------------------------------------------- */
/** \name Show All Materials Operator
 * \{ */

static int grease_pencil_material_reveal_exec(bContext *C, wmOperator * /*op*/)
{
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  bool changed = false;
  for (const int i : IndexRange(object->totcol)) {
    if (Material *ma = BKE_gpencil_material(object, i + 1)) {
      MaterialGPencilStyle &gp_style = *ma->gp_style;
      gp_style.flag &= ~GP_MATERIAL_HIDE;
      DEG_id_tag_update(&ma->id, ID_RECALC_COPY_ON_WRITE);
      changed = true;
    }
  }

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA | NA_EDITED, &grease_pencil);
  }

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_material_reveal(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Show All Materials";
  ot->idname = "GREASE_PENCIL_OT_material_reveal";
  ot->description = "Unhide all hidden Grease Pencil materials";

  /* Callbacks. */
  ot->exec = grease_pencil_material_reveal_exec;
  ot->poll = active_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lock Unselected Materials Operator
 * \{ */

static int grease_pencil_material_lock_unselected_exec(bContext *C, wmOperator * /*op*/)
{
  using namespace blender;
  using namespace blender::bke;

  const Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  bool changed = false;
  const Array<MutableDrawingInfo> drawings = retrieve_editable_drawings(*scene, grease_pencil);
  threading::parallel_for_each(drawings, [&](const MutableDrawingInfo &info) {
    IndexMaskMemory memory;
    const IndexMask strokes = ed::greasepencil::retrieve_editable_and_selected_strokes(
        *object, info.drawing, memory);
    if (strokes.is_empty()) {
      return;
    }

    AttributeAccessor attributes = info.drawing.strokes().attributes();
    const VArraySpan<int> material_indices = *attributes.lookup_or_default<int>(
        "material_index", ATTR_DOMAIN_CURVE, 0);

    for (const int material_index : IndexRange(object->totcol)) {
      bool found = false;
      strokes.foreach_index([&](const int64_t curve_i) {
        if (material_indices[curve_i] == material_index) {
          found = true;
        }
      });

      if (!found) {
        if (Material *ma = BKE_object_material_get(object, material_index + 1)) {
          MaterialGPencilStyle &gp_style = *ma->gp_style;
          gp_style.flag |= GP_MATERIAL_LOCKED;
          DEG_id_tag_update(&ma->id, ID_RECALC_COPY_ON_WRITE);
          changed = true;
        }
      }
    }
  });

  if (changed) {
    DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
    WM_event_add_notifier(C, NC_GEOM | ND_DATA | NA_EDITED, &grease_pencil);
  }

  return OPERATOR_FINISHED;
}

static void GREASE_PENCIL_OT_material_lock_unselected(wmOperatorType *ot)
{
  /* Identifiers. */
  ot->name = "Lock Unselected Materials";
  ot->idname = "GREASE_PENCIL_OT_material_lock_unselected";
  ot->description = "Lock any material not used in any selected stroke";

  /* Callbacks. */
  ot->exec = grease_pencil_material_lock_unselected_exec;
  ot->poll = active_grease_pencil_poll;

  ot->flag = OPTYPE_REGISTER | OPTYPE_UNDO;
}

/** \} */

}  // namespace blender::ed::greasepencil

void ED_operatortypes_grease_pencil_material()
{
  using namespace blender::ed::greasepencil;
  WM_operatortype_append(GREASE_PENCIL_OT_material_reveal);
  WM_operatortype_append(GREASE_PENCIL_OT_material_lock_unselected);
}
