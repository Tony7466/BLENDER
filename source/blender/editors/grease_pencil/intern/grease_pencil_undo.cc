/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup edgreasepencil
 */

#include "BKE_context.h"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_main.h"
#include "BKE_object.h"
#include "BKE_undo_system.h"

#include "CLG_log.h"

#include "DEG_depsgraph.h"

#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "ED_grease_pencil.h"
#include "ED_undo.h"

#include "WM_api.h"

static CLG_LogRef LOG = {"ed.undo.grease_pencil"};

namespace blender::ed::greasepencil::undo {

/* -------------------------------------------------------------------- */
/** \name Implements ED Undo System
 *
 * \note This is similar for all edit-mode types.
 * \{ */

struct GreasePencilStepData {
  int frame;
  char *active_layer_name;
  Array<GreasePencilDrawing> drawings_at_frame;
  blender::bke::greasepencil::LayerGroup root_group;
};

struct GreasePencilUndoStep {
  UndoStep step;
  UndoRefID_Object obedit_ref = {};
  GreasePencilStepData *data;
};

static bool step_encode(bContext *C, Main *bmain, UndoStep *us_p)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);

  const Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  const GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  us->obedit_ref.ptr = object;

  us->data = MEM_new<GreasePencilStepData>(__func__);
  GreasePencilStepData &step_data = *us->data;
  step_data.frame = scene->r.cfra;
  step_data.active_layer_name = BLI_strdup(grease_pencil.active_layer->wrap().name().c_str());

  Span<const Layer *> layers = grease_pencil.layers();
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();
  step_data.drawings_at_frame.reinitialize(layers.size());
  for (const int i : layers.index_range()) {
    GreasePencilDrawing &drawing = step_data.drawings_at_frame[i];

    const Layer &layer = *layers[i];
    int drawing_index = layer.drawing_index_at(step_data.frame);
    if (drawing_index == -1) {
      continue;
    }

    GreasePencilDrawingBase *drawing_base = drawings[drawing_index];
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing *src_drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);

      drawing.base.type = src_drawing->base.type;
      drawing.base.flag = src_drawing->base.flag;

      new (&drawing.geometry) blender::bke::CurvesGeometry(src_drawing->geometry.wrap());
      drawing.runtime = MEM_new<bke::greasepencil::DrawingRuntime>(__func__);
    }
    else if (drawing_base->type == GP_DRAWING_REFERENCE) {
      /* TODO */
    }
  }

  new (&step_data.root_group) LayerGroup(grease_pencil.root_group.wrap());

  bmain->is_memfile_undo_flush_needed = true;

  return true;
}

static void step_decode(
    bContext *C, Main *bmain, UndoStep *us_p, const eUndoStepDir /*dir*/, bool /*is_final*/)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);

  ED_undo_object_editmode_restore_helper(C, &us->obedit_ref.ptr, 1, sizeof(GreasePencilStepData));
  BLI_assert(BKE_object_is_in_editmode(us->obedit_ref.ptr));

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(us->obedit_ref.ptr->data);
  new (&grease_pencil.root_group.wrap())
      blender::bke::greasepencil::LayerGroup(us->data->root_group);

  grease_pencil.active_layer = reinterpret_cast<GreasePencilLayer *>(
      grease_pencil.find_layer_by_name(us->data->active_layer_name));

  Span<const Layer *> layers = grease_pencil.layers();
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();
  for (const int i : layers.index_range()) {
    GreasePencilDrawing &src_drawing = us->data->drawings_at_frame[i];
    const Layer &layer = *layers[i];
    int drawing_index = layer.drawing_index_at(us->data->frame);
    if (drawing_index == -1) {
      continue;
    }

    GreasePencilDrawingBase *drawing_base = drawings[drawing_index];
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing *dst_drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);

      dst_drawing->base.type = src_drawing.base.type;
      dst_drawing->base.flag = src_drawing.base.flag;

      new (&dst_drawing->geometry) blender::bke::CurvesGeometry(src_drawing.geometry.wrap());
      dst_drawing->runtime = MEM_new<bke::greasepencil::DrawingRuntime>(__func__);
    }
    else if (drawing_base->type == GP_DRAWING_REFERENCE) {
      /* TODO */
    }
  }

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);

  ED_undo_object_set_active_or_warn(
      CTX_data_scene(C), CTX_data_view_layer(C), us->obedit_ref.ptr, us_p->name, &LOG);

  bmain->is_memfile_undo_flush_needed = true;

  WM_event_add_notifier(C, NC_GEOM | ND_DATA, nullptr);
}

static void step_free(UndoStep *us_p)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);
  for (int i = 0; i < us->data->drawings_at_frame.size(); i++) {
    GreasePencilDrawing *drawing = &us->data->drawings_at_frame[i];
    drawing->geometry.wrap().~CurvesGeometry();
    MEM_delete(drawing->runtime);
    drawing->runtime = nullptr;
  }
  MEM_freeN(us->data->active_layer_name);
  MEM_delete(us->data);
}

static void foreach_ID_ref(UndoStep *us_p,
                           UndoTypeForEachIDRefFn foreach_ID_ref_fn,
                           void *user_data)
{
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);
  foreach_ID_ref_fn(user_data, ((UndoRefID *)&us->obedit_ref));
}

/** \} */

}  // namespace blender::ed::greasepencil::undo

void ED_grease_pencil_undosys_type(UndoType *ut)
{
  using namespace blender::ed;

  ut->name = "Edit Grease Pencil";
  ut->poll = greasepencil::editable_grease_pencil_poll;
  ut->step_encode = greasepencil::undo::step_encode;
  ut->step_decode = greasepencil::undo::step_decode;
  ut->step_free = greasepencil::undo::step_free;

  ut->step_foreach_ID_ref = greasepencil::undo::foreach_ID_ref;

  ut->flags = UNDOTYPE_FLAG_NEED_CONTEXT_FOR_ENCODE;

  ut->step_size = sizeof(greasepencil::undo::GreasePencilUndoStep);
}