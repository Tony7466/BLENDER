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

struct GreasePencilUndoDrawingReference {
  GreasePencilDrawingBase base;
  UndoRefID_GreasePencil grease_pencil_ref = {};
};

struct GreasePencilStepData {
  UndoRefID_Object obedit_ref = {};
  int frame;
  char *active_layer_name;
  Array<GreasePencilDrawingBase *> drawings_at_frame;
  blender::bke::greasepencil::LayerGroup root_group;
};

struct GreasePencilUndoStep {
  UndoStep step;
  GreasePencilStepData *data;
};

static bool step_encode(bContext *C, Main *bmain, UndoStep *us_p)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);

  const Scene *scene = CTX_data_scene(C);
  Object *object = CTX_data_active_object(C);
  const GreasePencil &grease_pencil = *static_cast<GreasePencil *>(object->data);

  us->data = MEM_new<GreasePencilStepData>(__func__);
  GreasePencilStepData &step_data = *us->data;

  step_data.obedit_ref.ptr = object;
  step_data.frame = scene->r.cfra;
  step_data.active_layer_name = BLI_strdup(grease_pencil.active_layer->wrap().name().c_str());

  Span<const Layer *> layers = grease_pencil.layers();
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();
  step_data.drawings_at_frame.reinitialize(layers.size());
  for (const int i : layers.index_range()) {
    const Layer &layer = *layers[i];
    int drawing_index = layer.drawing_index_at(step_data.frame);
    if (drawing_index == -1) {
      continue;
    }

    GreasePencilDrawingBase *drawing_base = drawings[drawing_index];
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing &dst_drawing = *MEM_new<GreasePencilDrawing>(__func__);
      GreasePencilDrawing *src_drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);

      dst_drawing.base.type = src_drawing->base.type;
      dst_drawing.base.flag = src_drawing->base.flag;

      new (&dst_drawing.geometry) blender::bke::CurvesGeometry(src_drawing->geometry.wrap());
      dst_drawing.runtime = MEM_new<bke::greasepencil::DrawingRuntime>(__func__);

      step_data.drawings_at_frame[i] = reinterpret_cast<GreasePencilDrawingBase *>(&dst_drawing);
    }
    else if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilUndoDrawingReference &dst_reference = *MEM_new<GreasePencilUndoDrawingReference>(
          __func__);
      GreasePencilDrawingReference *src_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);

      dst_reference.grease_pencil_ref.ptr = src_reference->id_reference;
    }
  }

  step_data.root_group.~LayerGroup();
  new (&step_data.root_group) LayerGroup(grease_pencil.root_group.wrap());

  bmain->is_memfile_undo_flush_needed = true;

  return true;
}

static void step_decode(
    bContext *C, Main *bmain, UndoStep *us_p, const eUndoStepDir /*dir*/, bool /*is_final*/)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);
  GreasePencilStepData *step = us->data;

  ED_undo_object_editmode_restore_helper(
      C, &step->obedit_ref.ptr, 1, sizeof(GreasePencilStepData));
  BLI_assert(BKE_object_is_in_editmode(step->obedit_ref.ptr));

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(step->obedit_ref.ptr->data);
  new (&grease_pencil.root_group.wrap()) blender::bke::greasepencil::LayerGroup(step->root_group);

  grease_pencil.active_layer = reinterpret_cast<GreasePencilLayer *>(
      grease_pencil.find_layer_by_name(step->active_layer_name));

  Span<const Layer *> layers = grease_pencil.layers();
  Span<GreasePencilDrawingBase *> drawings = grease_pencil.drawings();
  for (const int i : layers.index_range()) {
    const Layer &layer = *layers[i];
    int drawing_index = layer.drawing_index_at(step->frame);
    if (drawing_index == -1) {
      continue;
    }

    GreasePencilDrawingBase *drawing_base = drawings[drawing_index];
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing &src_drawing = *reinterpret_cast<GreasePencilDrawing *>(
          step->drawings_at_frame[i]);
      GreasePencilDrawing *dst_drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);

      dst_drawing->base.type = src_drawing.base.type;
      dst_drawing->base.flag = src_drawing.base.flag;

      new (&dst_drawing->geometry) blender::bke::CurvesGeometry(src_drawing.geometry.wrap());
      dst_drawing->runtime = MEM_new<bke::greasepencil::DrawingRuntime>(__func__);
    }
    else if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilUndoDrawingReference *src_reference =
          reinterpret_cast<GreasePencilUndoDrawingReference *>(step->drawings_at_frame[i]);
      GreasePencilDrawingReference *dst_reference =
          reinterpret_cast<GreasePencilDrawingReference *>(drawing_base);

      dst_reference->id_reference = src_reference->grease_pencil_ref.ptr;
    }
  }

  DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);

  ED_undo_object_set_active_or_warn(
      CTX_data_scene(C), CTX_data_view_layer(C), step->obedit_ref.ptr, us_p->name, &LOG);

  bmain->is_memfile_undo_flush_needed = true;

  WM_event_add_notifier(C, NC_GEOM | ND_DATA, nullptr);
}

static void step_free(UndoStep *us_p)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);
  for (GreasePencilDrawingBase *drawing_base : us->data->drawings_at_frame) {
    if (drawing_base->type == GP_DRAWING) {
      GreasePencilDrawing *drawing = reinterpret_cast<GreasePencilDrawing *>(drawing_base);
      drawing->geometry.wrap().~CurvesGeometry();
      MEM_delete(drawing->runtime);
      drawing->runtime = nullptr;
      MEM_freeN(drawing);
    }
    else if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilUndoDrawingReference *drawing_reference =
          reinterpret_cast<GreasePencilUndoDrawingReference *>(drawing_base);
      MEM_freeN(drawing_reference);
    }
  }
  MEM_freeN(us->data->active_layer_name);
  MEM_delete(us->data);
}

static void foreach_ID_ref(UndoStep *us_p,
                           UndoTypeForEachIDRefFn foreach_ID_ref_fn,
                           void *user_data)
{
  using namespace blender::bke::greasepencil;
  GreasePencilUndoStep *us = reinterpret_cast<GreasePencilUndoStep *>(us_p);

  for (GreasePencilDrawingBase *drawing_base : us->data->drawings_at_frame) {
    if (drawing_base->type == GP_DRAWING_REFERENCE) {
      GreasePencilUndoDrawingReference *drawing_ref =
          reinterpret_cast<GreasePencilUndoDrawingReference *>(drawing_base);
      foreach_ID_ref_fn(user_data, ((UndoRefID *)&drawing_ref->grease_pencil_ref));
    }
  }

  foreach_ID_ref_fn(user_data, ((UndoRefID *)&us->data->obedit_ref));
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