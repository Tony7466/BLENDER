/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_array.hh"
#include "BLI_array_utils.hh"
#include "BLI_index_mask.hh"
#include "BLI_kdtree.h"
#include "BLI_math_geom.h"
#include "BLI_task.hh"

#include "BKE_attribute.hh"
#include "BKE_brush.hh"
#include "BKE_colortools.hh"
#include "BKE_context.hh"
#include "BKE_crazyspace.hh"
#include "BKE_curves.hh"
#include "BKE_curves_utils.hh"
#include "BKE_grease_pencil.h"
#include "BKE_grease_pencil.hh"
#include "BKE_scene.h"

#include "DEG_depsgraph_query.hh"
#include "DNA_brush_enums.h"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

static constexpr int POINT_CACHE_CHUNK = 1024;
static constexpr float FIND_NEAREST_POINT_EPSILON = 1e-6f;

class WeightPaintOperation : public GreasePencilStrokeOperation {

 public:
  ~WeightPaintOperation() override {}

  void on_stroke_begin(const bContext &C, const InputSample &start_sample) override;
  void on_stroke_extended(const bContext &C, const InputSample &extension_sample) override;
  void on_stroke_done(const bContext &C) override;

  float radius = 50.0f;
  Brush *brush;
  float2 mouse_position;
  float2 mouse_position_previous;
  /* Brush direction during a stroke. */
  float2 brush_direction;
  bool brush_direction_is_set = false;
  Array<Array<ed::greasepencil::MutableDrawingInfo>> drawings_per_frame;
};

/**
 * Utility class that actually executes the update when the stroke is updated. That's useful
 * because it avoids passing a very large number of parameters between functions.
 */
struct WeightPaintOperationExecutor {

  float brush_radius{};

  int2 mouse_position_pixels{};
  int64_t brush_radius_squared_pixels{};

  /* Tool direction: add or subtract weight to vertices. */
  bool subtract = false;

  /* Auto-normalize weights of bone-deformed vertices? */
  bool auto_normalize = false;

  WeightPaintOperationExecutor(const bContext & /*C*/) {}

  void execute(WeightPaintOperation &self, const bContext &C, const InputSample &extension_sample)
  {
    using namespace blender::bke::greasepencil;
    Scene *scene = CTX_data_scene(&C);
    Depsgraph *depsgraph = CTX_data_depsgraph_pointer(&C);
    ARegion *region = CTX_wm_region(&C);
    Object *obact = CTX_data_active_object(&C);
    Object *ob_eval = DEG_get_evaluated_object(depsgraph, obact);

    /* Get the tool's data. */
    self.mouse_position = extension_sample.mouse_position;
    this->brush_radius = self.radius;
    if (BKE_brush_use_size_pressure(self.brush)) {
      this->brush_radius *= extension_sample.pressure;
    }

    this->mouse_position_pixels = int2(round_fl_to_int(self.mouse_position[0]),
                                       round_fl_to_int(self.mouse_position[1]));
    const int64_t brush_radius_pixels = round_fl_to_int(brush_radius);
    this->brush_radius_squared_pixels = brush_radius_pixels * brush_radius_pixels;

    /* Get the grease pencil drawing. */
    GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);

    bool changed = false;

    if (changed) {
      DEG_id_tag_update(&grease_pencil.id, ID_RECALC_GEOMETRY);
      WM_event_add_notifier(&C, NC_GEOM | ND_DATA, &grease_pencil);
    }
  }
};

void WeightPaintOperation::on_stroke_begin(const bContext &C, const InputSample & /*start_sample*/)
{
  const Scene *scene = CTX_data_scene(&C);
  const Object *obact = CTX_data_active_object(&C);
  Paint *paint = BKE_paint_get_active_from_context(&C);
  Brush *brush = BKE_paint_brush(paint);

  this->brush = brush;
  this->radius = BKE_brush_size_get(scene, brush);

  GreasePencil &grease_pencil = *static_cast<GreasePencil *>(obact->data);
  this->drawings_per_frame = ed::greasepencil::retrieve_editable_drawings_per_frame(*scene,
                                                                                    grease_pencil);
}

void WeightPaintOperation::on_stroke_extended(const bContext &C,
                                              const InputSample &extension_sample)
{
  WeightPaintOperationExecutor executor{C};
  executor.execute(*this, C, extension_sample);
}

void WeightPaintOperation::on_stroke_done(const bContext & /*C*/) {}

std::unique_ptr<GreasePencilStrokeOperation> new_weight_paint_operation()
{
  return std::make_unique<WeightPaintOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
