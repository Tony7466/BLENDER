/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_hash.h"
#include "BLI_math_vector.hh"
#include "BLI_rand.hh"
#include "BLI_task.hh"

#include "BKE_context.hh"
#include "BKE_curves.hh"
#include "BKE_grease_pencil.hh"
#include "BKE_paint.hh"

#include "ED_grease_pencil.hh"
#include "ED_view3d.hh"

#include "WM_api.hh"
#include "WM_types.hh"

#include "grease_pencil_intern.hh"
#include "paint_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

/* Use a hash to generate random numbers. */
static float hash_rng(unsigned int seed1, unsigned int seed2, int index)
{
  return BLI_hash_int_01(BLI_hash_int_3d(seed1, seed2, (unsigned int)index));
}

class RandomizeOperation : public GreasePencilStrokeOperationCommon {
 public:
  /* Get a different seed value for each stroke. */
  unsigned int unique_seed() const;

  bool on_stroke_extended_drawing(const bContext &C,
                                  const bke::greasepencil::Layer &layer,
                                  bke::greasepencil::Drawing &drawing,
                                  int frame_number,
                                  const ed::greasepencil::DrawingPlacement &placement,
                                  const IndexMask &point_selection,
                                  Span<float2> view_positions,
                                  const InputSample &extension_sample) override;
};

unsigned int RandomizeOperation::unique_seed() const
{
  /* Note: GPv2 method, this does not return a different value for every invocation: the time-based
   * part remains the same for one second and the pointer isn't guaranteed to be different. */
  //   unsigned int seed = (unsigned int)(BLI_time_now_seconds_i() & UINT_MAX);
  //   seed ^= POINTER_AS_UINT(this);

  return RandomNumberGenerator::from_random_seed().get_uint32();
}

bool RandomizeOperation::on_stroke_extended_drawing(
    const bContext &C,
    const bke::greasepencil::Layer & /*layer*/,
    bke::greasepencil::Drawing &drawing,
    int /*frame_number*/,
    const ed::greasepencil::DrawingPlacement &placement,
    const IndexMask &point_selection,
    const Span<float2> view_positions,
    const InputSample &extension_sample)
{
  const Scene &scene = *CTX_data_scene(&C);
  Paint &paint = *BKE_paint_get_active_from_context(&C);
  const Brush &brush = *BKE_paint_brush(&paint);
  const int sculpt_mode_flag = brush.gpencil_settings->sculpt_mode_flag;
  const unsigned int seed = this->unique_seed();

  bke::CurvesGeometry &curves = drawing.strokes_for_write();
  bke::MutableAttributeAccessor attributes = curves.attributes_for_write();

  bool changed = false;
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_POSITION) {
    MutableSpan<float3> positions = curves.positions_for_write();

    /* Jitter is applied perpendicular to the mouse movement vector. */
    const float2 forward = math::normalize(this->mouse_delta(extension_sample));
    const float2 sideways = float2(-forward.y, forward.x);

    point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
      const float2 &co = view_positions[point_i];
      const float influence = brush_influence(scene, brush, co, extension_sample);
      if (influence <= 0.0f) {
        return;
      }
      const float noise = 2.0f * hash_rng(seed, 5678, point_i) - 1.0f;
      positions[point_i] = placement.project(co + sideways * influence * noise);
    });

    drawing.tag_positions_changed();
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_STRENGTH) {
    MutableSpan<float> opacities = drawing.opacities_for_write();
    point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
      const float2 &co = view_positions[point_i];
      const float influence = brush_influence(scene, brush, co, extension_sample);
      if (influence <= 0.0f) {
        return;
      }
      const float noise = 2.0f * hash_rng(seed, 1212, point_i) - 1.0f;
      opacities[point_i] = math::clamp(opacities[point_i] + influence * noise, 0.0f, 1.0f);
    });
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_THICKNESS) {
    const MutableSpan<float> radii = drawing.radii_for_write();
    point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
      const float2 &co = view_positions[point_i];
      const float influence = brush_influence(scene, brush, co, extension_sample);
      if (influence <= 0.0f) {
        return;
      }
      const float noise = 2.0f * hash_rng(seed, 1212, point_i) - 1.0f;
      radii[point_i] = math::max(radii[point_i] + influence * noise * 0.001f, 0.0f);
    });
    curves.tag_radii_changed();
    changed = true;
  }
  if (sculpt_mode_flag & GP_SCULPT_FLAGMODE_APPLY_UV) {
    /* TODO stroke_u attribute not used yet. */
    bke::SpanAttributeWriter<float> rotations = attributes.lookup_or_add_for_write_span<float>(
        "rotation", bke::AttrDomain::Point);
    point_selection.foreach_index(GrainSize(4096), [&](const int64_t point_i) {
      const float2 &co = view_positions[point_i];
      const float influence = brush_influence(scene, brush, co, extension_sample);
      if (influence <= 0.0f) {
        return;
      }
      const float noise = 2.0f * hash_rng(seed, 1212, point_i) - 1.0f;
      rotations.span[point_i] = math::clamp(
          rotations.span[point_i] + influence * noise, -float(M_PI_2), float(M_PI_2));
    });
    rotations.finish();
    changed = true;
  }
  return changed;
}

std::unique_ptr<GreasePencilStrokeOperation> new_randomize_operation()
{
  return std::make_unique<RandomizeOperation>();
}

}  // namespace blender::ed::sculpt_paint::greasepencil
