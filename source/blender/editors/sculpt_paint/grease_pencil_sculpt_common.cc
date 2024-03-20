/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_brush.hh"
#include "BKE_colortools.hh"

#include "DNA_brush_enums.h"
#include "DNA_brush_types.h"

#include "grease_pencil_intern.hh"

namespace blender::ed::sculpt_paint::greasepencil {

void init_brush(Brush &brush)
{
  if (brush.gpencil_settings == nullptr) {
    BKE_brush_init_gpencil_settings(&brush);
  }
  BLI_assert(brush.gpencil_settings != nullptr);
  BKE_curvemapping_init(brush.gpencil_settings->curve_strength);
  BKE_curvemapping_init(brush.gpencil_settings->curve_sensitivity);
  BKE_curvemapping_init(brush.gpencil_settings->curve_jitter);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_pressure);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_strength);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_uv);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_hue);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_saturation);
  BKE_curvemapping_init(brush.gpencil_settings->curve_rand_value);
}

static float radius_from_input_sample(const Scene &scene,
                                      const Brush &brush,
                                      const InputSample &sample)
{
  float radius = BKE_brush_size_get(&scene, &brush);
  if (BKE_brush_use_size_pressure(&brush)) {
    radius *= BKE_curvemapping_evaluateF(
        brush.gpencil_settings->curve_sensitivity, 0, sample.pressure);
  }
  return radius;
}

float brush_influence(const Scene &scene,
                      const Brush &brush,
                      const int2 &co,
                      const InputSample &sample,
                      const float multi_frame_falloff)
{
  const float radius = radius_from_input_sample(scene, brush, sample);
  /* Basic strength factor from brush settings. */
  const bool use_pressure = (brush.gpencil_settings->flag & GP_BRUSH_USE_PRESSURE);
  const float influence_base = brush.alpha * multi_frame_falloff *
                               (use_pressure ? sample.pressure : 1.0f);

  /* Distance falloff. */
  int2 mval_i;
  round_v2i_v2fl(mval_i, sample.mouse_position);
  const float distance = float(len_v2v2_int(mval_i, co));
  /* Apply Brush curve. */
  const float brush_falloff = BKE_brush_curve_strength(&brush, distance, radius);

  return influence_base * brush_falloff;
}

}  // namespace blender::ed::sculpt_paint::greasepencil
