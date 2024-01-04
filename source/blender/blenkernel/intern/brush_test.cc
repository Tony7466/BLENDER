/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "DNA_brush_types.h"
#include "DNA_scene_types.h"

#include "BKE_brush.hh"

namespace blender::bke::tests {
TEST(brush, BKE_brush_input_samples_get_unified)
{
  ToolSettings ts;
  ts.unified_paint_settings.input_samples = 2;
  ts.unified_paint_settings.flag = UNIFIED_PAINT_INPUT_SAMPLES;

  Scene s;
  s.toolsettings = &ts;

  Brush b;
  b.input_samples = 3;

  int retVal = BKE_brush_input_samples_get(&s, &b);

  EXPECT_EQ(2, retVal);
}
TEST(brush, BKE_brush_input_samples_get_individual)
{
  ToolSettings ts;
  ts.unified_paint_settings.input_samples = 2;
  ts.unified_paint_settings.flag = 0;

  Scene s;
  s.toolsettings = &ts;

  Brush b;
  b.input_samples = 3;

  int retVal = BKE_brush_input_samples_get(&s, &b);

  EXPECT_EQ(3, retVal);
}
TEST(brush, BKE_brush_input_samples_set_unified)
{
  ToolSettings ts;
  ts.unified_paint_settings.input_samples = 64;
  ts.unified_paint_settings.flag = UNIFIED_PAINT_INPUT_SAMPLES;

  Scene s;
  s.toolsettings = &ts;

  Brush b;
  b.input_samples = 64;

  BKE_brush_input_samples_set(&s, &b, 1);

  EXPECT_EQ(1, ts.unified_paint_settings.input_samples);
  EXPECT_EQ(64, b.input_samples);
}
TEST(brush, BKE_brush_input_samples_set_individual)
{
  ToolSettings ts;
  ts.unified_paint_settings.input_samples = 64;
  ts.unified_paint_settings.flag = 0;

  Scene s;
  s.toolsettings = &ts;

  Brush b;
  b.input_samples = 64;

  BKE_brush_input_samples_set(&s, &b, 1);

  EXPECT_EQ(0, ts.unified_paint_settings.flag & UNIFIED_PAINT_INPUT_SAMPLES);
  EXPECT_EQ(64, ts.unified_paint_settings.input_samples);
  EXPECT_EQ(1, b.input_samples);
}
}  // namespace blender::bke::tests
