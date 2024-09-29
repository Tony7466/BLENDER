/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_main.hh"
#include "DNA_scene_types.h"
#include "RNA_access.hh"
#include "RNA_types.hh"

#include "IO_unit_type.hh"

const EnumPropertyItem io_unit_type[] = {
    {IO_UNIT_TYPE_METER,
     "Meter",
     0,
     "Meter",
     "Meter is used by blender and thus will persist the blender scale"},
    {IO_UNIT_TYPE_DECIMETER, "Decimeter", 0, "Decimeter", ""},
    {IO_UNIT_TYPE_CENTIMETER, "Centimeter", 0, "Centimeter", ""},
    {IO_UNIT_TYPE_MILLIMIETER, "Millimeter", 0, "Millimeter", ""},
    {IO_UNIT_TYPE_INCH, "Inch", 0, "Inch", ""},
    {IO_UNIT_TYPE_SCENE_LENGTH,
     "Scene Unit Length",
     0,
     "Scene Unit Length",
     "Use the scene unit, as defined in the scene settings by taking scene scale and scene unit "
     "length into account."},
    {IO_UNIT_TYPE_CUSTOM, "CUSTOM", 0, "CUSTOM", "Use the scale factor provided below"},
    {0, nullptr, 0, nullptr, nullptr}};

static float lookup_factor_meter_to_scene_unit(Scene *scene)
{
  /* Returns the factor used to convert the scene unit length to meter */

  char scene_unit = scene->unit.length_unit;
  if (scene->unit.system == USER_UNIT_METRIC) {
    switch (scene_unit) {
      case '\0':  // kilometer
        return 1000.0;
      case '\x3':  // meter
        return 1.0;
      case '\x5':  // centimeter
        return 0.01;
      case '\x6':  // millimeter
        return 0.001;
      case '\a':  // micrometer
        return 1e-6;
    }
  }
  else if (scene->unit.system == USER_UNIT_IMPERIAL) {
    switch (scene_unit) {
      case '\x2':  // mile
        return 1609.34;
      case '\x4':  // feet
        return 0.3047992424196;
      case '\x5':  // inch
        return 0.025399936868299999304;
      case '\x6':  // thou
        return 2.54e-5;
    }
  }
  return 1.0;
}

float get_scene_unit_scale_factor(bool asExport, Scene *scene)
{
  /* Returns the scale factor needed to convert from the set scene unit to meters */
  float scene_unit_scale = scene->unit.scale_length;
  scene_unit_scale = lookup_factor_meter_to_scene_unit(scene) / scene_unit_scale;
  return asExport ? (1.0 / scene_unit_scale) : (scene_unit_scale);
}

static void calculate_io_scale_factor(bool asExport, Scene *scene, PointerRNA *ptr)
{
  /* Calulates the scale factor for io depending on file unit selection, scene unit type and scale
   */

  int unit = RNA_enum_get(ptr, "file_unit_type");

  if (unit == IO_UNIT_TYPE_CUSTOM)
    return;

  // determine the io scale factor
  float scale_factor = 1.0;
  if (unit == IO_UNIT_TYPE_SCENE_LENGTH) {
    // using scene unit
    scale_factor = get_scene_unit_scale_factor(asExport, scene);
  }
  else {
    // using file unit
    scale_factor = asExport ? (scale_factor / io_units_factor[unit].factor) :
                              (scale_factor * io_units_factor[unit].factor);
  }

  RNA_float_set(ptr, "global_scale", scale_factor);
}

void io_ui_unit_type_export_update(Main * /*main*/, Scene *scene, PointerRNA *ptr)
{
  /* Set the selected scale factor value to scale input field. */

  calculate_io_scale_factor(true, scene, ptr);
}

void io_ui_unit_type_import_update(Main * /*main*/, Scene *scene, PointerRNA *ptr)
{
  /* Set the selected scale factor value to scale input field. */

  calculate_io_scale_factor(false, scene, ptr);
}
