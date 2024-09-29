/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_main.hh"
#include "DNA_scene_types.h"
#include "RNA_access.hh"
#include "RNA_types.hh"

#include "IO_unit_type.hh"

const EnumPropertyItem io_unit_type[] = {
    {IO_UNIT_TYPE_METER, "Meter", 0, "Meter", ""},
    {IO_UNIT_TYPE_DECIMETER, "Decimeter", 0, "Decimeter", ""},
    {IO_UNIT_TYPE_CENTIMETER, "Centimeter", 0, "Centimeter", ""},
    {IO_UNIT_TYPE_MILLIMIETER,
     "Millimeter",
     0,
     "Millimeter",
     "The assumed unit of STL files by most programs"},
    {IO_UNIT_TYPE_INCH, "Inch", 0, "Inch", ""},
    {IO_UNIT_TYPE_CUSTOM, "CUSTOM", 0, "CUSTOM", "Use the scale factor provided below"},
    {0, nullptr, 0, nullptr, nullptr}};

void io_ui_unit_type_export_update(Main * /*main*/, Scene * /*scene*/, PointerRNA *ptr)
{
  /* Set the selected scale factor value to scale input field. */

  int unit = RNA_enum_get(ptr, "file_unit_type");
  if (unit == IO_UNIT_TYPE_CUSTOM)
    return;
  RNA_float_set(ptr, "global_scale", io_units_factor_export[unit].factor);
}

void io_ui_unit_type_import_update(Main * /*main*/, Scene * /*scene*/, PointerRNA *ptr)
{
  /* Set the selected scale factor value to scale input field. */

  int unit = RNA_enum_get(ptr, "file_unit_type");
  if (unit == IO_UNIT_TYPE_CUSTOM)
    return;
  RNA_float_set(ptr, "global_scale", io_units_factor_import[unit].factor);
}
