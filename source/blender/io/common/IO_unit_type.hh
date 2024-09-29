/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

struct EnumPropertyItem;
struct Main;
struct PointerRNA;
struct Scene;

enum UnitType {
	IO_UNIT_TYPE_METER = 0,
	IO_UNIT_TYPE_DECIMETER = 1,
	IO_UNIT_TYPE_CENTIMETER = 2,
	IO_UNIT_TYPE_MILLIMIETER = 3,
	IO_UNIT_TYPE_INCH = 4,
	IO_UNIT_TYPE_CUSTOM = 5
};

typedef struct {
	UnitType unit_type;
	float factor;
} UnitFactor;

const UnitFactor io_units_factor_export[] = {
		{IO_UNIT_TYPE_METER, 1.0},
		{IO_UNIT_TYPE_DECIMETER, 10.0},
		{IO_UNIT_TYPE_CENTIMETER, 100.0},
		{IO_UNIT_TYPE_MILLIMIETER, 1000.0},
		{IO_UNIT_TYPE_INCH, 39.37007874}
};

const UnitFactor io_units_factor_import[] = {
		{IO_UNIT_TYPE_METER, 1.0},
		{IO_UNIT_TYPE_DECIMETER, 0.1},
		{IO_UNIT_TYPE_CENTIMETER, 0.01},
		{IO_UNIT_TYPE_MILLIMIETER, 0.0010},
		{IO_UNIT_TYPE_INCH, 0.0254}
};

void io_ui_unit_type_import_update(Main* main, Scene* scene, PointerRNA* ptr);
void io_ui_unit_type_export_update(Main *main, Scene *scene, PointerRNA *ptr);
