/* SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <pxr/usd/usd/stage.h>

#include <string>

struct Depsgraph;
struct ExportJobData;
struct USDExportParams;

namespace blender::io::usd {

/* Ensure classes and type converters necessary for invoking export hook are registered. */
void register_export_hook_converters();

/* Call the 'on_export' chaser function defined in the registred USDHook classes. */
void call_export_hooks(pxr::UsdStageRefPtr stage, Depsgraph *depsgraph);

}  // namespace blender::io::usd
