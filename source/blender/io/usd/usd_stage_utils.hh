/* SPDX-FileCopyrightText: 2023 NVIDIA Corporation. All rights reserved.
*
* SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include <pxr/usd/usd/stage.h>
#include "DNA_usd_stage_types.h"

void usd_stage_clear_cache();
pxr::UsdStageRefPtr get_pxr_stage_for_path(const char* filepath);
pxr::UsdStageRefPtr usd_stage_get_pxr_stage(const USDStage* stage);
bool usd_stage_remove_from_cache(const char* filepath);
