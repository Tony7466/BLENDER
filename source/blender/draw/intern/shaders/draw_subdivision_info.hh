/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "draw_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(draw_object_infos)
    .typedef_source("draw_shader_shared.hh")
    .define("OBINFO_LIB")
    .define("OrcoTexCoFactors", "(drw_infos[resource_id].orco_mul_bias)")
    .define("ObjectInfo", "(drw_infos[resource_id].infos)")
    .define("ObjectColor", "(drw_infos[resource_id].ob_color)")
    .uniform_buf(DRW_OBJ_INFOS_UBO_SLOT,
                 "ObjectInfos",
                 "drw_infos[DRW_RESOURCE_CHUNK_LEN]",
                 Frequency::BATCH);
