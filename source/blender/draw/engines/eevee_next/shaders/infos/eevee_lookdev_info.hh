/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(eevee_lookdev_display)
.vertex_source("eevee_lookdev_display_vert.glsl")
.frag_source("eevee_lookdev_display_frag.glsl")
.do_static_compilation(true);
