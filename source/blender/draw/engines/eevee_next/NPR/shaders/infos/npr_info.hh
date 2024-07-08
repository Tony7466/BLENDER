/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "gpu_shader_create_info.hh"

GPU_SHADER_CREATE_INFO(npr_surface)
    .push_constant(Type::INT, "npr_index")
    .fragment_out(0, Type::VEC4, "out_color")
    .fragment_source("npr_surface_frag.glsl")
    .additional_info("draw_fullscreen")
    .define("Closure", "int");
