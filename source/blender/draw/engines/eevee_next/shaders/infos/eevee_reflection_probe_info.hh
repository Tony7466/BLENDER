/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Shared
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_reflection_probe_data)
    .storage_buf(REFLECTION_PROBE_BUF_SLOT,
                 Qualifier::READ,
                 "ReflectionProbeData",
                 "reflection_probe_buf[]")
    .sampler(REFLECTION_PROBE_TEX_SLOT, ImageType::FLOAT_2D_ARRAY, "reflectionProbes");

/* Sample cubemap and remap into an octahedral texture. */
GPU_SHADER_CREATE_INFO(eevee_reflection_probe_remap)
    .local_group_size(REFLECTION_PROBE_GROUP_SIZE, REFLECTION_PROBE_GROUP_SIZE)
    .push_constant(Type::INT, "reflection_probe_index")
    .storage_buf(REFLECTION_PROBE_BUF_SLOT,
                 Qualifier::READ,
                 "ReflectionProbeData",
                 "reflection_probe_buf[]")
    .sampler(0, ImageType::FLOAT_CUBE, "cubemap_tx")
    .image(0, GPU_RGBA16F, Qualifier::WRITE, ImageType::FLOAT_2D_ARRAY, "octahedral_img")
    .compute_source("eevee_reflection_probe_remap_comp.glsl")
    .additional_info("eevee_shared")
    .do_static_compilation(true);

/* Extract spherical harmonics band L0 + L1 from octahedral mapped reflection probe. */
GPU_SHADER_CREATE_INFO(eevee_reflection_probe_spherical_harmonics_extract)
    .local_group_size(1, 1, 1)
    .push_constant(Type::INT, "reflection_probe_index")
    .additional_info("eevee_shared", "eevee_reflection_probe_data")
    .compute_source("eevee_reflection_probe_spherical_harmonics_extract_comp.glsl")
    .do_static_compilation(true);

/** \} */
