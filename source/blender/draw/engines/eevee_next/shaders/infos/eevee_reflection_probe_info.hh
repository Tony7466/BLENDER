#include "eevee_defines.hh"
#include "gpu_shader_create_info.hh"

/* -------------------------------------------------------------------- */
/** \name Shared
 * \{ */

GPU_SHADER_CREATE_INFO(eevee_reflection_probe_data)
    .sampler(REFLECTION_PROBE_TEX_SLOT, ImageType::FLOAT_2D_ARRAY, "reflectionProbes");

/* Read cubemap and store into octahedral texture. */
GPU_SHADER_CREATE_INFO(eevee_reflection_probe_remap)
    .local_group_size(REFLECTION_PROBE_GROUP_SIZE, REFLECTION_PROBE_GROUP_SIZE)
    .sampler(REFLECTION_PROBE_TEX_SLOT, ImageType::FLOAT_CUBE, "cubemap_tx")
    .image(REFLECTION_PROBE_OCTAHEDRAL_SLOT,
           GPU_RGBA16F,
           Qualifier::WRITE,
           ImageType::FLOAT_2D_ARRAY,
           "octahedral_img")
    .compute_source("eevee_reflection_probe_remap_comp.glsl")
    .do_static_compilation(true);

/** \} */
