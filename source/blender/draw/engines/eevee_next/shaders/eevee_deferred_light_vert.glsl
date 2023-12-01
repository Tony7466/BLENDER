/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/**
 * This pass load Gbuffer data and output a mask of tiles to process.
 * This mask is then processed by the compaction phase.
 */

#pragma BLENDER_REQUIRE(gpu_shader_utildefines_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_math_vector_lib.glsl)
#pragma BLENDER_REQUIRE(gpu_shader_codegen_lib.glsl)
#pragma BLENDER_REQUIRE(eevee_gbuffer_lib.glsl)

void main()
{
  int tiles_per_row = imageSize(tile_mask_img).x;

  ivec2 tile_co = ivec2(gpu_Instance % tiles_per_row, gpu_Instance / tiles_per_row);

  eClosureBits tile_closure = eClosureBits(imageLoad(tile_mask_img, tile_co).x);

  if ((tile_closure & active_closure) == eClosureBits(0u)) {
    gl_Position = vec4(0.0);
    return;
  }

  /* TODO */
  gl_Position = vec4(tile_co, 0.0, 1.0);
}
