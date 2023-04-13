/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "gpu_testing.hh"

#include "GPU_batch.h"
#include "GPU_batch_presets.h"
#include "GPU_framebuffer.h"
#include "GPU_matrix.h"

#include "BLI_math_vector.hh"

#include "intern/draw_cache.h"

namespace blender::gpu::tests {
const size_t Size = 512;

static void test_offscreen_draw_batch_quad()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size, Size, false, GPU_RGBA16F, nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);
  /*
  GPUTexture *color_texture = GPU_offscreen_color_texture(offscreen);
  float4 clear_color(0.0f);
  GPU_texture_clear(color_texture, GPU_DATA_FLOAT, clear_color);
  */

  GPUBatch *batch = DRW_cache_quad_get();
  float4 color(1.0f, 0.5f, 0.0f, 1.0f);

  GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_UNIFORM_COLOR);
  GPU_batch_uniform_4fv(batch, "color", color);

  GPU_matrix_push();
  GPU_matrix_scale_1f(0.5f);
  GPU_matrix_rotate_2d(45.0f);

  GPU_batch_draw(batch);
  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();
  GPU_matrix_pop();

  GPU_offscreen_free(offscreen);
  DRW_shape_cache_free();
}
GPU_TEST(offscreen_draw_batch_quad);

static void test_offscreen_draw_batch_sphere()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size, Size, false, GPU_RGBA16F, nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);

  GPUBatch *batch = DRW_cache_sphere_get(DRW_LOD_MEDIUM);
  float4 color(1.0f, 0.5f, 0.0f, 0.8f);

  GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_UNIFORM_COLOR);
  GPU_batch_uniform_4fv(batch, "color", color);
  GPU_blend(GPU_BLEND_ALPHA);

  GPU_matrix_push();
  GPU_matrix_scale_1f(0.5f);
  GPU_matrix_rotate_2d(45.0f);

  GPU_batch_draw(batch);
  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();
  GPU_matrix_pop();

  GPU_offscreen_free(offscreen);
  DRW_shape_cache_free();
}
GPU_TEST(offscreen_draw_batch_sphere);

template<eGPUBlend blend_type>
void offscreen_blending_test(float4 source_a, float4 source_b, float4 expected_result)
{
  GPUOffScreen *offscreen = GPU_offscreen_create(1, 1, false, GPU_RGBA16F, nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);
  GPUTexture *color_texture = GPU_offscreen_color_texture(offscreen);
  GPU_texture_clear(color_texture, GPU_DATA_FLOAT, source_a);

  GPUBatch *batch = DRW_cache_quad_get();

  GPU_batch_program_set_builtin(batch, GPU_SHADER_3D_UNIFORM_COLOR);
  GPU_batch_uniform_4fv(batch, "color", source_b);
  GPU_blend(blend_type);

  GPU_batch_draw(batch);
  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  float4 read_back;
  GPU_memory_barrier(GPU_BARRIER_TEXTURE_FETCH);
  GPU_offscreen_read_pixels(offscreen, GPU_DATA_FLOAT, &read_back);
  EXPECT_EQ(read_back, expected_result);

  GPU_offscreen_free(offscreen);
  DRW_shape_cache_free();
}

static void test_blending_none()
{
  offscreen_blending_test<GPU_BLEND_NONE>(float4(1.0f, 0.0f, 1.0f, 1.0f),
                                          float4(0.0f, 1.0f, 0.0f, 0.5f),
                                          float4(0.0f, 1.0f, 0.0f, 0.5f));
}
GPU_TEST(blending_none)

}  // namespace blender::gpu::tests