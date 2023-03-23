/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_framebuffer.h"
#include "gpu_testing.hh"

#include "BLI_math_vector.hh"

namespace blender::gpu::tests {

static void test_framebuffer_clear_color_single_attachment()
{
  const int2 size(10, 10);
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_RGBA32F, usage, nullptr);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create(__func__);
  GPU_framebuffer_ensure_config(&framebuffer,
                                {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(texture)});
  GPU_framebuffer_bind(framebuffer);

  const float4 clear_color(0.1f, 0.2f, 0.5f, 1.0f);
  GPU_framebuffer_clear_color(framebuffer, clear_color);
  GPU_finish();

  float4 *read_data = static_cast<float4 *>(GPU_texture_read(texture, GPU_DATA_FLOAT, 0));
  for (float4 pixel_color : Span<float4>(read_data, size.x * size.y)) {
    EXPECT_EQ(pixel_color, clear_color);
  }
  MEM_freeN(read_data);

  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(texture);
}
GPU_TEST(framebuffer_clear_color_single_attachment);

static void test_framebuffer_clear_color_multiple_attachments()
{
  const int2 size(10, 10);
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture1 = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_RGBA32F, usage, nullptr);
  GPUTexture *texture2 = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_RGBA32F, usage, nullptr);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create(__func__);
  GPU_framebuffer_ensure_config(
      &framebuffer,
      {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(texture1), GPU_ATTACHMENT_TEXTURE(texture2)});
  GPU_framebuffer_bind(framebuffer);

  const float4 clear_color(0.1f, 0.2f, 0.5f, 1.0f);
  GPU_framebuffer_clear_color(framebuffer, clear_color);
  GPU_finish();

  float4 *read_data1 = static_cast<float4 *>(GPU_texture_read(texture1, GPU_DATA_FLOAT, 0));
  for (float4 pixel_color : Span<float4>(read_data1, size.x * size.y)) {
    EXPECT_EQ(pixel_color, clear_color);
  }
  MEM_freeN(read_data1);

  float4 *read_data2 = static_cast<float4 *>(GPU_texture_read(texture2, GPU_DATA_FLOAT, 0));
  for (float4 pixel_color : Span<float4>(read_data1, size.x * size.y)) {
    EXPECT_EQ(pixel_color, clear_color);
  }
  MEM_freeN(read_data2);

  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(texture1);
  GPU_texture_free(texture2);
}
GPU_TEST(framebuffer_clear_color_multiple_attachments);

static void test_framebuffer_clear_multiple_color_multiple_attachments()
{
  const int2 size(10, 10);
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture1 = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_RGBA32F, usage, nullptr);
  GPUTexture *texture2 = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_RGBA32F, usage, nullptr);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create(__func__);
  GPU_framebuffer_ensure_config(
      &framebuffer,
      {GPU_ATTACHMENT_NONE, GPU_ATTACHMENT_TEXTURE(texture1), GPU_ATTACHMENT_TEXTURE(texture2)});
  GPU_framebuffer_bind(framebuffer);

  const float4 clear_color[2] = {float4(0.1f, 0.2f, 0.5f, 1.0f), float4(0.5f, 0.2f, 0.1f, 1.0f)};
  GPU_framebuffer_multi_clear(
      framebuffer, static_cast<const float(*)[4]>(static_cast<const void *>(clear_color)));
  GPU_finish();

  float4 *read_data1 = static_cast<float4 *>(GPU_texture_read(texture1, GPU_DATA_FLOAT, 0));
  for (float4 pixel_color : Span<float4>(read_data1, size.x * size.y)) {
    EXPECT_EQ(pixel_color, clear_color[0]);
  }
  MEM_freeN(read_data1);

  float4 *read_data2 = static_cast<float4 *>(GPU_texture_read(texture2, GPU_DATA_FLOAT, 0));
  for (float4 pixel_color : Span<float4>(read_data1, size.x * size.y)) {
    EXPECT_EQ(pixel_color, clear_color[1]);
  }
  MEM_freeN(read_data2);

  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(texture1);
  GPU_texture_free(texture2);
}
GPU_TEST(framebuffer_clear_multiple_color_multiple_attachments);

static void test_framebuffer_clear_depth()
{
  const int2 size(10, 10);
  eGPUTextureUsage usage = GPU_TEXTURE_USAGE_ATTACHMENT | GPU_TEXTURE_USAGE_HOST_READ;
  GPUTexture *texture = GPU_texture_create_2d(
      __func__, UNPACK2(size), 1, GPU_DEPTH_COMPONENT32F, usage, nullptr);

  GPUFrameBuffer *framebuffer = GPU_framebuffer_create(__func__);
  GPU_framebuffer_ensure_config(&framebuffer, {GPU_ATTACHMENT_TEXTURE(texture)});
  GPU_framebuffer_bind(framebuffer);

  const float clear_depth = 0.5f;
  GPU_framebuffer_clear_depth(framebuffer, clear_depth);
  GPU_finish();

  float *read_data = static_cast<float *>(GPU_texture_read(texture, GPU_DATA_FLOAT, 0));
  for (float pixel_depth : Span<float>(read_data, size.x * size.y)) {
    EXPECT_EQ(pixel_depth, clear_depth);
  }
  MEM_freeN(read_data);

  GPU_framebuffer_free(framebuffer);
  GPU_texture_free(texture);
}
GPU_TEST(framebuffer_clear_depth);

}  // namespace blender::gpu::tests
