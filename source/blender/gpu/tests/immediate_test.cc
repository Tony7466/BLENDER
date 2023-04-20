/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_shader_builtin.h"
#include "gpu_testing.hh"

#include "BLI_math_vector.hh"

namespace blender::gpu::tests {

static constexpr int Size = 256;

static void test_immediate_polyline()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);
  float4 color(1.0f, 0.5f, 0.25f, 1.0f);
  immUniform1f("lineWidth", 5.0f);
  immUniform2f("viewportSize", Size, Size);
  immUniformColor4fv(color);
  immBegin(GPU_PRIM_LINES, 4);
  immVertex3f(pos, -1.0f, -1.0f, 0.0f);
  immVertex3f(pos, 1.0f, 1.0f, 0.0f);
  immVertex3f(pos, -1.0f, 1.0f, 0.0f);
  immVertex3f(pos, 1.0f, -1.0f, 0.0f);
  immEnd();

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  GPU_offscreen_free(offscreen);
}
GPU_TEST(immediate_polyline)

static void test_immediate_crosshair()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  float4 color(1.0, 0.5, 0.25, 1.0);
  immUniformColor4fv(color);
  immBegin(GPU_PRIM_LINES, 4);
  immVertex3f(pos, -1.0f, 0.0f, 0.0f);
  immVertex3f(pos, 1.0f, 0.0f, 0.0f);
  immVertex3f(pos, 0.0f, -1.0f, 0.0f);
  immVertex3f(pos, 0.0f, 1.0f, 0.0f);
  immEnd();

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  GPU_offscreen_free(offscreen);
}
GPU_TEST(immediate_crosshair)

/**
 * This test overflows the default immediate buffer and a second one would be allocated.
 */
static void test_immediate_arrow()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);
  GPU_depth_test(GPU_DEPTH_GREATER);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  /* Creates an immediate buffer and fill a part of the buffer. */
  float4 color(1.0, 0.5, 0.25, 1.0);
  immUniformColor4fv(color);
  imm_draw_circle_fill_3d(pos, 0.0, 0.0, 0.8f, 256);

  /* The cylinder doesn't fit in the immediate buffer and a new one will be constructed. */
  float4 color2(1.0, 0.0, 0.0, 1.0);
  immUniformColor4fv(color2);
  imm_draw_cylinder_fill_3d(pos, 1.0f, 0.0, 1.0f, 256, 256);

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  GPU_offscreen_free(offscreen);
}
GPU_TEST(immediate_arrow)

static void test_immediate_one_plane()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  float4 color(1.0, 0.5, 0.25, 1.0);
  immUniformColor4fv(color);
  immBegin(GPU_PRIM_TRI_STRIP, 4);
  immVertex3f(pos, -1.0f, 1.0f, 0.0f);
  immVertex3f(pos, 1.0f, 1.0f, 0.0f);
  immVertex3f(pos, -1.0f, -1.0f, 0.0f);
  immVertex3f(pos, 1.0f, -1.0f, 0.0f);
  immEnd();

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  /* Read back data and perform some basic tests. */
  float read_data[4 * Size * Size];
  GPU_offscreen_read_color(offscreen, GPU_DATA_FLOAT, &read_data);
  for (int pixel_index = 0; pixel_index < Size * Size; pixel_index++) {
    float4 read_color = float4(&read_data[pixel_index * 4]);
    EXPECT_EQ(read_color, color);
  }

  GPU_offscreen_free(offscreen);
}
GPU_TEST(immediate_one_plane)

/**
 * Draws two planes with two different colors.
 * - Tests that both planes are stored in the same buffer (depends on backend).
 * - Test that data of the first plane isn't overwritten by the second plane.
 *   (push constants, buffer, bind points etc.)
 */
static void test_immediate_two_planes()
{
  GPUOffScreen *offscreen = GPU_offscreen_create(Size,
                                                 Size,
                                                 false,
                                                 GPU_RGBA16F,
                                                 GPU_TEXTURE_USAGE_ATTACHMENT |
                                                     GPU_TEXTURE_USAGE_HOST_READ,
                                                 nullptr);
  BLI_assert(offscreen != nullptr);
  GPU_offscreen_bind(offscreen, false);

  GPUVertFormat *format = immVertexFormat();
  uint pos = GPU_vertformat_attr_add(format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);

  immBindBuiltinProgram(GPU_SHADER_3D_UNIFORM_COLOR);

  float4 color(1.0, 0.5, 0.25, 1.0);
  immUniformColor4fv(color);
  immBegin(GPU_PRIM_TRI_STRIP, 4);
  immVertex3f(pos, -1.0f, 1.0f, 0.0f);
  immVertex3f(pos, 0.0f, 1.0f, 0.0f);
  immVertex3f(pos, -1.0f, -1.0f, 0.0f);
  immVertex3f(pos, 0.0f, -1.0f, 0.0f);
  immEnd();

  float4 color2(0.25, 0.5, 1.0, 1.0);
  immUniformColor4fv(color2);
  immBegin(GPU_PRIM_TRI_STRIP, 4);
  immVertex3f(pos, 0.0f, 1.0f, 0.0f);
  immVertex3f(pos, 1.0f, 1.0f, 0.0f);
  immVertex3f(pos, 0.0f, -1.0f, 0.0f);
  immVertex3f(pos, 1.0f, -1.0f, 0.0f);
  immEnd();

  GPU_offscreen_unbind(offscreen, false);
  GPU_flush();

  /* Read back data and perform some basic tests.
   * Not performing detailed tests as there might be driver specific limitations. */
  float read_data[4 * Size * Size];
  GPU_offscreen_read_color(offscreen, GPU_DATA_FLOAT, &read_data);
  int64_t color_num = 0;
  int64_t color2_num = 0;
  for (int pixel_index = 0; pixel_index < Size * Size; pixel_index++) {
    float4 read_color = float4(&read_data[pixel_index * 4]);
    if (read_color == color) {
      color_num++;
    }
    else if (read_color == color2) {
      color2_num++;
    }
    else {
      EXPECT_TRUE(read_color == color || read_color == color2);
    }
  }
  EXPECT_TRUE(color_num > 0);
  EXPECT_TRUE(color2_num > 0);

  GPU_offscreen_free(offscreen);
}
GPU_TEST(immediate_two_planes)

}  // namespace blender::gpu::tests
