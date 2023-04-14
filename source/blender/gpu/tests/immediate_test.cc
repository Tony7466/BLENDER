/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_framebuffer.h"
#include "GPU_immediate.h"
#include "GPU_shader_builtin.h"
#include "gpu_testing.hh"

#include "BLI_math_vector.hh"

namespace blender::gpu::tests {

static constexpr int Size = 10;

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

  immBindBuiltinProgram(GPU_SHADER_3D_POLYLINE_UNIFORM_COLOR);
  float2 viewport_size(Size);
  float4 color(1.0, 0.5, 0.25, 1.0);
  immUniform2fv("viewportSize", viewport_size);
  immUniform1f("lineWidth", 1.0f);
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

}  // namespace blender::gpu::tests
