/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "GPU_capabilities.h"
#include "GPU_compute.h"
#include "GPU_shader.h"
#include "GPU_storage_buffer.h"

#include "BLI_math_vector.hh"
#include "BLI_vector.hh"

#include "gpu_testing.hh"

namespace blender::gpu::tests {

static void push_constants(const char *info_name)
{
  if (!GPU_compute_shader_support() && !GPU_shader_storage_buffer_objects_support()) {
    /* We can't test as a the platform does not support compute shaders. */
    std::cout << "Skipping test: platform not supported";
    return;
  }

  static constexpr uint SIZE = 16;

  /* Build compute shader. */
  GPUShader *shader = GPU_shader_create_from_info_name(info_name);
  EXPECT_NE(shader, nullptr);
  GPU_shader_bind(shader);

  /* Construct IBO. */
  GPUStorageBuf *ssbo = GPU_storagebuf_create_ex(
      SIZE * sizeof(float), nullptr, GPU_USAGE_DEVICE_ONLY, __func__);
  GPU_storagebuf_bind(ssbo, GPU_shader_get_ssbo_binding(shader, "data_out"));

  const float float_in = 10.0f;
  const float2 vec2_in(20.0f, 21.0f);
  const float3 vec3_in(30.0f, 31.0f, 32.0f);
  const float4 vec4_in(40.0f, 41.0f, 42.0f, 43.0f);
  GPU_shader_uniform_1f(shader, "float_in", float_in);
  GPU_shader_uniform_2fv(shader, "vec2_in", vec2_in);
  GPU_shader_uniform_3fv(shader, "vec3_in", vec3_in);
  GPU_shader_uniform_4fv(shader, "vec4_in", vec4_in);

  /* Dispatch compute task. */
  GPU_compute_dispatch(shader, 1, 1, 1);

  /* Check if compute has been done. */
  GPU_memory_barrier(GPU_BARRIER_SHADER_STORAGE);

  /* Download the index buffer. */
  float data[SIZE];
  GPU_storagebuf_read(ssbo, data);

  /* Check the results. */
  EXPECT_EQ(data[0], float_in);
  EXPECT_EQ(data[1], vec2_in.x);
  EXPECT_EQ(data[2], vec2_in.y);
  EXPECT_EQ(data[3], vec3_in.x);
  EXPECT_EQ(data[4], vec3_in.y);
  EXPECT_EQ(data[5], vec3_in.z);
  EXPECT_EQ(data[6], vec4_in.x);
  EXPECT_EQ(data[7], vec4_in.y);
  EXPECT_EQ(data[8], vec4_in.z);
  EXPECT_EQ(data[9], vec4_in.w);

  /* Cleanup. */
  GPU_shader_unbind();
  GPU_storagebuf_free(ssbo);
  GPU_shader_free(shader);
}

static void test_push_constants_packed()
{
  push_constants("gpu_push_constants_packed_test");
}
GPU_TEST(push_constants_packed)

}  // namespace blender::gpu::tests